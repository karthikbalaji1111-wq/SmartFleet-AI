"""Task state machine for container storage, retrieval, and perception-guided pickup.

Controls the sequential staging, visual target acquisition, closed-loop fine alignment,
physical fork insertion, multi-stage pickup verification, rack clearance, and transport.
"""

from __future__ import annotations

import math
import threading
from typing import Any, Literal, Optional

from pydantic import BaseModel

from .alignment import (
    AlignmentController,
    AlignmentTolerances,
    TargetAcquisition,
    TargetObservation,
    calculate_staging_pose,
    calculate_target_relative_pose,
    derive_fork_insertion_target,
    derive_pickup_orientation,
    world_to_robot_body,
)
from .config import get_config
from .perception import PerceptionSystem
from .robot_model import fork_surface_height
from .state import BodyState, RobotState

TaskState = Literal[
    "idle",
    "approach_source",
    "acquire_target",
    "align_pickup",
    "raise_to_source",
    "extend_to_source",
    "lift_source",
    "verify_pickup",
    "retract_source",
    "back_away_source",
    "lower_source",
    "approach_dest",
    "raise_to_dest",
    "extend_to_dest",
    "lower_dest",
    "retract_dest",
    "back_away_dest",
    "completed",
    "failed",
    "cancelled",
]


class TaskRequest(BaseModel):
    container_id: str
    destination: str
    priority: int = 0
    type: Literal["storage", "retrieval"] = "storage"


class PickupTelemetry(BaseModel):
    target_id: Optional[str] = None
    stage: str = "idle"
    lateral_error_cm: Optional[float] = None
    heading_error_deg: Optional[float] = None
    distance_m: Optional[float] = None
    alignment_status: Optional[str] = None
    verified_pickup: bool = False
    cargo_state: Literal["empty", "attached", "uncertain"] = "empty"
    failure_reason: Optional[str] = None


class TaskTelemetry(BaseModel):
    state: TaskState
    active_task: TaskRequest | None
    error: str | None
    pickup: Optional[PickupTelemetry] = None


class TaskManager:
    """Orchestrates container handling workflows with perception-guided alignment."""

    def __init__(self, world, perception_system: Optional[PerceptionSystem] = None) -> None:
        self.world = world
        self.state: TaskState = "idle"
        self.task: TaskRequest | None = None
        self.error: str | None = None
        self._wait_time = 0.0

        # Perception and alignment components
        self.perception_system = perception_system
        self.target_acq = TargetAcquisition(self.world.config, max_observation_age=2.0)
        self.alignment_controller = AlignmentController()
        self.pickup_telemetry_data = PickupTelemetry()

        # Operational tracking variables
        self._acquire_timer = 0.0
        self._initial_container_z = 0.0
        self._src_location_id = ""
        self._src_z = 0.0
        self._dest_z = 0.0
        self._insertion_lift_tgt = 0.0
        self._insertion_fork_tgt = 0.58
        self._latest_obs: Optional[TargetObservation] = None
        self._desired_heading = 0.0
        self._back_away_timer = 0.0

    def set_perception_system(self, ps: PerceptionSystem) -> None:
        self.perception_system = ps

    def _get_perception_result(self) -> Any:
        """Fetch latest perception result from system or capture on demand."""
        if self.perception_system is not None:
            return self.perception_system.capture_and_process()
        # Fallback if no perception system was injected: create on-demand if world initialized
        if getattr(self.world, "is_initialized", False):
            lock = getattr(self.world, "_lock", None) or threading.RLock()
            self.perception_system = PerceptionSystem(self.world, lock)
            return self.perception_system.capture_and_process()
        return None

    def _find_location(self, loc_id: str):
        config = self.world.config.warehouse
        for s in config.stations:
            if s.id == loc_id:
                return s
        for s in config.slots:
            if s.id == loc_id:
                return s
        return None

    def _get_approach_pose(self, loc):
        cx, cy = loc.center[0], loc.center[1]
        if cx > 8.0:
            return cx - 0.9, cy, 0.0  # Face east
        aisles = [3.75, 0.25, -3.25]
        best_aisle_y = min(aisles, key=lambda ay: abs(ay - cy))
        if cy > best_aisle_y:
            yaw = math.pi / 2.0  # Face north
            ay = cy - 0.9
        else:
            yaw = -math.pi / 2.0  # Face south
            ay = cy + 0.9
        return cx, ay, yaw

    def _calc_lift_target(self, desired_surface_z: float) -> float:
        baseline = fork_surface_height(self.world.config.robot, 0.0)
        return max(0.0, desired_surface_z - baseline)

    def submit(self, request: TaskRequest) -> None:
        if self.state not in ("idle", "completed", "failed", "cancelled"):
            raise ValueError(f"Cannot submit task while {self.state}")

        if request.container_id not in self.world.container_ids:
            raise ValueError(f"Unknown container {request.container_id}")

        dest_loc = self._find_location(request.destination)
        if not dest_loc:
            raise ValueError(f"Unknown destination {request.destination}")

        # Check occupancy
        for cid, loc_id in getattr(self.world, "container_locations", {}).items():
            if cid != request.container_id and loc_id == request.destination:
                raise ValueError(f"Destination {request.destination} is already occupied by {cid}")

        self.task = request
        self.error = None
        self.alignment_controller.reset()
        self._latest_obs = None
        self._acquire_timer = 0.0

        # Find container's current location in warehouse
        c_state = next(c for c in self.world.get_container_states() if c.id == request.container_id)
        self._initial_container_z = c_state.position[2]

        config = self.world.config.warehouse
        all_locs = list(config.stations) + list(config.slots)
        src_loc = min(
            all_locs,
            key=lambda s: math.hypot(s.center[0] - c_state.position[0], s.center[1] - c_state.position[1]),
        )
        self._src_location_id = src_loc.id
        self._src_z = src_loc.size[2] if hasattr(src_loc, "size") else src_loc.center[2]
        self._dest_z = dest_loc.size[2] if hasattr(dest_loc, "size") else dest_loc.center[2]

        # Calculate safe staging pose with 1.0m standoff
        ax, ay, ayaw = calculate_staging_pose(self._src_location_id, config, standoff_distance=1.0)

        self.state = "approach_source"
        self.pickup_telemetry_data = PickupTelemetry(
            target_id=request.container_id,
            stage="approach_source",
            cargo_state="empty",
        )

        dest = self.world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=0.5)
        self.world.plan_navigation(dest)
        self.world.start_navigation()

    def cancel(self) -> None:
        if self.state in ("idle", "completed", "failed", "cancelled"):
            return
        self.world.cancel_navigation("task cancelled")
        self.world.stop()
        self.world.stop_mechanism("lift")
        self.world.stop_mechanism("forks")
        self.alignment_controller.reset()
        self.state = "cancelled"
        self.pickup_telemetry_data.stage = "cancelled"
        self.pickup_telemetry_data.failure_reason = "Task cancelled"
        self.task = None

    def tick(self, dt: float) -> None:
        if self.state in ("idle", "completed", "failed", "cancelled"):
            return

        r = self.world.get_robot_state()

        # --------------------------------------------------------- #
        # Stage 1: Navigate to safe staging position
        # --------------------------------------------------------- #
        if self.state == "approach_source":
            if self.world.navigator.status == "arrived":
                # Clean controller handoff: reset navigation ownership
                self.world.navigator.reset()
                self.world.stop()
                self._acquire_timer = 0.0
                # Position lift carriage to target tier height so camera has unobstructed view
                view_lift_tgt = self._calc_lift_target(self._src_z + 0.05)
                self.world.set_mechanism_target("lift", view_lift_tgt)
                self.state = "acquire_target"
                self.pickup_telemetry_data.stage = "acquire_target"
            elif self.world.navigator.status in ("failed", "cancelled"):
                self.state = "failed"
                self.error = f"Navigation to staging position failed: {self.world.navigator.reason}"
                self.pickup_telemetry_data.failure_reason = self.error

        # --------------------------------------------------------- #
        # Stage 2: Visual Target Acquisition
        # --------------------------------------------------------- #
        elif self.state == "acquire_target":
            self._acquire_timer += dt
            perc = self._get_perception_result()
            obs, err = self.target_acq.acquire_target(self.task.container_id, perc, self.world.sim_time)
            if obs is not None:
                self._latest_obs = obs
                self.alignment_controller.reset()
                self._desired_heading = derive_pickup_orientation(
                    self._src_location_id, self.world.config.warehouse
                )
                self.state = "align_pickup"
                self.pickup_telemetry_data.stage = "align_pickup"
                self.pickup_telemetry_data.alignment_status = "Target acquired; starting alignment"
            elif self._acquire_timer > 6.0:
                self.state = "failed"
                self.error = f"Target acquisition failed: {err}"
                self.pickup_telemetry_data.failure_reason = self.error

        # --------------------------------------------------------- #
        # Stage 3: Closed-loop fine alignment
        # --------------------------------------------------------- #
        elif self.state == "align_pickup":
            # Continuously acquire fresh observations
            perc = self._get_perception_result()
            obs, err = self.target_acq.acquire_target(self.task.container_id, perc, self.world.sim_time)
            if obs is not None:
                self._latest_obs = obs

            rel_pose = None
            if self._latest_obs and (self.world.sim_time - self._latest_obs.timestamp) <= 2.0:
                rel_pose = calculate_target_relative_pose(
                    r.pose.position[0],
                    r.pose.position[1],
                    r.pose.position[2],
                    r.pose.heading,
                    self._latest_obs,
                    self._desired_heading,
                )
                self.pickup_telemetry_data.lateral_error_cm = rel_pose.y_rel * 100.0
                self.pickup_telemetry_data.heading_error_deg = math.degrees(rel_pose.orientation_error)
                self.pickup_telemetry_data.distance_m = rel_pose.x_rel

            v, w, is_aligned, status_msg = self.alignment_controller.update(rel_pose, dt)
            self.pickup_telemetry_data.alignment_status = status_msg

            if is_aligned:
                self.world.stop()
                lift_tgt, fork_tgt = derive_fork_insertion_target(
                    self.world.config.robot,
                    container_height=0.22,
                    shelf_surface_z=self._src_z,
                    insertion_depth=0.60,
                )
                self._insertion_lift_tgt = lift_tgt
                self._insertion_fork_tgt = fork_tgt
                self.world.set_mechanism_target("lift", lift_tgt)
                self.state = "raise_to_source"
                self.pickup_telemetry_data.stage = "raise_to_source"
            elif self.alignment_controller.state.phase == "failed":
                self.world.stop()
                self.state = "failed"
                self.error = f"Fine alignment failed: {self.alignment_controller.state.failure_reason}"
                self.pickup_telemetry_data.failure_reason = self.error
            else:
                self.world.set_velocity_command(v, w, duration=0.2)

        # --------------------------------------------------------- #
        # Stage 4: Lift mechanism to insertion height
        # --------------------------------------------------------- #
        elif self.state == "raise_to_source":
            if r.lift.fault:
                self.state = "failed"
                self.error = f"Lift faulted during height positioning: {r.lift.fault}"
                self.pickup_telemetry_data.failure_reason = self.error
                return
            if r.lift.at_target:
                self.world.set_mechanism_target("forks", self._insertion_fork_tgt)
                self.state = "extend_to_source"
                self.pickup_telemetry_data.stage = "extend_to_source"

        # --------------------------------------------------------- #
        # Stage 5: Extend forks under container
        # --------------------------------------------------------- #
        elif self.state == "extend_to_source":
            if r.forks.fault:
                self.state = "failed"
                self.error = f"Forks faulted during extension: {r.forks.fault}"
                self.pickup_telemetry_data.failure_reason = self.error
                return
            if r.forks.at_target:
                # Attach container physically in PyBullet
                self.world.attach_container(self.task.container_id)
                # Elevate container off shelf (15cm above shelf surface)
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._src_z + 0.15))
                self.state = "lift_source"
                self.pickup_telemetry_data.stage = "lift_source"

        # --------------------------------------------------------- #
        # Stage 6: Lift container off shelf
        # --------------------------------------------------------- #
        elif self.state == "lift_source":
            if r.lift.fault:
                self.state = "failed"
                self.error = f"Lift faulted while lifting container: {r.lift.fault}"
                self.pickup_telemetry_data.failure_reason = self.error
                return
            if r.lift.at_target:
                self.state = "verify_pickup"
                self.pickup_telemetry_data.stage = "verify_pickup"

        # --------------------------------------------------------- #
        # Stage 7: Physical Pickup Verification
        # --------------------------------------------------------- #
        elif self.state == "verify_pickup":
            has_constraint = self.task.container_id in self.world._cargo_constraints
            c_states = self.world.get_container_states()
            c_state = next((c for c in c_states if c.id == self.task.container_id), None)

            if not c_state:
                self.state = "failed"
                self.error = "Physical verification failed: container missing from simulation"
                self.pickup_telemetry_data.cargo_state = "uncertain"
                self.pickup_telemetry_data.failure_reason = self.error
                return

            elevation = c_state.position[2] - self._initial_container_z
            elevated_ok = elevation >= 0.05  # elevated >= 5cm above original support

            if not has_constraint:
                self.state = "failed"
                self.error = "Physical verification failed: constraint not established"
                self.pickup_telemetry_data.cargo_state = "empty"
                self.pickup_telemetry_data.failure_reason = self.error
                return

            if not elevated_ok:
                self.state = "failed"
                self.error = (
                    f"Physical verification failed: insufficient elevation "
                    f"({elevation*100:.1f}cm < 5.0cm)"
                )
                self.pickup_telemetry_data.cargo_state = "uncertain"
                self.pickup_telemetry_data.failure_reason = self.error
                return

            # Verification successful!
            self.pickup_telemetry_data.verified_pickup = True
            self.pickup_telemetry_data.cargo_state = "attached"
            self.world.set_mechanism_target("forks", 0.0)
            self.state = "retract_source"
            self.pickup_telemetry_data.stage = "retract_source"

        # --------------------------------------------------------- #
        # Stage 8: Retract forks with attached container
        # --------------------------------------------------------- #
        elif self.state == "retract_source":
            if r.forks.fault:
                self.state = "failed"
                self.error = f"Forks faulted during retraction: {r.forks.fault}"
                self.pickup_telemetry_data.failure_reason = self.error
                return
            if r.forks.at_target:
                # Confirm container remains attached
                if self.task.container_id not in self.world._cargo_constraints:
                    self.state = "failed"
                    self.error = "Cargo detached during fork retraction"
                    self.pickup_telemetry_data.cargo_state = "uncertain"
                    self.pickup_telemetry_data.failure_reason = self.error
                    return
                self._back_away_timer = 2.0  # Drive back for 2s at -0.4 m/s (0.8m)
                self.state = "back_away_source"
                self.pickup_telemetry_data.stage = "back_away_source"

        # --------------------------------------------------------- #
        # Stage 9: Back away to clear rack face
        # --------------------------------------------------------- #
        elif self.state == "back_away_source":
            self.world.set_velocity_command(-0.4, 0.0)
            self._back_away_timer -= dt
            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0)  # Lower to safe travel height
                self.state = "lower_source"
                self.pickup_telemetry_data.stage = "lower_source"

        # --------------------------------------------------------- #
        # Stage 10: Lower to travel height and transition to destination
        # --------------------------------------------------------- #
        elif self.state == "lower_source":
            if r.lift.at_target:
                dest_loc = self._find_location(self.task.destination)
                ax, ay, ayaw = self._get_approach_pose(dest_loc)
                dest = self.world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=0.5)
                self.world.plan_navigation(dest)
                self.world.start_navigation()
                self.state = "approach_dest"
                self.pickup_telemetry_data.stage = "approach_dest"

        # --------------------------------------------------------- #
        # Downstream Destination Placement (Storage / Retrieval)
        # --------------------------------------------------------- #
        elif self.state == "approach_dest":
            if self.world.navigator.status == "arrived":
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z + 0.15))
                self.state = "raise_to_dest"
                self.pickup_telemetry_data.stage = "raise_to_dest"
            elif self.world.navigator.status in ("failed", "cancelled"):
                self.state = "failed"
                self.error = f"Navigation to destination failed: {self.world.navigator.reason}"
                self.pickup_telemetry_data.failure_reason = self.error

        elif self.state == "raise_to_dest":
            if r.lift.at_target:
                self.world.set_mechanism_target("forks", 0.6)
                self.state = "extend_to_dest"
                self.pickup_telemetry_data.stage = "extend_to_dest"

        elif self.state == "extend_to_dest":
            if r.forks.at_target:
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z + 0.05))
                self.state = "lower_dest"
                self.pickup_telemetry_data.stage = "lower_dest"

        elif self.state == "lower_dest":
            if r.lift.at_target:
                self.world.detach_container(self.task.container_id)
                self.pickup_telemetry_data.cargo_state = "empty"
                self.world.set_mechanism_target("forks", 0.0)
                self.state = "retract_dest"
                self.pickup_telemetry_data.stage = "retract_dest"

        elif self.state == "retract_dest":
            if r.forks.at_target:
                self._back_away_timer = 2.0
                self.state = "back_away_dest"
                self.pickup_telemetry_data.stage = "back_away_dest"

        elif self.state == "back_away_dest":
            self.world.set_velocity_command(-0.4, 0.0)
            self._back_away_timer -= dt
            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0)
                if hasattr(self.world, "container_locations") and self.task:
                    self.world.container_locations[self.task.container_id] = self.task.destination
                self.state = "completed"
                self.pickup_telemetry_data.stage = "completed"
                self.task = None

    def telemetry(self) -> TaskTelemetry:
        return TaskTelemetry(
            state=self.state,
            active_task=self.task,
            error=self.error,
            pickup=self.pickup_telemetry_data,
        )
