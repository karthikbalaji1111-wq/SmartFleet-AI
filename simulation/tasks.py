import math
from typing import Literal

from pydantic import BaseModel
from .state import RobotState, BodyState
from .robot_model import fork_surface_height

TaskState = Literal[
    "idle",
    "approach_source",
    "raise_to_source",
    "extend_to_source",
    "lift_source",
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
    "failed"
]

class TaskRequest(BaseModel):
    container_id: str
    destination: str

class TaskTelemetry(BaseModel):
    state: TaskState
    active_task: TaskRequest | None
    error: str | None

class TaskManager:
    def __init__(self, world):
        self.world = world
        self.state: TaskState = "idle"
        self.task: TaskRequest | None = None
        self.error: str | None = None
        self._wait_time = 0.0
        
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
        
        # Stations (inbound/outbound) are at x=10.4.
        if cx > 8.0:
            return cx - 0.9, cy, 0.0 # Face east
            
        # Racks
        # Aisle y coordinates: 3.75, 0.25, -3.25
        aisles = [3.75, 0.25, -3.25]
        best_aisle_y = min(aisles, key=lambda ay: abs(ay - cy))
        
        if cy > best_aisle_y:
            yaw = math.pi / 2 # Face north
            ay = cy - 0.9
        else:
            yaw = -math.pi / 2 # Face south
            ay = cy + 0.9
            
        return cx, ay, yaw


    def _calc_lift_target(self, desired_surface_z: float) -> float:
        baseline = fork_surface_height(self.world.config.robot, 0.0)
        return max(0.0, desired_surface_z - baseline)

    def submit(self, request: TaskRequest):
        if self.state not in ("idle", "completed", "failed"):
            raise ValueError(f"Cannot submit task while {self.state}")
        
        if request.container_id not in self.world.container_ids:
            raise ValueError(f"Unknown container {request.container_id}")
            
        dest_loc = self._find_location(request.destination)
        if not dest_loc:
            raise ValueError(f"Unknown destination {request.destination}")
            
        # Check occupancy
        for cid, loc_id in getattr(self.world, 'container_locations', {}).items():
            if cid != request.container_id and loc_id == request.destination:
                raise ValueError(f"Destination {request.destination} is already occupied by {cid}")

        self.task = request
        self.error = None
        
        # Container current location is needed to know where to approach.
        # In a real system, the WMS knows where the container is.
        # We can find the container's physical position and find the closest location.
        c_state = next(c for c in self.world.get_container_states() if c.id == request.container_id)
        
        # Find closest station or slot
        config = self.world.config.warehouse
        all_locs = list(config.stations) + list(config.slots)
        src_loc = min(all_locs, key=lambda s: math.hypot(s.center[0] - c_state.position[0], s.center[1] - c_state.position[1]))
        
        ax, ay, ayaw = self._get_approach_pose(src_loc)
        
        self.state = "approach_source"
        dest = self.world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=0.5)
        self.world.plan_navigation(dest)
        self.world.start_navigation()
        
        # Keep track of source and destination heights
        self._src_z = src_loc.size[2] if hasattr(src_loc, 'size') else src_loc.center[2]
        self._dest_z = dest_loc.size[2] if hasattr(dest_loc, 'size') else dest_loc.center[2]

    def cancel(self):
        if self.state in ("idle", "completed", "failed"):
            return
        self.world.cancel_navigation("task cancelled")
        self.world.stop_mechanism("lift")
        self.world.stop_mechanism("forks")
        self.state = "idle"
        self.task = None

    def tick(self, dt: float):
        if self.state in ("idle", "completed", "failed"):
            return
            
        r = self.world.get_robot_state()
        
        if self.state == "approach_source":
            if self.world.navigator.status == "arrived":
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._src_z + 0.05)) # Slide above shelf, through container
                self.state = "raise_to_source"
            elif self.world.navigator.status in ("failed", "cancelled"):
                self.state = "failed"
                self.error = "Navigation to source failed"

        elif self.state == "raise_to_source":
            if r.lift.at_target:
                self.world.set_mechanism_target("forks", 0.6) # Extend forks
                self.state = "extend_to_source"

        elif self.state == "extend_to_source":
            if r.forks.at_target:
                self.world.attach_container(self.task.container_id)
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._src_z + 0.15)) # Lift container off shelf
                self.state = "lift_source"

        elif self.state == "lift_source":
            if r.lift.at_target:
                self.world.set_mechanism_target("forks", 0.0) # Retract forks
                self.state = "retract_source"

        elif self.state == "retract_source":
            if r.forks.at_target:
                self._back_away_timer = 2.0  # Drive back for 2 seconds at 0.5 m/s (1.0 meter)
                self.state = "back_away_source"

        elif self.state == "back_away_source":
            self.world.set_velocity_command(-0.5, 0.0)
            self._back_away_timer -= dt
            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0) # Lower to travel height
                self.state = "lower_source"

        elif self.state == "lower_source":
            if r.lift.at_target:
                dest_loc = self._find_location(self.task.destination)
                ax, ay, ayaw = self._get_approach_pose(dest_loc)
                dest = self.world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=0.5)
                self.world.plan_navigation(dest)
                self.world.start_navigation()
                self.state = "approach_dest"

        elif self.state == "approach_dest":
            if self.world.navigator.status == "arrived":
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z + 0.15)) # Above slot
                self.state = "raise_to_dest"
            elif self.world.navigator.status in ("failed", "cancelled"):
                self.state = "failed"
                self.error = "Navigation to dest failed"

        elif self.state == "raise_to_dest":
            if r.lift.at_target:
                self.world.set_mechanism_target("forks", 0.6) # Extend forks
                self.state = "extend_to_dest"

        elif self.state == "extend_to_dest":
            if r.forks.at_target:
                self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z + 0.05)) # Lower forks to 1cm above shelf
                self.state = "lower_dest"

        elif self.state == "lower_dest":
            if r.lift.at_target:
                self.world.detach_container(self.task.container_id)
                self.world.set_mechanism_target("forks", 0.0) # Retract forks
                self.state = "retract_dest"

        elif self.state == "retract_dest":
            if r.forks.at_target:
                self._back_away_timer = 2.0
                self.state = "back_away_dest"

        elif self.state == "back_away_dest":
            self.world.set_velocity_command(-0.5, 0.0)
            self._back_away_timer -= dt
            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0) # Lower lift to travel
                # Update logical occupancy
                if hasattr(self.world, 'container_locations') and self.task:
                    self.world.container_locations[self.task.container_id] = self.task.destination
                self.state = "completed"
                self.task = None

    def telemetry(self) -> TaskTelemetry:
        return TaskTelemetry(
            state=self.state,
            active_task=self.task,
            error=self.error
        )
