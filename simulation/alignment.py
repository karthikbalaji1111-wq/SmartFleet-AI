"""Perception-guided target acquisition, coordinate transformation, and alignment.

Provides deterministic target identification from PyBullet simulated perception observations,
coordinate transformations between warehouse and robot body frames, bearing and insertion
orientation error calculations, and bounded closed-loop fine-alignment control.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal, Optional, Tuple, Union

from .config import AppConfig, RobotConfig, WarehouseConfig, get_config
from .robot_model import fork_surface_height, chassis_top_z, CARRIAGE_BASE_Z


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi] radians."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TargetObservation:
    """Validated perception observation of a requested container."""
    container_id: str
    category: str
    world_pos: Tuple[float, float, float]  # (x, y, z) in meters
    distance: float  # metric depth / direct distance
    bbox: Tuple[int, int, int, int]  # [x, y, w, h] in pixels
    timestamp: float
    age: float  # seconds since capture
    aabb_boundary_distance: Optional[float] = None  # cm


@dataclass(frozen=True)
class TargetRelativePose:
    """Target position and orientation errors expressed in the robot body frame."""
    container_id: str
    x_rel: float  # forward distance from axle midpoint (+ forward)
    y_rel: float  # lateral offset from centerline (+ left, - right)
    z_rel: float  # vertical offset from axle midpoint (+ up)
    distance_2d: float  # hypot(x_rel, y_rel)
    distance_3d: float  # hypot(x_rel, y_rel, z_rel)
    bearing_error: float  # rad, angle toward target in robot body frame (+ left)
    desired_heading: float  # rad, nominal warehouse yaw required for fork insertion
    orientation_error: float  # rad, heading difference from required insertion heading
    observation_age: float  # seconds


class TargetAcquisition:
    """Identifies and validates perception observations for a requested container."""

    def __init__(
        self,
        warehouse: Optional[Union[AppConfig, WarehouseConfig]] = None,
        max_observation_age: float = 1.5,
        min_distance: float = 0.1,
        max_distance: float = 9.5,
    ) -> None:
        cfg = warehouse or get_config()
        self.warehouse_config = cfg.warehouse if hasattr(cfg, "warehouse") else cfg
        self.max_observation_age = max_observation_age
        self.min_distance = min_distance
        self.max_distance = max_distance

        # Known valid containers
        self.valid_container_ids = {c.id for c in self.warehouse_config.containers}
        # Floor boundaries for validity check (+/- half size)
        self.floor_x_limit = self.warehouse_config.floor.size_x / 2.0
        self.floor_y_limit = self.warehouse_config.floor.size_y / 2.0
        self.max_height = self.warehouse_config.walls.height + 0.5

    def acquire_target(
        self,
        target_container_id: str,
        perception_result: Any,
        current_time: float,
    ) -> Tuple[Optional[TargetObservation], Optional[str]]:
        """Validate and extract the requested container from a perception result.

        Returns (TargetObservation, None) on success, or (None, rejection_reason).
        """
        # 1. Target ID must exist in warehouse metadata
        if target_container_id not in self.valid_container_ids:
            return None, f"Container '{target_container_id}' is not a valid warehouse container"

        # 2. Perception result availability
        if perception_result is None:
            return None, "No perception observation available"

        # Support both Pydantic model and dict
        timestamp = getattr(perception_result, "timestamp", None)
        if timestamp is None and isinstance(perception_result, dict):
            timestamp = perception_result.get("timestamp")

        if timestamp is None:
            return None, "Perception observation missing timestamp"

        # 3. Freshness check
        import time as _time
        ts = float(timestamp)
        if ts > 1e8:
            reference_time = current_time if current_time > 1e8 else _time.time()
        else:
            reference_time = current_time

        age = reference_time - ts
        if age < -0.05:  # small clock tolerance
            return None, f"Observation timestamp is in the future ({age:.3f}s)"
        if age > self.max_observation_age:
            return None, f"Observation is stale ({age:.2f}s > {self.max_observation_age:.2f}s limit)"

        # 4. Search detected objects
        objects = getattr(perception_result, "objects", None)
        if objects is None and isinstance(perception_result, dict):
            objects = perception_result.get("objects", [])

        if not objects:
            return None, f"No objects detected in camera field of view; target '{target_container_id}' not visible"

        target_obj = None
        detected_container_ids = []
        for obj in objects:
            instance_id = getattr(obj, "instance_id", None) or (obj.get("instance_id") if isinstance(obj, dict) else None)
            category = getattr(obj, "category", None) or (obj.get("category") if isinstance(obj, dict) else None)
            if category == "Container":
                detected_container_ids.append(instance_id)
            if instance_id == target_container_id:
                target_obj = obj
                break

        if target_obj is None:
            if detected_container_ids:
                return None, (
                    f"Target '{target_container_id}' not in camera view; "
                    f"visible containers: {', '.join(detected_container_ids)}"
                )
            return None, f"Target '{target_container_id}' is not in the camera field of view"

        # 5. Extract fields
        category = getattr(target_obj, "category", None) or (target_obj.get("category") if isinstance(target_obj, dict) else "")
        if category != "Container":
            return None, f"Target '{target_container_id}' identified with invalid category '{category}'"

        distance = getattr(target_obj, "distance", None) or (target_obj.get("distance") if isinstance(target_obj, dict) else None)
        if distance is None or not math.isfinite(distance):
            return None, f"Non-finite distance for target '{target_container_id}'"
        if distance < self.min_distance or distance > self.max_distance:
            return None, f"Target depth {distance:.2f}m is outside valid sensor range [{self.min_distance:.1f}, {self.max_distance:.1f}]m"

        pos = getattr(target_obj, "position", None) or (target_obj.get("position") if isinstance(target_obj, dict) else None)
        if not pos:
            return None, f"Missing 3D position for target '{target_container_id}'"

        if isinstance(pos, dict):
            px, py, pz = pos.get("x", float("nan")), pos.get("y", float("nan")), pos.get("z", float("nan"))
        else:
            px, py, pz = getattr(pos, "x", float("nan")), getattr(pos, "y", float("nan")), getattr(pos, "z", float("nan"))

        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            return None, f"Non-finite 3D coordinates ({px}, {py}, {pz}) for target '{target_container_id}'"

        # 6. Warehouse boundary sanity check
        if abs(px) > self.floor_x_limit or abs(py) > self.floor_y_limit or pz < 0.0 or pz > self.max_height:
            return None, (
                f"Target coordinates ({px:.2f}, {py:.2f}, {pz:.2f}) lie outside warehouse boundaries "
                f"[±{self.floor_x_limit:.1f}, ±{self.floor_y_limit:.1f}, {self.max_height:.1f}]"
            )

        bbox_raw = getattr(target_obj, "bbox", None) or (target_obj.get("bbox") if isinstance(target_obj, dict) else [0, 0, 0, 0])
        bbox = tuple(int(v) for v in bbox_raw)

        aabb_dist = getattr(target_obj, "aabb_boundary_distance", None)
        if aabb_dist is None and isinstance(target_obj, dict):
            aabb_dist = target_obj.get("aabb_boundary_distance")

        return TargetObservation(
            container_id=target_container_id,
            category=category,
            world_pos=(float(px), float(py), float(pz)),
            distance=float(distance),
            bbox=bbox,
            timestamp=float(timestamp),
            age=max(0.0, age),
            aabb_boundary_distance=float(aabb_dist) if aabb_dist is not None else None,
        ), None


def world_to_robot_body(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    world_x: float,
    world_y: float,
    world_z: float = 0.0,
    robot_z: float = 0.0,
) -> Tuple[float, float, float]:
    """Transform world 3D coordinate into the robot body frame (+X forward, +Y left, +Z up)."""
    dx = world_x - robot_x
    dy = world_y - robot_y
    dz = world_z - robot_z
    cy = math.cos(robot_yaw)
    sy = math.sin(robot_yaw)
    x_rel = dx * cy + dy * sy
    y_rel = -dx * sy + dy * cy
    return x_rel, y_rel, dz


def derive_pickup_orientation(
    location_id: str,
    warehouse_config: Optional[WarehouseConfig] = None,
) -> float:
    """Determine the required robot yaw heading for feasible fork insertion into location_id.

    Derived from actual rack/station geometry and accessible aisle facing:
    - Stations (inbound/outbound stand at x=10.4): robot faces east (yaw = 0.0).
    - Racks along X:
      - Racks at y > aisle (e.g. Rack A at y=5.5): robot faces north (yaw = +pi/2).
      - Racks at y < aisle (e.g. Rack B at y=2.0 from aisle 3.75): robot faces south (yaw = -pi/2).
    """
    wh = warehouse_config or get_config().warehouse

    # 1. Check stations
    station = next((s for s in wh.stations if s.id == location_id), None)
    if station:
        if station.center[0] > 8.0:
            return 0.0  # Station stands face west; robot must face east
        return 0.0

    # 2. Check slots
    slot = next((s for s in wh.slots if s.id == location_id), None)
    if slot:
        rack = next((r for r in wh.racks if r.id == slot.rack_id), None)
        if rack:
            cx, cy = rack.center[0], rack.center[1]
            aisles = [3.75, 0.25, -3.25]
            best_aisle_y = min(aisles, key=lambda ay: abs(ay - cy))
            if cy > best_aisle_y:
                return math.pi / 2.0  # Face north (+Y)
            else:
                return -math.pi / 2.0  # Face south (-Y)

    # Fallback heuristic if location_id is not directly resolved
    return 0.0


def calculate_target_relative_pose(
    robot_x: float,
    robot_y: float,
    robot_z: float,
    robot_yaw: float,
    observation: TargetObservation,
    desired_heading: float,
) -> TargetRelativePose:
    """Calculate relative coordinates, bearing error, and pickup orientation error."""
    wx, wy, wz = observation.world_pos
    x_rel, y_rel, z_rel = world_to_robot_body(robot_x, robot_y, robot_yaw, wx, wy, wz, robot_z)

    dist_2d = math.hypot(x_rel, y_rel)
    dist_3d = math.hypot(x_rel, y_rel, z_rel)
    bearing_error = math.atan2(y_rel, x_rel)
    orientation_error = normalize_angle(desired_heading - robot_yaw)

    return TargetRelativePose(
        container_id=observation.container_id,
        x_rel=x_rel,
        y_rel=y_rel,
        z_rel=z_rel,
        distance_2d=dist_2d,
        distance_3d=dist_3d,
        bearing_error=bearing_error,
        desired_heading=desired_heading,
        orientation_error=orientation_error,
        observation_age=observation.age,
    )


def calculate_staging_pose(
    location_id: str,
    warehouse_config: Optional[WarehouseConfig] = None,
    standoff_distance: float = 1.0,
) -> Tuple[float, float, float]:
    """Calculate a safe staging pose (x, y, yaw) in the aisle with standoff from target location.

    Leaves sufficient room for camera observation, fine alignment, and fork insertion
    while keeping the robot in the navigable aisle footprint.
    """
    wh = warehouse_config or get_config().warehouse

    # 1. Station location
    station = next((s for s in wh.stations if s.id == location_id), None)
    if station:
        cx, cy = station.center[0], station.center[1]
        # Inbound/outbound stations stand at x=10.4. Safe approach from west:
        return cx - standoff_distance, cy, 0.0

    # 2. Slot location
    slot = next((s for s in wh.slots if s.id == location_id), None)
    if slot:
        sx, sy = slot.center[0], slot.center[1]
        aisles = [3.75, 0.25, -3.25]
        best_aisle_y = min(aisles, key=lambda ay: abs(ay - sy))

        if sy > best_aisle_y:
            # Rack is north of aisle; robot stays in aisle facing north
            return sx, sy - standoff_distance, math.pi / 2.0
        else:
            # Rack is south of aisle; robot stays in aisle facing south
            return sx, sy + standoff_distance, -math.pi / 2.0

    # Fallback to center of floor facing east
    return 0.0, 0.0, 0.0


def derive_fork_insertion_target(
    robot_config: RobotConfig,
    container_height: float,
    shelf_surface_z: float,
    insertion_depth: float = 0.60,
) -> Tuple[float, float]:
    """Derive lift target (m) and fork extension target (m) for safe container pickup.

    Args:
        robot_config: Canonical robot geometry
        container_height: Height of target container in meters
        shelf_surface_z: World Z coordinate of the shelf/stand support surface
        insertion_depth: Desired forward fork extension in meters (default 0.60m)

    Returns:
        (lift_target, fork_target)
    """
    baseline = fork_surface_height(robot_config, 0.0)
    desired_fork_surface_z = shelf_surface_z + 0.05
    lift_target = max(robot_config.lift.lower, min(robot_config.lift.upper, desired_fork_surface_z - baseline))
    fork_target = max(robot_config.forks.lower, min(robot_config.forks.upper, insertion_depth))

    return lift_target, fork_target


@dataclass
class AlignmentTolerances:
    """Tolerances for declaring fine pickup alignment satisfied."""
    lateral_tol: float = 0.035  # m (+/- 3.5 cm)
    heading_tol: float = 0.040  # rad (+/- 2.3 deg)
    bearing_tol: float = 0.050  # rad (+/- 2.9 deg)
    distance_min: float = 0.65  # m (minimum standoff from axle)
    distance_max: float = 0.85  # m (maximum standoff from axle)
    timeout_s: float = 12.0     # seconds
    max_attempts: Optional[int] = None  # optional discrete control step limit
    max_lost_observations: int = 50     # max consecutive missing observations


@dataclass
class AlignmentState:
    """Live state of the fine alignment controller."""
    active: bool = False
    aligned: bool = False
    phase: Literal["idle", "heading_align", "approach", "fine_tune", "aligned", "failed"] = "idle"
    attempts: int = 0
    lost_frames: int = 0
    elapsed: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    forward_distance: float = 0.0
    message: str = "Idle"
    failure_reason: Optional[str] = None


class AlignmentController:
    """Closed-loop alignment controller for fine perception-guided pickup alignment."""

    def __init__(self, tolerances: Optional[AlignmentTolerances] = None) -> None:
        self.tol = tolerances or AlignmentTolerances()
        self.state = AlignmentState()

    def reset(self) -> None:
        self.state = AlignmentState()

    def update(
        self,
        rel_pose: Optional[TargetRelativePose],
        dt: float,
    ) -> Tuple[float, float, bool, str]:
        """Compute bounded velocity commands (v, w) based on target relative pose.

        Returns:
            (linear_velocity, angular_velocity, is_aligned, status_message)
        """
        if not self.state.active:
            self.state.active = True
            self.state.phase = "heading_align"
            self.state.attempts = 0
            self.state.lost_frames = 0
            self.state.elapsed = 0.0

        self.state.elapsed += dt

        # Timeout check
        if self.state.elapsed > self.tol.timeout_s:
            self.state.phase = "failed"
            self.state.failure_reason = f"Alignment timed out after {self.state.elapsed:.1f}s"
            self.state.message = self.state.failure_reason
            return 0.0, 0.0, False, self.state.message

        if rel_pose is None:
            self.state.lost_frames += 1
            if self.state.lost_frames > self.tol.max_lost_observations:
                self.state.phase = "failed"
                self.state.failure_reason = "Target observation lost during alignment"
                self.state.message = self.state.failure_reason
                return 0.0, 0.0, False, self.state.message
            return 0.0, 0.0, False, "Waiting for fresh observation..."

        self.state.lost_frames = 0
        self.state.lateral_error = rel_pose.y_rel
        self.state.heading_error = rel_pose.orientation_error
        self.state.forward_distance = rel_pose.x_rel

        lat_ok = abs(rel_pose.y_rel) <= self.tol.lateral_tol
        heading_ok = abs(rel_pose.orientation_error) <= self.tol.heading_tol
        dist_ok = self.tol.distance_min <= rel_pose.x_rel <= self.tol.distance_max

        # Check full convergence
        if lat_ok and heading_ok and dist_ok:
            self.state.aligned = True
            self.state.phase = "aligned"
            self.state.message = (
                f"Aligned! Lat error: {rel_pose.y_rel * 100:.1f}cm, "
                f"Heading error: {math.degrees(rel_pose.orientation_error):.1f}°, "
                f"Dist: {rel_pose.x_rel:.2f}m"
            )
            return 0.0, 0.0, True, self.state.message

        self.state.attempts += 1
        if self.tol.max_attempts is not None and self.state.attempts > self.tol.max_attempts:
            self.state.phase = "failed"
            self.state.failure_reason = (
                f"Alignment exceeded max attempts ({self.state.attempts}); "
                f"final lat: {rel_pose.y_rel * 100:.1f}cm, heading: {math.degrees(rel_pose.orientation_error):.1f}°"
            )
            self.state.message = self.state.failure_reason
            return 0.0, 0.0, False, self.state.message

        # Compute track heading error: incorporates crab angle to eliminate lateral offset during approach
        lookahead = max(0.40, rel_pose.x_rel)
        crab_angle = math.atan2(rel_pose.y_rel, lookahead)
        crab_angle = max(-0.25, min(0.25, crab_angle))
        track_heading_error = normalize_angle(rel_pose.orientation_error + crab_angle)

        # 1. Coarse heading alignment phase (turn in place if off-track by > 0.12 rad / ~6.9 deg)
        if abs(track_heading_error) > 0.12:
            self.state.phase = "heading_align"
            w_raw = 1.4 * track_heading_error
            w = max(-0.30, min(0.30, w_raw))
            if abs(w) < 0.08:
                w = math.copysign(0.08, w)
            msg = f"Aligning heading ({math.degrees(track_heading_error):.1f}°)"
            return 0.0, w, False, msg

        # 2. Distance approach phase: drive forward while tracking approach line
        if rel_pose.x_rel > self.tol.distance_max:
            self.state.phase = "approach"
            v_raw = 0.35 * (rel_pose.x_rel - (self.tol.distance_min + self.tol.distance_max) / 2.0)
            v = max(0.06, min(0.18, v_raw))
            w = max(-0.25, min(0.25, 1.4 * track_heading_error))
            msg = f"Approaching target (dist {rel_pose.x_rel:.2f}m, lat {rel_pose.y_rel*100:.1f}cm)"
            return v, w, False, msg

        # 3. Forward margin phase: if lateral error exceeds tolerance and distance margin allows
        if abs(rel_pose.y_rel) > self.tol.lateral_tol and rel_pose.x_rel > (self.tol.distance_min + 0.04):
            self.state.phase = "approach"
            v = 0.06
            w = max(-0.25, min(0.25, 1.4 * track_heading_error))
            msg = f"Creeping forward to reduce lateral error (lat {rel_pose.y_rel*100:.1f}cm)"
            return v, w, False, msg

        # 4. Fine-tuning phase (in distance range; finalize heading alignment)
        self.state.phase = "fine_tune"
        w_raw = 1.4 * rel_pose.orientation_error
        w = max(-0.25, min(0.25, w_raw))
        if abs(rel_pose.orientation_error) > self.tol.heading_tol and abs(w) < 0.08:
            w = math.copysign(0.08, w)
        v = 0.0
        msg = f"Fine-tuning alignment (lat: {rel_pose.y_rel*100:.1f}cm, yaw: {math.degrees(rel_pose.orientation_error):.1f}°)"
        return v, w, False, msg
