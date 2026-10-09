"""Canonical configuration models and loader.

``config/warehouse.json`` is the single source of truth for the warehouse
layout, the robot's dimensions and limits, and the simulation parameters. The
physics world, the API and (through the API) the 3D frontend all read it, so
the rendered shelves always match the collision geometry.
"""

from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator, computed_field

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "warehouse.json"

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]


class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


# --------------------------------------------------------------------------- #
# Warehouse
# --------------------------------------------------------------------------- #
class FloorConfig(_Model):
    size_x: float = Field(gt=0)
    size_y: float = Field(gt=0)
    grid_spacing: float = Field(gt=0)


class WallConfig(_Model):
    height: float = Field(gt=0)
    thickness: float = Field(gt=0)


class RackType(_Model):
    length: float = Field(gt=0)
    depth: float = Field(gt=0)
    height: float = Field(gt=0)
    bays: int = Field(ge=1)
    post_size: float = Field(gt=0)
    shelf_thickness: float = Field(gt=0)
    levels: tuple[float, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def _check_levels(self) -> RackType:
        if any(b <= a for a, b in zip(self.levels, self.levels[1:])):
            raise ValueError("rack levels must be strictly increasing")
        if self.levels[0] < self.shelf_thickness or self.levels[-1] > self.height:
            raise ValueError("rack levels must lie between the shelf thickness and the rack height")
        if 2 * self.post_size >= min(self.length, self.depth):
            raise ValueError("rack posts are too large for the rack footprint")
        return self


class RackConfig(_Model):
    id: str = Field(min_length=1)
    type: str
    center: Vec2
    axis: Literal["x", "y"] = "x"


class ZoneConfig(_Model):
    id: str = Field(min_length=1)
    kind: Literal["loading", "delivery", "home"]
    label: str
    center: Vec2
    size: Vec2


class StationConfig(_Model):
    id: str = Field(min_length=1)
    zone: str
    center: Vec2
    size: Vec3



class StorageSlot(_Model):
    id: str
    rack_id: str
    bay: int
    level: int
    center: Vec3

class ContainerConfig(_Model):
    id: str = Field(min_length=1)
    size: Vec3
    mass: float = Field(gt=0)
    location: str


class WarehouseConfig(_Model):
    name: str
    floor: FloorConfig
    walls: WallConfig
    rack_types: dict[str, RackType]
    racks: tuple[RackConfig, ...]
    zones: tuple[ZoneConfig, ...]
    stations: tuple[StationConfig, ...]
    containers: tuple[ContainerConfig, ...]

    @model_validator(mode="after")
    def _check_references(self) -> WarehouseConfig:
        ids = [r.id for r in self.racks] + [z.id for z in self.zones] + [s.id for s in self.stations]
        dupes = sorted({i for i in ids if ids.count(i) > 1})
        if dupes:
            raise ValueError(f"duplicate warehouse element ids: {dupes}")
        for rack in self.racks:
            if rack.type not in self.rack_types:
                raise ValueError(f"rack {rack.id!r} references unknown rack type {rack.type!r}")
        zone_ids = {z.id for z in self.zones}
        for station in self.stations:
            if station.zone not in zone_ids:
                raise ValueError(f"station {station.id!r} references unknown zone {station.zone!r}")
        return self

    @computed_field
    @property
    def slots(self) -> list[StorageSlot]:
        slots = []
        for rack in self.racks:
            rt = self.rack_types[rack.type]
            # Bay width
            bay_width = (rt.length - rt.post_size) / rt.bays
            start_offset = -rt.length / 2 + rt.post_size / 2 + bay_width / 2
            
            # Get max container width
            max_cont_width = max([c.size[0] for c in self.containers]) if self.containers else 0.4
            clearance = 0.05
            slot_needed = max_cont_width + clearance
            for bay in range(rt.bays):
                slots_per_bay = max(1, int(bay_width / slot_needed))
                sub_slot_width = bay_width / slots_per_bay
                sub_start_offset = -bay_width / 2 + sub_slot_width / 2
                
                for lvl_idx, z in enumerate(rt.levels):
                    for s_idx in range(slots_per_bay):
                        slot_id = f"{rack.id}-B{bay+1}-L{lvl_idx+1}-S{s_idx+1}"
                        
                        # Local x offset for the bay
                        bay_center_x = start_offset + bay * bay_width
                        local_x = bay_center_x + sub_start_offset + s_idx * sub_slot_width
                        
                        if rack.axis == "x":
                            cx = rack.center[0] + local_x
                            cy = rack.center[1]
                        else:
                            cx = rack.center[0]
                            cy = rack.center[1] + local_x
                            
                        slots.append(StorageSlot(
                            id=slot_id,
                            rack_id=rack.id,
                            bay=bay+1,
                            level=lvl_idx+1,
                            center=(cx, cy, z)
                        ))
        return slots

    @model_validator(mode="after")
    def _validate_locations(self) -> WarehouseConfig:
        valid_locs = {s.id for s in self.stations} | {s.id for s in self.slots}
        for c in self.containers:
            if c.location not in valid_locs:
                raise ValueError(f"container {c.id!r} references unknown location {c.location!r}")
        return self

        return self

    def rack_type(self, rack: RackConfig) -> RackType:
        return self.rack_types[rack.type]

    def station(self, station_id: str) -> StationConfig:
        return next(s for s in self.stations if s.id == station_id)

    def zone(self, zone_id: str) -> ZoneConfig:
        return next(z for z in self.zones if z.id == zone_id)


# --------------------------------------------------------------------------- #
# Robot
# --------------------------------------------------------------------------- #
class Pose2D(_Model):
    x: float
    y: float
    yaw: float = Field(ge=-math.pi - 1e-9, le=math.pi + 1e-9)


class ChassisConfig(_Model):
    length: float = Field(gt=0)
    width: float = Field(gt=0)
    height: float = Field(gt=0)
    ground_clearance: float = Field(gt=0)
    mass: float = Field(gt=0)


class WheelConfig(_Model):
    radius: float = Field(gt=0)
    width: float = Field(gt=0)
    track_width: float = Field(gt=0)
    mass: float = Field(gt=0)
    max_torque: float = Field(gt=0)
    max_velocity: float = Field(gt=0)
    lateral_friction: float = Field(gt=0)


class CasterConfig(_Model):
    radius: float = Field(gt=0)
    offset_x: float = Field(gt=0)
    mass: float = Field(gt=0)


class MastConfig(_Model):
    height: float = Field(gt=0)
    offset_x: float
    upright_size: float = Field(gt=0)
    upright_spacing: float = Field(gt=0)
    mass: float = Field(gt=0)


class ActuatorConfig(_Model):
    """Operating parameters shared by the prismatic lift and fork mechanisms."""

    lower: float  # joint travel limits, m (also written into the URDF)
    upper: float
    default_position: float  # home position after start-up / reset
    position_tolerance: float = Field(gt=0)  # |target - position| that counts as "at target"
    max_jog_step: float = Field(gt=0)  # largest accepted incremental move
    max_velocity: float = Field(gt=0)  # m/s, enforced by the PyBullet motor
    max_force: float = Field(gt=0)  # N, motor force limit

    @model_validator(mode="after")
    def _check_range(self) -> ActuatorConfig:
        if self.upper <= self.lower:
            raise ValueError("actuator upper limit must exceed lower limit")
        if not self.lower <= self.default_position <= self.upper:
            raise ValueError("actuator default_position must lie within [lower, upper]")
        if self.max_jog_step > self.upper - self.lower:
            raise ValueError("actuator max_jog_step must not exceed the travel range")
        return self


class LiftConfig(ActuatorConfig):
    carriage_mass: float = Field(gt=0)


class ForkConfig(ActuatorConfig):
    length: float = Field(gt=0)
    tine_width: float = Field(gt=0)
    tine_thickness: float = Field(gt=0)
    tine_spacing: float = Field(gt=0)
    mass: float = Field(gt=0)


class MotionLimits(_Model):
    max_linear_velocity: float = Field(gt=0)
    max_angular_velocity: float = Field(gt=0)
    max_linear_acceleration: float = Field(gt=0)
    max_angular_acceleration: float = Field(gt=0)
    max_linear_deceleration: float = Field(gt=0)  # braking (incl. Stop), m/s^2
    max_angular_deceleration: float = Field(gt=0)
    max_handling_tilt: float = Field(gt=0, lt=0.5)  # rad; lift/forks stop if the chassis tilts more while moving
    min_command_duration: float = Field(gt=0)
    max_command_duration: float = Field(gt=0)
    default_command_duration: float = Field(gt=0)

    @model_validator(mode="after")
    def _check_durations(self) -> MotionLimits:
        if not self.min_command_duration <= self.default_command_duration <= self.max_command_duration:
            raise ValueError("default command duration must lie within [min, max]")
        return self


class RobotConfig(_Model):
    id: str
    name: str
    start_pose: Pose2D
    chassis: ChassisConfig
    wheels: WheelConfig
    casters: CasterConfig
    mast: MastConfig
    lift: LiftConfig
    forks: ForkConfig
    limits: MotionLimits

    @model_validator(mode="after")
    def _check_geometry(self) -> RobotConfig:
        if self.chassis.ground_clearance >= self.wheels.radius:
            raise ValueError("chassis ground clearance must be smaller than the wheel radius")
        if self.wheels.track_width - self.wheels.width < self.chassis.width:
            raise ValueError("drive wheels must sit outside the chassis (track_width - wheel width >= chassis width)")
        if self.casters.offset_x + self.casters.radius > self.chassis.length / 2:
            raise ValueError("casters must fit under the chassis")
        if self.casters.radius > self.wheels.radius:
            raise ValueError("caster radius must not exceed the wheel radius")
        # Max wheel speed needed for the motion limits must be achievable.
        needed = (
            self.limits.max_linear_velocity + self.limits.max_angular_velocity * self.wheels.track_width / 2
        ) / self.wheels.radius
        if needed > self.wheels.max_velocity:
            raise ValueError(
                f"motion limits require {needed:.2f} rad/s wheel speed, above wheel max_velocity "
                f"{self.wheels.max_velocity:.2f}"
            )
        return self


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #
class SimulationConfig(_Model):
    physics_hz: int = Field(ge=60, le=2000)
    gravity: float = Field(gt=0)
    solver_iterations: int = Field(ge=1)
    settle_steps: int = Field(ge=0)
    floor_friction: float = Field(gt=0)
    loop_hz: int = Field(ge=1)
    max_steps_per_tick: int = Field(ge=1)
    telemetry_hz: int = Field(ge=1, le=120)

    @property
    def timestep(self) -> float:
        return 1.0 / self.physics_hz


# --------------------------------------------------------------------------- #
# Navigation
# --------------------------------------------------------------------------- #
class GridConfig(_Model):
    resolution: float = Field(gt=0.01, le=0.5)  # m per occupancy cell
    clearance_margin: float = Field(ge=0)  # extra clearance beyond the robot's footprint radius, m
    preferred_clearance: float = Field(ge=0)  # beyond the hard clearance, cells closer than this cost more, m
    proximity_weight: float = Field(ge=0)  # extra cost factor at the hard clearance limit (0 = plain A*)
    allow_diagonal: bool = True
    start_snap_radius: float = Field(ge=0)  # robot may start this far inside the inflated zone, m
    goal_snap_radius: float = Field(ge=0)  # floor-picked goals may be moved this far to free space, m

    @model_validator(mode="after")
    def _check_margin(self) -> GridConfig:
        # A point anywhere in a free cell is at most half a cell diagonal closer to an
        # obstacle than the cell centre; the margin must absorb that discretisation.
        if self.clearance_margin < self.resolution * math.sqrt(2) / 2:
            raise ValueError("clearance_margin must be at least half a grid cell diagonal")
        return self


class FollowerConfig(_Model):
    control_hz: int = Field(ge=5, le=240)  # path-follower update rate
    cruise_speed: float = Field(gt=0)  # m/s, must not exceed robot.limits.max_linear_velocity
    max_turn_rate: float = Field(gt=0)  # rad/s, must not exceed robot.limits.max_angular_velocity
    heading_gain: float = Field(gt=0)  # angular rate per rad of heading error, 1/s
    rotate_in_place_threshold: float = Field(gt=0, lt=math.pi / 2)  # rad; larger errors turn on the spot
    waypoint_tolerance: float = Field(gt=0)  # m; intermediate waypoint counts as reached
    goal_tolerance: float = Field(gt=0)  # m; final position tolerance
    heading_tolerance: float = Field(gt=0)  # rad; final heading tolerance (destinations with yaw)
    braking_fraction: float = Field(gt=0, le=1)  # share of max_linear_deceleration used for planned slowdowns
    max_cross_track_error: float = Field(gt=0)  # m; farther from the route aborts navigation
    progress_timeout: float = Field(gt=0)  # s without progress before navigation fails
    min_progress: float = Field(gt=0)  # m (or rad while turning) that counts as progress
    command_duration: float = Field(gt=0)  # s; dead-man duration of each autonomous drive command


class TravelConfig(_Model):
    """Conservative travel interlock: autonomous driving only in this configuration."""

    max_lift: float = Field(ge=0)  # m, lift joint position
    max_fork_extension: float = Field(ge=0)  # m, fork joint position


class DestinationConfig(_Model):
    id: str = Field(min_length=1)
    label: str
    kind: Literal["home", "aisle", "corridor", "staging"]
    x: float
    y: float
    yaw: float | None = None  # optional final heading, rad


class NavigationConfig(_Model):
    grid: GridConfig
    controller: FollowerConfig
    travel: TravelConfig
    destinations: tuple[DestinationConfig, ...]

    @model_validator(mode="after")
    def _check(self) -> NavigationConfig:
        ids = [d.id for d in self.destinations]
        if len(ids) != len(set(ids)):
            raise ValueError("navigation destination ids must be unique")
        c = self.controller
        if c.goal_tolerance > c.waypoint_tolerance:
            raise ValueError("goal_tolerance must not exceed waypoint_tolerance")
        if c.command_duration < 2.0 / c.control_hz:
            raise ValueError("command_duration must cover at least two controller periods")
        return self

    def destination(self, destination_id: str) -> DestinationConfig | None:
        return next((d for d in self.destinations if d.id == destination_id), None)


class FrameConfig(_Model):
    up_axis: Literal["z"]
    description: str


class AppConfig(_Model):
    schema_version: Literal[1]
    units: dict[str, str]
    frame: FrameConfig
    warehouse: WarehouseConfig
    robot: RobotConfig
    simulation: SimulationConfig
    navigation: NavigationConfig

    @model_validator(mode="after")
    def _check_navigation_against_robot(self) -> AppConfig:
        c, lim = self.navigation.controller, self.robot.limits
        if c.cruise_speed > lim.max_linear_velocity:
            raise ValueError("navigation cruise_speed exceeds robot.limits.max_linear_velocity")
        if c.max_turn_rate > lim.max_angular_velocity:
            raise ValueError("navigation max_turn_rate exceeds robot.limits.max_angular_velocity")
        if not lim.min_command_duration <= c.command_duration <= lim.max_command_duration:
            raise ValueError("navigation command_duration outside robot.limits command duration range")
        t = self.navigation.travel
        if not self.robot.lift.lower <= t.max_lift <= self.robot.lift.upper:
            raise ValueError("navigation travel.max_lift outside the lift travel range")
        if not self.robot.forks.lower <= t.max_fork_extension <= self.robot.forks.upper:
            raise ValueError("navigation travel.max_fork_extension outside the fork travel range")
        if self.robot.lift.default_position > t.max_lift or self.robot.forks.default_position > t.max_fork_extension:
            raise ValueError("lift/fork default positions must be a safe travel configuration")
        return self


def load_config(path: str | Path | None = None) -> AppConfig:
    """Load and validate a configuration file (defaults to the canonical one)."""
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    with config_path.open("r", encoding="utf-8") as fh:
        raw = json.load(fh)
    return AppConfig.model_validate(raw)


@lru_cache(maxsize=1)
def get_config() -> AppConfig:
    """Return the canonical configuration (loaded once per process)."""
    return load_config()
