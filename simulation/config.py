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

from pydantic import BaseModel, ConfigDict, Field, model_validator

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


class ContainerConfig(_Model):
    id: str = Field(min_length=1)
    size: Vec3
    mass: float = Field(gt=0)
    station: str


class WarehouseConfig(_Model):
    name: str
    floor: FloorConfig
    walls: WallConfig
    rack_types: dict[str, RackType]
    racks: tuple[RackConfig, ...]
    zones: tuple[ZoneConfig, ...]
    stations: tuple[StationConfig, ...]
    container: ContainerConfig

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
        if self.container.station not in {s.id for s in self.stations}:
            raise ValueError(f"container references unknown station {self.container.station!r}")
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
