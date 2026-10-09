"""Telemetry models. Every value here is read from PyBullet at query time."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict

from .navigation.models import NavigationTelemetry

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (x, y, z, w), PyBullet order


class _Model(BaseModel):
    model_config = ConfigDict(frozen=True)


class PoseTelemetry(_Model):
    position: Vec3  # world frame, metres (body-frame origin = axle midpoint)
    orientation: Quat
    heading: float  # yaw, rad, CCW from +X, wrapped to (-pi, pi]
    heading_deg: float
    roll: float
    pitch: float


class VelocityTelemetry(_Model):
    forward: float  # m/s along the robot's heading (measured)
    lateral: float  # m/s to the robot's left (measured; ~0 for a non-slipping diff drive)
    yaw_rate: float  # rad/s about +Z (measured)
    linear_world: Vec3
    angular_world: Vec3


class JointTelemetry(_Model):
    position: float
    velocity: float
    applied_effort: float


class WheelsTelemetry(_Model):
    left: JointTelemetry
    right: JointTelemetry
    target_left: float  # rad/s motor targets sent to PyBullet
    target_right: float
    brake_engaged: bool  # wheels in position-hold because the robot is at rest


class MechanismTelemetry(_Model):
    """Lift or fork prismatic joint, as measured by PyBullet."""

    position: float  # measured joint position, m
    velocity: float  # measured joint velocity, m/s
    target: float  # position target held by the PyBullet motor, m
    error: float  # target - position, m
    at_target: bool  # |error| within the configured tolerance
    state: Literal["holding", "moving", "blocked"]
    fault: Literal["stalled", "overload", "tilt"] | None  # why it is blocked (cleared by the next command)
    applied_force: float  # motor force reported by PyBullet, N
    lower: float
    upper: float
    default: float
    max_velocity: float


class CommandTelemetry(_Model):
    active: bool
    target_linear: float  # requested, m/s
    target_angular: float  # requested, rad/s
    linear: float  # after acceleration limiting, m/s
    angular: float
    remaining: float  # s of sim time until the dead-man timeout zeroes the request


class RobotState(_Model):
    id: str
    pose: PoseTelemetry
    velocity: VelocityTelemetry
    wheels: WheelsTelemetry
    lift: MechanismTelemetry
    forks: MechanismTelemetry
    fork_surface_height: float  # top of the fork tines above the floor, from the fork link pose, m
    command: CommandTelemetry


class BodyState(_Model):
    id: str
    position: Vec3
    orientation: Quat


class WorldState(_Model):
    sim_time: float
    step: int
    robot: RobotState
    container: BodyState
    navigation: NavigationTelemetry
