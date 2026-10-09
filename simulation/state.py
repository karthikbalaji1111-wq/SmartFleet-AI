"""Telemetry models. Every value here is read from PyBullet at query time."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict

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


class ActuatorTelemetry(_Model):
    position: float  # measured joint position, m
    velocity: float
    lower: float
    upper: float
    target: float
    mode: Literal["hold"]  # Milestone 1: actuator held at target; motion control deferred


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
    lift: ActuatorTelemetry
    forks: ActuatorTelemetry
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
