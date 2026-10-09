"""Differential-drive kinematics and command validation.

This module only converts *velocity requests* into *wheel velocity targets*.
It never integrates a pose: the robot's position and heading always come from
PyBullet, as a result of the wheel motors acting through contact friction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import MotionLimits, RobotConfig


class CommandError(ValueError):
    """Raised for malformed or out-of-bounds motion commands."""


@dataclass(frozen=True)
class WheelSpeeds:
    """Wheel angular velocity targets in rad/s (positive rolls the robot forward)."""

    left: float
    right: float


@dataclass(frozen=True)
class VelocityCommand:
    """A validated body-velocity request held for ``duration`` seconds of sim time."""

    linear: float  # m/s, +forward
    angular: float  # rad/s, +counter-clockwise (left turn)
    duration: float  # s


@dataclass(frozen=True)
class DiffDriveKinematics:
    wheel_radius: float
    track_width: float

    @classmethod
    def from_config(cls, robot: RobotConfig) -> DiffDriveKinematics:
        return cls(wheel_radius=robot.wheels.radius, track_width=robot.wheels.track_width)

    def wheel_speeds(self, linear: float, angular: float) -> WheelSpeeds:
        """Inverse kinematics: body twist (v, w) -> wheel angular velocities."""
        half_track = self.track_width / 2
        return WheelSpeeds(
            left=(linear - angular * half_track) / self.wheel_radius,
            right=(linear + angular * half_track) / self.wheel_radius,
        )

    def body_twist(self, left: float, right: float) -> tuple[float, float]:
        """Forward kinematics: wheel angular velocities -> body twist (v, w)."""
        v_left, v_right = left * self.wheel_radius, right * self.wheel_radius
        return (v_left + v_right) / 2, (v_right - v_left) / self.track_width


def validate_command(linear: float, angular: float, duration: float, limits: MotionLimits) -> VelocityCommand:
    """Reject (never silently clamp) non-finite or out-of-bounds requests."""
    for name, value in (("linear", linear), ("angular", angular), ("duration", duration)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CommandError(f"{name} must be a number, got {type(value).__name__}")
        if not math.isfinite(value):
            raise CommandError(f"{name} must be finite, got {value!r}")
    if abs(linear) > limits.max_linear_velocity:
        raise CommandError(f"linear velocity {linear} m/s exceeds limit ±{limits.max_linear_velocity} m/s")
    if abs(angular) > limits.max_angular_velocity:
        raise CommandError(f"angular velocity {angular} rad/s exceeds limit ±{limits.max_angular_velocity} rad/s")
    if not limits.min_command_duration <= duration <= limits.max_command_duration:
        raise CommandError(
            f"duration {duration} s outside [{limits.min_command_duration}, {limits.max_command_duration}] s"
        )
    return VelocityCommand(float(linear), float(angular), float(duration))


def approach(current: float, target: float, max_delta: float) -> float:
    """Move ``current`` towards ``target`` by at most ``max_delta`` (rate limiter)."""
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def ramp(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    """Rate-limit a velocity: ``accel`` while speeding up, ``decel`` while
    slowing down. Crossing zero brakes to zero first, then accelerates."""
    if current == 0.0 or (current > 0) == (target > 0) and abs(target) >= abs(current):
        return approach(current, target, accel * dt)
    stop_at = target if (current > 0) == (target > 0) else 0.0  # slow down (to zero if reversing)
    return approach(current, stop_at, decel * dt)
