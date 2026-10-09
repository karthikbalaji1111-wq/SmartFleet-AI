"""Differential-drive kinematics, command validation and limits."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from backend.app.schemas import VelocityCommandRequest
from simulation.diff_drive import CommandError, DiffDriveKinematics, approach, ramp, validate_command


@pytest.fixture
def kin(config) -> DiffDriveKinematics:
    return DiffDriveKinematics.from_config(config.robot)


def test_straight_line_gives_equal_wheel_speeds(kin, config):
    ws = kin.wheel_speeds(0.5, 0.0)
    assert ws.left == pytest.approx(ws.right) == pytest.approx(0.5 / config.robot.wheels.radius)


def test_ccw_rotation_spins_right_wheel_forward(kin, config):
    w = 1.0
    ws = kin.wheel_speeds(0.0, w)
    expected = w * config.robot.wheels.track_width / 2 / config.robot.wheels.radius
    assert ws.right == pytest.approx(expected)
    assert ws.left == pytest.approx(-expected)


@pytest.mark.parametrize(("v", "w"), [(0.3, 0.0), (-0.7, 0.4), (0.0, -1.2), (1.0, 1.5)])
def test_inverse_and_forward_kinematics_round_trip(kin, v, w):
    ws = kin.wheel_speeds(v, w)
    assert kin.body_twist(ws.left, ws.right) == pytest.approx((v, w))


def test_wheel_speeds_at_limits_are_within_motor_rating(kin, config):
    lim = config.robot.limits
    for v in (-lim.max_linear_velocity, lim.max_linear_velocity):
        for w in (-lim.max_angular_velocity, lim.max_angular_velocity):
            ws = kin.wheel_speeds(v, w)
            assert max(abs(ws.left), abs(ws.right)) <= config.robot.wheels.max_velocity


def test_validate_command_accepts_in_bounds(config):
    lim = config.robot.limits
    cmd = validate_command(lim.max_linear_velocity, -lim.max_angular_velocity, lim.max_command_duration, lim)
    assert (cmd.linear, cmd.angular, cmd.duration) == (
        lim.max_linear_velocity,
        -lim.max_angular_velocity,
        lim.max_command_duration,
    )


@pytest.mark.parametrize(
    ("linear", "angular", "duration"),
    [
        (1.01, 0.0, 0.5),
        (-1.01, 0.0, 0.5),
        (0.0, 1.51, 0.5),
        (0.0, -9.0, 0.5),
        (math.nan, 0.0, 0.5),
        (0.0, math.inf, 0.5),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 2.5),
        (True, 0.0, 0.5),
        ("0.5", 0.0, 0.5),
    ],
)
def test_validate_command_rejects_bad_values(config, linear, angular, duration):
    with pytest.raises(CommandError):
        validate_command(linear, angular, duration, config.robot.limits)


def test_rate_limiter():
    assert approach(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert approach(0.95, 1.0, 0.1) == pytest.approx(1.0)
    assert approach(0.0, -1.0, 0.25) == pytest.approx(-0.25)
    assert approach(0.5, 0.5, 0.1) == 0.5


def test_ramp_uses_acceleration_and_deceleration_limits():
    dt = 0.1
    assert ramp(0.0, 1.0, accel=1.0, decel=2.0, dt=dt) == pytest.approx(0.1)  # speeding up
    assert ramp(0.5, 0.0, accel=1.0, decel=2.0, dt=dt) == pytest.approx(0.3)  # braking
    assert ramp(-0.5, 0.0, accel=1.0, decel=2.0, dt=dt) == pytest.approx(-0.3)
    assert ramp(0.5, 0.2, accel=1.0, decel=2.0, dt=dt) == pytest.approx(0.3)  # slowing, same sign
    assert ramp(0.1, -1.0, accel=1.0, decel=2.0, dt=dt) == 0.0  # reversing brakes to zero first
    assert ramp(0.0, -1.0, accel=1.0, decel=2.0, dt=dt) == pytest.approx(-0.1)


# --------------------------------------------------------------------------- #
# API request schema (bounds come from the canonical config)
# --------------------------------------------------------------------------- #
def test_api_schema_accepts_valid_and_defaults_duration(config):
    req = VelocityCommandRequest.model_validate({"linear": 0.5, "angular": -1})
    assert req.angular == -1.0
    assert req.duration == config.robot.limits.default_command_duration


@pytest.mark.parametrize(
    "payload",
    [
        {"linear": 1.5, "angular": 0.0},
        {"linear": 0.0, "angular": -2.0},
        {"linear": "0.5", "angular": 0.0},
        {"linear": True, "angular": 0.0},
        {"linear": 0.5},
        {"linear": 0.5, "angular": 0.0, "turbo": True},
        {"linear": math.nan, "angular": 0.0},
        {"linear": 0.5, "angular": 0.0, "duration": 30.0},
        {"linear": None, "angular": 0.0},
    ],
)
def test_api_schema_rejects_malformed(payload):
    with pytest.raises(ValidationError):
        VelocityCommandRequest.model_validate(payload)


# --------------------------------------------------------------------------- #
# Physics-level command handling
# --------------------------------------------------------------------------- #
def test_world_rejects_out_of_bounds_command(world):
    with pytest.raises(CommandError):
        world.set_velocity_command(5.0, 0.0, 0.5)
    state = world.get_robot_state()
    assert not state.command.active


def test_wheel_motor_targets_follow_kinematics_after_ramp(world, kin):
    world.set_velocity_command(0.4, 0.6, 2.0)
    world.step(240)  # 1 s: well past the acceleration ramp
    state = world.get_robot_state()
    expected = kin.wheel_speeds(0.4, 0.6)
    assert state.command.linear == pytest.approx(0.4)
    assert state.command.angular == pytest.approx(0.6)
    assert state.wheels.target_left == pytest.approx(expected.left)
    assert state.wheels.target_right == pytest.approx(expected.right)
    # The physical wheel joints actually track the targets.
    assert state.wheels.left.velocity == pytest.approx(expected.left, abs=0.2)
    assert state.wheels.right.velocity == pytest.approx(expected.right, abs=0.2)


def test_acceleration_is_limited(world, config):
    world.set_velocity_command(1.0, 0.0, 1.0)
    world.step(24)  # 0.1 s
    state = world.get_robot_state()
    assert state.command.linear == pytest.approx(config.robot.limits.max_linear_acceleration * 0.1, abs=1e-6)
