"""Actual robot movement in the physics simulation.

These tests drive the wheel motors and check what PyBullet reports: the robot
must translate/rotate as commanded, continuously (no teleporting), and the
distance travelled must be explained by wheel rotation.
"""

from __future__ import annotations

import math

import pytest

from .conftest import drive, unwrapped_yaw_change


def _xy(state):
    return state.pose.position[0], state.pose.position[1]


def _assert_continuous(world, start, samples, max_speed):
    """No step may move the robot further than physically possible."""
    limit = max_speed * world.timestep * 1.5 + 1e-4
    prev = _xy(start)
    for s in samples:
        cur = _xy(s)
        assert math.dist(prev, cur) <= limit, f"jump of {math.dist(prev, cur):.4f} m in one step"
        prev = cur


def test_drives_forward_along_heading(world, config):
    start = world.get_robot_state()
    yaw0 = start.pose.heading
    samples = drive(world, 0.5, 0.0, 3.0)
    end = samples[-1]
    dx, dy = end.pose.position[0] - start.pose.position[0], end.pose.position[1] - start.pose.position[1]
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    # Ideal: 0.5 m/s with a 1 m/s^2 ramp for 3 s = 1.375 m.
    assert 1.2 < forward < 1.45, f"forward displacement {forward:.3f} m"
    assert abs(lateral) < 0.05, f"lateral drift {lateral:.3f} m"
    assert abs(unwrapped_yaw_change(yaw0, samples)) < math.radians(2)
    assert end.velocity.forward == pytest.approx(0.5, abs=0.05)
    assert abs(end.pose.roll) < 0.05 and abs(end.pose.pitch) < 0.05
    _assert_continuous(world, start, samples, 0.5)

    # Motion is explained by wheel rotation (rolling, little slip).
    wheel_travel = (
        (end.wheels.left.position - start.wheels.left.position)
        + (end.wheels.right.position - start.wheels.right.position)
    ) / 2 * config.robot.wheels.radius
    assert wheel_travel == pytest.approx(forward, rel=0.08)


def test_drives_in_reverse(world):
    start = world.get_robot_state()
    yaw0 = start.pose.heading
    samples = drive(world, -0.4, 0.0, 2.0)
    end = samples[-1]
    dx, dy = end.pose.position[0] - start.pose.position[0], end.pose.position[1] - start.pose.position[1]
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    assert -0.75 < forward < -0.55, f"reverse displacement {forward:.3f} m"
    assert end.velocity.forward == pytest.approx(-0.4, abs=0.05)


@pytest.mark.parametrize("direction", [1.0, -1.0])
def test_turns_in_place(world, direction):
    start = world.get_robot_state()
    samples = drive(world, 0.0, direction * 1.0, 2.0)
    end = samples[-1]
    turned = unwrapped_yaw_change(start.pose.heading, samples)
    # Ideal: 1 rad/s with a 3 rad/s^2 ramp for 2 s = 1.83 rad.
    assert 1.6 < direction * turned < 1.95, f"turned {turned:.3f} rad"
    assert math.dist(_xy(start), _xy(end)) < 0.05, "in-place turn should not translate"
    assert end.velocity.yaw_rate == pytest.approx(direction * 1.0, abs=0.1)
    _assert_continuous(world, start, samples, 0.5)


def test_arc_combines_translation_and_rotation(world):
    start = world.get_robot_state()
    samples = drive(world, 0.5, 0.5, 2.0)
    end = samples[-1]
    assert unwrapped_yaw_change(start.pose.heading, samples) > 0.6
    assert math.dist(_xy(start), _xy(end)) > 0.6


def test_dead_man_timeout_stops_robot(world):
    start = world.get_robot_state()
    world.set_velocity_command(0.6, 0.0, 0.4)
    world.step(int(3.0 / world.timestep))
    end = world.get_robot_state()
    assert not end.command.active
    assert abs(end.velocity.forward) < 0.02
    travelled = math.dist(_xy(start), _xy(end))
    assert 0.05 < travelled < 0.4, f"travelled {travelled:.3f} m on a 0.4 s request"


def test_stop_brakes_along_profile_without_slip(world, config):
    drive(world, 0.8, 0.0, 1.5)
    moving = world.get_robot_state()
    assert moving.velocity.forward > 0.7
    world.stop()
    samples = []
    for _ in range(int(1.0 / world.timestep)):
        world.step(1)
        samples.append(world.get_robot_state())
    end = samples[-1]
    assert abs(end.velocity.forward) < 0.02
    assert end.command.linear == 0.0 and not end.command.active
    # Braking distance matches the deceleration profile: v^2 / (2 a) ~ 0.16 m.
    braking = math.dist(_xy(moving), _xy(end))
    ideal = 0.8**2 / (2 * config.robot.limits.max_linear_deceleration)
    assert braking == pytest.approx(ideal, abs=0.05)
    assert abs(unwrapped_yaw_change(moving.pose.heading, samples)) < math.radians(0.3), "braking should not yaw"


def test_immediate_stop_brakes_hard(world):
    drive(world, 0.8, 0.0, 1.5)
    world.stop(immediate=True)
    world.step(int(1.0 / world.timestep))
    end = world.get_robot_state()
    assert abs(end.velocity.forward) < 0.05
    assert end.command.linear == 0.0


def test_no_motion_without_command(world):
    start = world.get_robot_state()
    world.step(int(2.0 / world.timestep))
    end = world.get_robot_state()
    assert math.dist(_xy(start), _xy(end)) < 2e-3
    assert abs(end.pose.heading - start.pose.heading) < 2e-3
    assert end.wheels.brake_engaged


def test_parking_brake_holds_pose_after_driving(world):
    """Regression: wheel velocity motors at 0 rad/s let the robot slowly spin
    in place; the parking brake must hold it still."""
    drive(world, 0.5, 0.0, 2.0)
    world.stop()
    world.step(int(1.5 / world.timestep))
    parked = world.get_robot_state()
    assert parked.wheels.brake_engaged
    world.step(int(10.0 / world.timestep))
    later = world.get_robot_state()
    assert math.dist(_xy(parked), _xy(later)) < 1e-3
    assert abs(later.pose.heading - parked.pose.heading) < math.radians(0.05)
    # A new request releases the brake and the robot drives again.
    samples = drive(world, 0.4, 0.0, 1.0)
    assert not samples[-1].wheels.brake_engaged
    assert samples[-1].velocity.forward > 0.3
