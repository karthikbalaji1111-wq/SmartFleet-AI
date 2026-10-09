"""Robot pose and telemetry validity; PyBullet is the only source of truth."""

from __future__ import annotations

import json
import math

import pytest

from simulation.geometry import container_initial_position
from simulation.robot_model import fork_surface_height

from .conftest import wrap_angle


def _all_floats(obj):
    if isinstance(obj, dict):
        for v in obj.values():
            yield from _all_floats(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            yield from _all_floats(v)
    elif isinstance(obj, float):
        yield obj


def test_initial_pose_matches_configured_start(world, config):
    s = world.get_robot_state()
    start = config.robot.start_pose
    assert s.pose.position[0] == pytest.approx(start.x, abs=1e-3)
    assert s.pose.position[1] == pytest.approx(start.y, abs=1e-3)
    assert abs(wrap_angle(s.pose.heading - start.yaw)) < 1e-3
    # Body-frame origin is the axle midpoint, so it rests one wheel radius up.
    assert s.pose.position[2] == pytest.approx(config.robot.wheels.radius, abs=5e-3)
    assert abs(s.pose.roll) < 0.01 and abs(s.pose.pitch) < 0.01
    assert math.isclose(sum(q * q for q in s.pose.orientation), 1.0, abs_tol=1e-6)
    assert s.pose.heading_deg == pytest.approx(math.degrees(s.pose.heading))


def test_initial_state_is_safe_and_at_rest(world, config):
    s = world.get_robot_state()
    assert not s.command.active
    assert s.command.linear == 0.0 and s.command.angular == 0.0
    assert abs(s.velocity.forward) < 0.01 and abs(s.velocity.yaw_rate) < 0.01
    assert s.lift.position == pytest.approx(config.robot.lift.lower, abs=1e-3)
    assert s.forks.position == pytest.approx(config.robot.forks.lower, abs=1e-3)
    assert (s.lift.lower, s.lift.upper) == (config.robot.lift.lower, config.robot.lift.upper)
    assert (s.forks.lower, s.forks.upper) == (config.robot.forks.lower, config.robot.forks.upper)
    for mech, cfg in ((s.lift, config.robot.lift), (s.forks, config.robot.forks)):
        assert mech.state == "holding" and mech.at_target
        assert mech.target == cfg.default_position
        assert abs(mech.velocity) < 1e-3
    # Fork surface height comes from the fork link pose in PyBullet.
    assert s.fork_surface_height == pytest.approx(fork_surface_height(config.robot, s.lift.position), abs=5e-3)


def test_lift_and_forks_hold_position_over_time(world, config):
    world.step(480)
    s = world.get_robot_state()
    assert s.lift.position == pytest.approx(config.robot.lift.lower, abs=2e-3)
    assert s.forks.position == pytest.approx(config.robot.forks.lower, abs=2e-3)


def test_telemetry_is_finite_and_json_serializable(world):
    state = world.get_state()
    payload = json.loads(state.model_dump_json())
    values = list(_all_floats(payload))
    assert values and all(math.isfinite(v) for v in values)
    assert payload["robot"]["id"] == "amr-01"


def test_container_rests_on_station(world, config):
    c = world.get_container_state()
    expected = container_initial_position(config.warehouse)
    assert c.position == pytest.approx(expected, abs=0.01)


def test_state_is_read_from_pybullet_not_cached(world, pb):
    """Moving the body inside PyBullet is immediately visible in telemetry,
    proving there is no second, independently integrated pose."""
    pb.resetBasePositionAndOrientation(
        world.robot_id, (1.0, -2.0, 0.1), pb.getQuaternionFromEuler((0, 0, 0.5)), physicsClientId=world.client_id
    )
    s = world.get_robot_state()
    assert s.pose.position[:2] == pytest.approx((1.0, -2.0))
    assert s.pose.heading == pytest.approx(0.5)
