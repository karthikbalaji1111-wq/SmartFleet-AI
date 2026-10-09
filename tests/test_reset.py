"""Reset returns the world to its configured initial state."""

from __future__ import annotations

import math

import pytest

from simulation.geometry import container_initial_position

from .conftest import drive, wrap_angle


def test_reset_restores_initial_state(world, config):
    drive(world, 0.6, 0.8, 2.0)
    moved = world.get_robot_state()
    start = config.robot.start_pose
    assert math.dist(moved.pose.position[:2], (start.x, start.y)) > 0.3

    world.reset()
    s = world.get_robot_state()
    assert world.sim_time == 0.0 and world.step_count == 0
    assert s.pose.position[0] == pytest.approx(start.x, abs=1e-3)
    assert s.pose.position[1] == pytest.approx(start.y, abs=1e-3)
    assert abs(wrap_angle(s.pose.heading - start.yaw)) < 1e-3
    assert abs(s.velocity.forward) < 0.01 and abs(s.velocity.yaw_rate) < 0.01
    assert not s.command.active and s.command.linear == 0.0 and s.command.angular == 0.0
    assert s.lift.position == pytest.approx(config.robot.lift.lower, abs=1e-3)
    assert s.forks.position == pytest.approx(config.robot.forks.lower, abs=1e-3)
    assert world.get_container_state().position == pytest.approx(
        container_initial_position(config.warehouse), abs=0.01
    )


def test_reset_keeps_world_consistent(world, pb):
    bodies_before = pb.getNumBodies(physicsClientId=world.client_id)
    world.reset()
    world.reset()
    assert pb.getNumBodies(physicsClientId=world.client_id) == bodies_before
    assert set(world.model.joints)  # model re-validated after rebuild


def test_simulation_is_repeatable_after_reset(world):
    first = drive(world, 0.5, 0.4, 1.5)[-1]
    world.reset()
    second = drive(world, 0.5, 0.4, 1.5)[-1]
    assert second.pose.position == pytest.approx(first.pose.position, abs=1e-4)
    assert second.pose.heading == pytest.approx(first.pose.heading, abs=1e-4)
