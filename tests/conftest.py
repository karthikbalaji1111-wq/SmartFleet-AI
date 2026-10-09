"""Shared fixtures. Physics tests are skipped (and reported) when PyBullet is
not installed; config, kinematics and API-contract tests always run."""

from __future__ import annotations

import math

import pytest

from simulation.config import AppConfig, get_config

PYBULLET_SKIP_REASON = "PyBullet not installed in this environment (see README: Installing PyBullet on Windows)"


@pytest.fixture(scope="session")
def config() -> AppConfig:
    return get_config()


@pytest.fixture
def pb():
    return pytest.importorskip("pybullet", reason=PYBULLET_SKIP_REASON)


@pytest.fixture
def world(pb, config):
    from simulation.world import SimulationWorld

    w = SimulationWorld(config)
    w.initialize()
    try:
        yield w
    finally:
        w.shutdown()


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def drive(world, linear: float, angular: float, seconds: float, *, resend: float = 0.25, duration: float = 0.5):
    """Hold a velocity request like the UI does (re-sent before the dead-man
    timeout) and return the robot state after every physics step."""
    steps = round(seconds / world.timestep)
    resend_steps = max(1, round(resend / world.timestep))
    samples = []
    for k in range(steps):
        if k % resend_steps == 0:
            world.set_velocity_command(linear, angular, duration)
        world.step(1)
        samples.append(world.get_robot_state())
    return samples


def unwrapped_yaw_change(start_yaw: float, samples) -> float:
    total, prev = 0.0, start_yaw
    for s in samples:
        total += wrap_angle(s.pose.heading - prev)
        prev = s.pose.heading
    return total
