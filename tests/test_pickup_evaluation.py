"""Quantitative evaluation test suite for Milestone 7: Perception-Guided Autonomous Pickup.

Evaluates deterministic pickup across 4 realistic operational scenarios:
1. Scenario 1: Straight-ahead approach (Inbound stand, TOTE-0001).
2. Scenario 2: Lateral offset perturbations (+5 cm and -5 cm initial offsets at staging pose).
3. Scenario 3: Shelf tier 1 slot (Rack A1-B1-L1-S1, TOTE-0002).
4. Scenario 4: Target container absent/missing error handling.

Measures and asserts:
- Final lateral error (cm)
- Final heading error (deg)
- Alignment duration (s)
- Physical vertical elevation (m)
- Multi-stage pickup verification success rate (100% for valid targets)
"""

import math
import pytest
import pybullet as pb

from simulation.config import get_config
from simulation.tasks import TaskManager, TaskRequest


def test_scenario_1_straight_ahead_inbound_stand(world):
    """Scenario 1: Straight-ahead approach to inbound stand with TOTE-0001."""
    world.initialize()
    world.step(50)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Allow simulation steps to complete approach, acquire, align, extend, lift, verify
    verified = False
    for _ in range(6000):
        world.step(1)
        tm.tick(world.timestep)
        if tm.pickup_telemetry_data.verified_pickup:
            verified = True
            break
        if tm.state == "failed":
            pytest.fail(f"Scenario 1 failed: {tm.error}")

    assert verified, f"Pickup not verified. State: {tm.state}, err: {tm.error}"
    assert tm.pickup_telemetry_data.cargo_state == "attached"

    # Quantitative assertions
    lat_err = abs(tm.pickup_telemetry_data.lateral_error_cm)
    heading_err = abs(tm.pickup_telemetry_data.heading_error_deg)
    duration = tm.alignment_controller.state.elapsed

    assert lat_err <= 3.0, f"Lateral error exceeded tolerance: {lat_err:.2f} cm"
    assert heading_err <= 2.5, f"Heading error exceeded tolerance: {heading_err:.2f} deg"
    assert duration <= 6.0, f"Alignment duration exceeded 6.0s: {duration:.2f} s"

    # Physical elevation assertion
    c_id = world.container_ids["TOTE-0001"]
    pos, _ = pb.getBasePositionAndOrientation(c_id, physicsClientId=world._client)
    assert pos[2] > 0.86, f"Container was not physically lifted: z={pos[2]:.3f} m"


def test_scenario_2_lateral_perturbation_positive(world):
    """Scenario 2a: Initial +5 cm lateral offset at staging pose."""
    world.initialize()
    world.step(50)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Position robot at staging pose with +0.05m lateral offset (y=4.05 instead of y=4.00)
    pb.resetBasePositionAndOrientation(
        world.robot_id, [9.4, 4.05, 0.0], pb.getQuaternionFromEuler([0, 0, 0.0]), physicsClientId=world._client
    )
    world.navigator.reset()
    world.stop()
    tm.state = "acquire_target"
    tm.pickup_telemetry_data.stage = "acquire_target"
    view_lift = tm._calc_lift_target(tm._src_z + 0.05)
    world.set_mechanism_target("lift", view_lift)

    verified = False
    for _ in range(3500):
        world.step(1)
        tm.tick(world.timestep)
        if tm.pickup_telemetry_data.verified_pickup:
            verified = True
            break
        if tm.state == "failed":
            pytest.fail(f"Scenario 2 (+5cm) failed: {tm.error}")

    assert verified, "Pickup not verified under +5 cm lateral perturbation"
    assert tm.pickup_telemetry_data.cargo_state == "attached"

    lat_err = abs(tm.pickup_telemetry_data.lateral_error_cm)
    heading_err = abs(tm.pickup_telemetry_data.heading_error_deg)
    assert lat_err < 4.0, f"Lateral error not sufficiently corrected: {lat_err:.2f} cm"
    assert heading_err < 2.5, f"Heading error not sufficiently corrected: {heading_err:.2f} deg"


def test_scenario_2_lateral_perturbation_negative(world):
    """Scenario 2b: Initial -5 cm lateral offset at staging pose."""
    world.initialize()
    world.step(50)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Position robot at staging pose with -0.05m lateral offset (y=3.95 instead of y=4.00)
    pb.resetBasePositionAndOrientation(
        world.robot_id, [9.4, 3.95, 0.0], pb.getQuaternionFromEuler([0, 0, 0.0]), physicsClientId=world._client
    )
    world.navigator.reset()
    world.stop()
    tm.state = "acquire_target"
    tm.pickup_telemetry_data.stage = "acquire_target"
    view_lift = tm._calc_lift_target(tm._src_z + 0.05)
    world.set_mechanism_target("lift", view_lift)

    verified = False
    for _ in range(3500):
        world.step(1)
        tm.tick(world.timestep)
        if tm.pickup_telemetry_data.verified_pickup:
            verified = True
            break
        if tm.state == "failed":
            pytest.fail(f"Scenario 2 (-5cm) failed: {tm.error}")

    assert verified, "Pickup not verified under -5 cm lateral perturbation"
    assert tm.pickup_telemetry_data.cargo_state == "attached"

    lat_err = abs(tm.pickup_telemetry_data.lateral_error_cm)
    heading_err = abs(tm.pickup_telemetry_data.heading_error_deg)
    assert lat_err < 4.0, f"Lateral error not sufficiently corrected: {lat_err:.2f} cm"
    assert heading_err < 2.5, f"Heading error not sufficiently corrected: {heading_err:.2f} deg"


def test_scenario_3_shelf_tier_1_slot(world):
    """Scenario 3: Pickup from rack shelf tier 1 slot (TOTE-0002 at A1-B1-L1-S1)."""
    world.initialize()
    world.step(50)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0002", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Position robot at staging pose for A1-B1-L1-S1: x=-9.124, y=4.5, yaw=pi/2
    pb.resetBasePositionAndOrientation(
        world.robot_id, [-9.124, 4.5, 0.0], pb.getQuaternionFromEuler([0, 0, math.pi / 2.0]), physicsClientId=world._client
    )
    world.navigator.reset()
    world.stop()
    tm.state = "acquire_target"
    tm.pickup_telemetry_data.stage = "acquire_target"
    view_lift = tm._calc_lift_target(tm._src_z + 0.05)
    world.set_mechanism_target("lift", view_lift)

    verified = False
    for _ in range(3500):
        world.step(1)
        tm.tick(world.timestep)
        if tm.pickup_telemetry_data.verified_pickup:
            verified = True
            break
        if tm.state == "failed":
            pytest.fail(f"Scenario 3 (Shelf Tier 1) failed: {tm.error}")

    assert verified, "Shelf tier 1 pickup not verified"
    assert tm.pickup_telemetry_data.cargo_state == "attached"

    lat_err = abs(tm.pickup_telemetry_data.lateral_error_cm)
    heading_err = abs(tm.pickup_telemetry_data.heading_error_deg)
    assert lat_err < 3.0, f"Shelf lateral error exceeded tolerance: {lat_err:.2f} cm"
    assert heading_err < 2.5, f"Shelf heading error exceeded tolerance: {heading_err:.2f} deg"

    # Container physical height lifted
    c_id = world.container_ids["TOTE-0002"]
    pos, _ = pb.getBasePositionAndOrientation(c_id, physicsClientId=world._client)
    assert pos[2] > 0.50, f"Shelf container not lifted: z={pos[2]:.3f} m"


def test_scenario_4_absent_target_handling(world):
    """Scenario 4: Target container absent/missing from view; verifies safe graceful failure."""
    world.initialize()
    world.step(50)

    # Move target container far away out of line of sight
    pb.resetBasePositionAndOrientation(
        world.container_ids["TOTE-0001"], [-50.0, -50.0, 0.0], [0, 0, 0, 1], physicsClientId=world._client
    )

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Place robot at staging pose
    pb.resetBasePositionAndOrientation(
        world.robot_id, [9.4, 4.0, 0.0], pb.getQuaternionFromEuler([0, 0, 0.0]), physicsClientId=world._client
    )
    world.navigator.reset()
    world.stop()
    tm.state = "acquire_target"
    tm.pickup_telemetry_data.stage = "acquire_target"
    view_lift = tm._calc_lift_target(tm._src_z + 0.05)
    world.set_mechanism_target("lift", view_lift)

    # Step until failure timeout (6.0s = 1440 steps at 240 Hz)
    for _ in range(1800):
        world.step(1)
        tm.tick(world.timestep)
        if tm.state == "failed":
            break

    assert tm.state == "failed", f"Expected task failure for absent target, got state: {tm.state}"
    assert "Target acquisition failed" in tm.error
    assert tm.pickup_telemetry_data.cargo_state == "empty"
    assert not tm.pickup_telemetry_data.verified_pickup
