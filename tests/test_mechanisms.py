"""Physics-backed lift and fork control (PyBullet position motors on prismatic joints).

Every test advances the real simulation and checks joint states measured by
PyBullet; nothing is mocked.
"""

from __future__ import annotations

import math

import pytest

from simulation.diff_drive import CommandError
from simulation.mechanisms import MECHANISM_JOINTS, OVERLOAD_TIME, STALL_TIME, lift_presets
from simulation.robot_model import FORK_LINK, fork_surface_height

from .conftest import drive


def _state(world, name):
    return getattr(world.get_robot_state(), name)


def _track(world, name, seconds):
    """Step for ``seconds`` and return the mechanism telemetry after every step."""
    samples = []
    for _ in range(round(seconds / world.timestep)):
        world.step(1)
        samples.append(_state(world, name))
    return samples


def _fork_tip_forward(world, pb):
    """Fork link position projected on the robot's heading (world frame), m."""
    link = pb.getLinkState(world.robot_id, world.model.links[FORK_LINK], computeForwardKinematics=True,
                           physicsClientId=world.client_id)  # fmt: skip
    yaw = world.get_robot_state().pose.heading
    return link[4][0] * math.cos(yaw) + link[4][1] * math.sin(yaw)


def test_mechanism_joints_are_discovered_by_name(world, pb):
    c, body = world.client_id, world.robot_id
    by_name = {
        pb.getJointInfo(body, i, physicsClientId=c)[1].decode(): i for i in range(pb.getNumJoints(body, physicsClientId=c))
    }
    for name, joint in MECHANISM_JOINTS.items():
        assert world.model.joints[joint] == by_name[joint], name
        info = pb.getJointInfo(body, by_name[joint], physicsClientId=c)
        assert info[2] == pb.JOINT_PRISMATIC
    assert pb.getConnectionInfo(physicsClientId=c)["connectionMethod"] == pb.DIRECT


def test_lift_raises_under_motor_control_with_speed_limit(world, config):
    lift = config.robot.lift
    start = _state(world, "lift")
    cmd = world.set_mechanism_target("lift", 1.0)
    assert (cmd.target, cmd.previous_target) == (1.0, lift.default_position)
    samples = _track(world, "lift", 4.0)

    positions = [s.position for s in samples]
    assert all(b >= a - 1e-6 for a, b in zip(positions, positions[1:])), "raise should be monotonic"
    assert max(abs(s.velocity) for s in samples) <= lift.max_velocity * 1.02
    assert all(abs(b - a) <= lift.max_velocity * world.timestep * 1.5 for a, b in zip(positions, positions[1:]))
    # 1.0 m at 0.30 m/s takes ~3.33 s.
    reached = next(i for i, s in enumerate(samples) if s.at_target) * world.timestep
    assert reached == pytest.approx(1.0 / lift.max_velocity, abs=0.15)
    assert samples[int(1.0 / world.timestep)].state == "moving"
    end = samples[-1]
    assert end.position == pytest.approx(1.0, abs=lift.position_tolerance)
    assert end.state == "holding" and end.at_target and end.fault is None
    # Holding the 10 kg carriage + forks against gravity: ~98 N.
    assert end.applied_force == pytest.approx((config.robot.lift.carriage_mass + config.robot.forks.mass) * 9.81, rel=0.05)
    surface = world.get_robot_state().fork_surface_height
    assert surface - fork_surface_height(config.robot, start.position) == pytest.approx(1.0, abs=0.01)


def test_lift_lowers_to_requested_position(world, config):
    world.set_mechanism_target("lift", 1.2)
    _track(world, "lift", 4.5)
    world.set_mechanism_target("lift", 0.25)
    samples = _track(world, "lift", 3.5)
    positions = [s.position for s in samples]
    assert all(b <= a + 1e-6 for a, b in zip(positions, positions[1:])), "lowering should be monotonic"
    assert samples[-1].position == pytest.approx(0.25, abs=config.robot.lift.position_tolerance)
    assert samples[-1].state == "holding"


def test_forks_extend_and_retract(world, pb, config):
    forks = config.robot.forks
    tip0 = _fork_tip_forward(world, pb)
    world.set_mechanism_target("forks", 0.5)
    samples = _track(world, "forks", 2.5)
    assert max(abs(s.velocity) for s in samples) <= forks.max_velocity * 1.02
    assert samples[-1].position == pytest.approx(0.5, abs=forks.position_tolerance)
    # The fork link physically moved 0.5 m forward along the robot heading.
    assert _fork_tip_forward(world, pb) - tip0 == pytest.approx(0.5, abs=0.01)

    world.set_mechanism_target("forks", 0.0)
    samples = _track(world, "forks", 2.5)
    assert samples[-1].position == pytest.approx(0.0, abs=forks.position_tolerance)
    assert samples[-1].state == "holding"
    assert _fork_tip_forward(world, pb) == pytest.approx(tip0, abs=0.01)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("lift", 1.61),
        ("lift", -0.01),
        ("lift", math.nan),
        ("lift", math.inf),
        ("lift", True),
        ("lift", "1.0"),
        ("forks", 0.61),
        ("forks", -0.2),
    ],
)
def test_invalid_targets_are_rejected(world, name, value):
    before = _state(world, name).target
    with pytest.raises(CommandError):
        world.set_mechanism_target(name, value)
    assert _state(world, name).target == before


def test_out_of_range_message_is_clear(world):
    with pytest.raises(CommandError, match=r"lift target 1\.900 m is outside the limits \[0\.00, 1\.60\] m"):
        world.set_mechanism_target("lift", 1.9)


def test_jog_moves_target_and_saturates_at_limits(world, config):
    forks = config.robot.forks
    targets = [world.jog_mechanism("forks", 0.2) for _ in range(3)]  # 0.2, 0.4, 0.6
    assert [t.target for t in targets] == pytest.approx([0.2, 0.4, 0.6])
    assert not any(t.clamped for t in targets)
    cmd = world.jog_mechanism("forks", 0.2)  # would be 0.8 m
    assert cmd.target == forks.upper and cmd.clamped
    _track(world, "forks", 3.0)
    assert _state(world, "forks").position == pytest.approx(forks.upper, abs=forks.position_tolerance)
    with pytest.raises(CommandError, match="jog step"):
        world.jog_mechanism("forks", forks.max_jog_step + 0.01)
    with pytest.raises(CommandError, match="non-zero"):
        world.jog_mechanism("lift", 0.0)


def test_joint_limits_hold_even_if_motor_is_driven_past_them(world, pb, config):
    """Bypass validation entirely: PyBullet's URDF joint limits still stop travel."""
    c, body = world.client_id, world.robot_id
    for joint, cfg, beyond in (("lift_joint", config.robot.lift, 3.0), ("fork_joint", config.robot.forks, 2.0)):
        pb.setJointMotorControl2(body, world.model.joints[joint], pb.POSITION_CONTROL, targetPosition=beyond,
                                 force=cfg.max_force, maxVelocity=cfg.max_velocity, physicsClientId=c)  # fmt: skip
    peak = {"lift_joint": 0.0, "fork_joint": 0.0}
    for _ in range(round(8.0 / world.timestep)):
        world.step(1)
        for joint in peak:
            peak[joint] = max(peak[joint], pb.getJointState(body, world.model.joints[joint], physicsClientId=c)[0])
    assert peak["lift_joint"] <= config.robot.lift.upper + 2e-3
    assert peak["fork_joint"] <= config.robot.forks.upper + 2e-3
    assert peak["lift_joint"] == pytest.approx(config.robot.lift.upper, abs=2e-3)


@pytest.mark.parametrize(("name", "far"), [("lift", 1.5), ("forks", 0.6)])
def test_stop_holds_mechanism_mid_travel(world, name, far):
    world.set_mechanism_target(name, far)
    _track(world, name, 1.0)
    moving = _state(world, name)
    assert moving.state == "moving" and moving.position > 0.15
    cmd = world.stop_mechanism(name)
    assert cmd.target == pytest.approx(moving.position, abs=0.01)
    samples = _track(world, name, 2.0)
    assert max(abs(s.position - cmd.target) for s in samples[24:]) < 0.01, "no continued commanded movement"
    assert samples[-1].state == "holding" and samples[-1].target == cmd.target
    assert samples[-1].position < far - 0.3


def test_blocked_mechanism_is_detected_and_held(world, pb):
    """A beam above the forks stops the lift; it must be flagged and held, not pushed on forever."""
    c = world.client_id
    beam = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.3, 0.18, 0.05), physicsClientId=c)
    pb.createMultiBody(0, beam, basePosition=(5.0, 0.0, 1.05), physicsClientId=c)  # over the forks at the start pose
    base0 = world.get_robot_state().pose.position
    world.set_mechanism_target("lift", 1.5)
    samples = _track(world, "lift", 4.0)
    end = samples[-1]
    assert end.state == "blocked" and end.fault == "overload"
    assert 0.4 < end.position < 0.75, f"stopped at {end.position:.3f} m"
    assert end.target == pytest.approx(end.position, abs=0.01), "blocked mechanism holds where it stopped"
    first_blocked = next(i for i, s in enumerate(samples) if s.state == "blocked")
    contact = next(i for i, s in enumerate(samples) if abs(s.velocity) < 0.01 and s.position > 0.3)
    # Motor force saturates on contact, so overload trips well before the stall timer.
    assert (first_blocked - contact) * world.timestep <= OVERLOAD_TIME + 0.05
    assert (first_blocked - contact) * world.timestep < STALL_TIME
    events = world.drain_events()
    assert any(level == "warning" and "lift blocked" in msg and "overload" in msg for level, msg in events)
    assert math.dist(base0[:2], world.get_robot_state().pose.position[:2]) < 0.005
    # A new command clears the fault and the lift moves away from the obstruction.
    world.set_mechanism_target("lift", 0.1)
    samples = _track(world, "lift", 2.5)
    assert samples[-1].state == "holding"
    assert samples[-1].position == pytest.approx(0.1, abs=0.005)


def test_lift_blocked_by_rack_shelf_names_the_obstacle(world, pb):
    """Regression for a Robot Lab session: forks extended into rack C2 between
    shelf levels, then the lift raised into the shelf above. The lift must stop
    at that shelf's underside and the event must name it."""
    c = world.client_id
    # Facing rack C2 (south), tine tips 0.2 m inside the rack, clear of uprights.
    pb.resetBasePositionAndOrientation(world.robot_id, (1.79, -0.455, 0.101),
                                       pb.getQuaternionFromEuler((0, 0, -math.pi / 2)), physicsClientId=c)  # fmt: skip
    world.step(120)
    world.set_mechanism_target("lift", 0.10)  # tines just above shelf L1 (top 0.40 m)
    world.step(round(1.0 / world.timestep))
    world.set_mechanism_target("forks", 0.6)
    world.step(round(3.0 / world.timestep))
    assert world.get_robot_state().forks.at_target, "forks slide into the rack between shelves"
    world.drain_events()
    base0 = world.get_robot_state().pose.position
    world.set_mechanism_target("lift", 0.98)
    samples = []
    for _ in range(round(4.0 / world.timestep)):
        world.step(1)
        samples.append(world.get_robot_state())
    s = samples[-1]
    assert s.lift.state == "blocked" and s.lift.fault == "overload"
    assert s.fork_surface_height == pytest.approx(0.81, abs=0.01), "stopped at the underside of shelf L2"
    events = world.drain_events()
    assert any("lift blocked by C2/shelf-1" in msg for _, msg in events), events
    # The jammed lift must not lever the robot off the floor (it pitched 26° with a 2000 N motor).
    assert max(max(abs(x.pose.pitch), abs(x.pose.roll)) for x in samples) < 0.02
    assert math.dist(base0[:2], s.pose.position[:2]) < 0.01


def test_tilt_interlock_stops_moving_mechanisms(world, pb):
    """If the chassis tilts past limits.max_handling_tilt while the lift moves,
    the lift is stopped immediately."""
    world.set_mechanism_target("lift", 1.5)
    world.step(round(0.5 / world.timestep))
    assert world.get_robot_state().lift.state == "moving"
    # Test harness disturbance: tip the chassis by 0.08 rad (> 0.05 rad limit).
    pos = world.get_robot_state().pose.position
    pb.resetBasePositionAndOrientation(world.robot_id, (pos[0], pos[1], pos[2] + 0.05),
                                       pb.getQuaternionFromEuler((0.0, 0.08, math.pi)),
                                       physicsClientId=world.client_id)  # fmt: skip
    world.step(1)
    lift = world.get_robot_state().lift
    assert lift.state == "blocked" and lift.fault == "tilt"
    assert lift.target < 0.3
    assert any("tilt" in msg for _, msg in world.drain_events())


def test_mechanisms_do_not_disturb_parked_chassis(world):
    start = world.get_robot_state()
    world.set_mechanism_target("lift", 1.6)
    world.set_mechanism_target("forks", 0.6)
    world.step(round(6.0 / world.timestep))
    end = world.get_robot_state()
    assert end.lift.at_target and end.forks.at_target
    assert math.dist(start.pose.position[:2], end.pose.position[:2]) < 0.002
    assert abs(end.pose.heading - start.pose.heading) < math.radians(0.1)
    assert abs(end.pose.pitch) < 0.02 and end.wheels.brake_engaged


def test_chassis_still_drives_with_lift_raised(world, config):
    world.set_mechanism_target("lift", 1.0)
    world.step(round(4.0 / world.timestep))
    start = world.get_robot_state()
    samples = drive(world, 0.5, 0.0, 2.0)
    end = samples[-1]
    travelled = math.dist(start.pose.position[:2], end.pose.position[:2])
    # 0.5 m/s with a 1 m/s^2 ramp for 2 s = 0.875 m.
    assert travelled == pytest.approx(0.875, abs=0.06)
    assert max(abs(s.lift.position - 1.0) for s in samples) < 0.01, "lift holds height while driving"
    assert max(abs(s.pose.pitch) for s in samples) < 0.05


def test_reset_restores_mechanism_defaults(world, config):
    world.set_mechanism_target("lift", 0.8)
    world.set_mechanism_target("forks", 0.3)
    world.step(round(3.0 / world.timestep))
    world.reset()
    s = world.get_robot_state()
    for mech, cfg in ((s.lift, config.robot.lift), (s.forks, config.robot.forks)):
        assert mech.target == cfg.default_position
        assert mech.position == pytest.approx(cfg.default_position, abs=1e-3)
        assert mech.state == "holding"


def test_lift_presets_align_forks_with_real_surfaces(config):
    presets = {p.id: p for p in lift_presets(config)}
    lift = config.robot.lift
    assert presets["home"].position == lift.default_position
    levels = sorted({lvl for rt in config.warehouse.rack_types.values() for lvl in rt.levels})
    for i, level in enumerate(levels, start=1):
        p = presets[f"shelf-{i}"]
        assert lift.lower <= p.position <= lift.upper
        assert p.surface_height == pytest.approx(level + 0.02)
    assert presets["stand-1"].surface_height == pytest.approx(config.warehouse.stations[0].size[2] + 0.02)


def test_preset_reached_in_physics(world, config):
    preset = next(p for p in lift_presets(config) if p.id == "shelf-2")
    world.set_mechanism_target("lift", preset.position)
    world.step(round(3.0 / world.timestep))
    s = world.get_robot_state()
    assert s.fork_surface_height == pytest.approx(preset.surface_height, abs=0.01)
