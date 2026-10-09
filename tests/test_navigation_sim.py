"""Autonomous navigation in PyBullet: the planned route is driven by the real
differential-drive dynamics (wheel motors + contact friction)."""

from __future__ import annotations

import math

import pytest

from simulation.navigation.navigator import MotionConflictError, NavigationError


def _nav_to(world, destination_id=None, **point):
    dest = world.navigator.resolve_destination(destination_id, **point)
    plan = world.plan_navigation(dest)
    world.start_navigation()
    return dest, plan


def _run(world, pb=None, max_time=90.0, every=8):
    """Step until navigation ends; collect measured states and static contacts."""
    samples, contacts = [], 0
    statics = set(world.static_body_ids.values())
    start = world.sim_time
    while world.navigator.status == "navigating" and world.sim_time - start < max_time:
        world.step(every)
        samples.append(world.get_robot_state())
        if pb is not None:
            contacts += sum(
                1 for p in pb.getContactPoints(bodyA=world.robot_id, physicsClientId=world.client_id) if p[2] in statics
            )
    return samples, contacts


def _xy(state):
    return state.pose.position[0], state.pose.position[1]


def test_robot_starts_at_configured_pose_with_idle_navigation(world, config):
    s = world.get_robot_state()
    assert _xy(s) == pytest.approx((config.robot.start_pose.x, config.robot.start_pose.y), abs=1e-3)
    nav = world.get_state().navigation
    assert nav.status == "idle" and nav.destination is None and nav.waypoint_count == 0
    assert nav.interlock == []


def test_navigation_moves_the_physical_robot_continuously(world, config):
    start = world.get_robot_state()
    _nav_to(world, "aisle-bc")
    samples = []
    for _ in range(round(3.0 / world.timestep)):
        world.step(1)
        samples.append(world.get_robot_state())
    assert world.navigator.status == "navigating"
    assert math.dist(_xy(start), _xy(samples[-1])) > 0.8, "robot must actually travel"
    limit = config.robot.limits.max_linear_velocity * world.timestep * 1.5
    prev = _xy(start)
    for s in samples:
        assert math.dist(prev, _xy(s)) <= limit, "no teleporting: per-step motion bounded by speed limit"
        prev = _xy(s)
    wheel_turn = samples[-1].wheels.left.position - start.wheels.left.position
    assert abs(wheel_turn) * config.robot.wheels.radius > 0.6, "motion comes from wheel rotation"
    assert world.get_state().navigation.command_linear > 0


def test_turn_around_rack_end_without_touching_obstacles(world, pb, config):
    """Home -> aisle A/B: drive west, turn north around rack B2's east end, turn west into the aisle."""
    dest, plan = _nav_to(world, "aisle-ab")
    assert len(plan.waypoints) >= 3, "route has real corners"
    samples, contacts = _run(world, pb)
    nav = world.navigator
    assert nav.status == "arrived", nav.reason
    assert contacts == 0, "robot never touched a rack, wall or stand"
    grid = nav.grid
    # The chassis centre stayed in planner-free space (footprint clear in any heading),
    # allowing for the waypoint tolerance at gentle corners.
    worst = min(grid.cell_clearance(*_xy(s)) for s in samples)
    assert worst >= grid.inflation_radius - config.navigation.controller.waypoint_tolerance
    assert worst >= grid.robot_radius, "physical footprint clearance kept"
    end = world.get_robot_state()
    assert math.dist(_xy(end), (dest.x, dest.y)) <= config.navigation.controller.goal_tolerance
    assert max(abs(s.pose.pitch) for s in samples) < 0.02


def test_robot_stops_and_stays_at_destination(world, config):
    dest, _ = _nav_to(world, x=2.5, y=0.0)
    _run(world)
    assert world.navigator.status == "arrived"
    arrived = world.get_robot_state()
    assert math.dist(_xy(arrived), (2.5, 0.0)) <= config.navigation.controller.goal_tolerance
    assert abs(arrived.velocity.forward) < 0.02 and not arrived.command.active
    world.step(round(3.0 / world.timestep))
    later = world.get_robot_state()
    assert math.dist(_xy(arrived), _xy(later)) < 0.005, "wheels stopped (parking brake holds)"
    assert later.wheels.brake_engaged
    tel = world.get_state().navigation
    assert tel.progress == 1.0 and tel.remaining_distance == 0.0 and tel.command_linear == 0.0


def test_destination_heading_is_reached(world, config):
    dest, _ = _nav_to(world, "loading-staging")
    _run(world)
    assert world.navigator.status == "arrived"
    s = world.get_robot_state()
    err = (s.pose.heading - dest.yaw + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) <= config.navigation.controller.heading_tolerance + 0.01
    assert math.dist(_xy(s), (dest.x, dest.y)) <= config.navigation.controller.goal_tolerance


def test_pause_stops_forward_motion_and_resume_completes(world, config):
    _nav_to(world, "aisle-bc")
    world.step(round(4.0 / world.timestep))
    assert world.get_robot_state().velocity.forward > 0.4
    assert world.pause_navigation()
    world.step(round(1.5 / world.timestep))
    paused = world.get_robot_state()
    assert world.navigator.status == "paused"
    assert abs(paused.velocity.forward) < 0.02, "pause brakes the robot"
    world.step(round(2.0 / world.timestep))
    assert math.dist(_xy(paused), _xy(world.get_robot_state())) < 0.005, "no motion while paused"
    world.resume_navigation()
    _run(world)
    assert world.navigator.status == "arrived"


def test_cancel_stops_the_robot(world):
    _nav_to(world, "aisle-bc")
    world.step(round(4.0 / world.timestep))
    assert world.cancel_navigation()
    world.step(round(1.5 / world.timestep))
    s = world.get_robot_state()
    assert world.navigator.status == "cancelled"
    assert abs(s.velocity.forward) < 0.02 and not s.command.active
    with pytest.raises(NavigationError, match="plan it again"):
        world.start_navigation()


@pytest.mark.parametrize(
    ("mechanism", "position", "needle"),
    [("forks", 0.3, "forks extended"), ("lift", 0.5, "lift at")],
)
def test_unsafe_lift_or_fork_configuration_blocks_navigation(world, mechanism, position, needle):
    world.set_mechanism_target(mechanism, position)
    world.step(round(3.0 / world.timestep))
    before = world.get_robot_state()
    dest = world.navigator.resolve_destination("aisle-bc")
    world.plan_navigation(dest)  # planning is allowed; it moves nothing
    with pytest.raises(NavigationError) as err:
        world.start_navigation()
    assert err.value.code == "interlock"
    assert any(needle in r for r in err.value.reasons)
    world.step(round(1.0 / world.timestep))
    after = world.get_robot_state()
    assert math.dist(_xy(before), _xy(after)) < 0.002, "robot did not move"
    assert getattr(after, mechanism).position == pytest.approx(position, abs=0.01), "mechanism untouched by navigation"
    assert world.get_state().navigation.interlock


def test_mechanism_commands_rejected_while_navigating(world):
    _nav_to(world, "aisle-bc")
    world.step(48)
    with pytest.raises(MotionConflictError):
        world.set_mechanism_target("lift", 0.5)
    with pytest.raises(MotionConflictError):
        world.jog_mechanism("forks", 0.1)
    world.cancel_navigation()
    world.set_mechanism_target("lift", 0.05)  # allowed again once navigation stopped


def test_blocked_route_fails_instead_of_pushing_forever(world, pb, config):
    """An obstacle that is not on the static map: the robot must stop and report failure."""
    _nav_to(world, "aisle-bc")
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.2, 1.4, 0.5), physicsClientId=world.client_id)
    pb.createMultiBody(0, shape, basePosition=(1.0, 0.25, 0.5), physicsClientId=world.client_id)
    _run(world, max_time=40.0)
    nav = world.navigator
    assert nav.status == "failed"
    assert "no progress" in nav.reason or "left the planned route" in nav.reason
    world.step(round(1.0 / world.timestep))
    assert abs(world.get_robot_state().velocity.forward) < 0.02


def test_start_rejected_if_robot_moved_since_planning(world):
    world.plan_navigation(world.navigator.resolve_destination("aisle-bc"))
    world.set_velocity_command(0.6, 0.0, 2.0)
    world.step(round(2.0 / world.timestep))
    with pytest.raises(NavigationError) as err:
        world.start_navigation()
    assert err.value.code == "route_invalid"


def test_floor_point_inside_rack_is_snapped_or_rejected(world):
    nav = world.navigator
    with pytest.raises(NavigationError, match="clearance zone of B2"):
        nav.resolve_destination(x=0.0, y=1.4, snap=False)
    dest = nav.resolve_destination(x=0.0, y=1.4, snap=True)
    assert dest.snapped and nav.grid.is_free(*nav.grid.world_to_cell(dest.x, dest.y))
    assert math.dist((dest.x, dest.y), (0.0, 1.4)) <= world.config.navigation.grid.goal_snap_radius
    with pytest.raises(NavigationError, match="no navigable floor"):
        nav.resolve_destination(x=0.0, y=2.0, snap=True)  # middle of rack B2
    with pytest.raises(NavigationError, match="outside"):
        nav.resolve_destination(x=50.0, y=0.0)


def test_reset_clears_navigation(world):
    _nav_to(world, "aisle-bc")
    world.step(240)
    version = world.navigator.route_version
    world.reset()
    nav = world.get_state().navigation
    assert nav.status == "idle" and nav.destination is None
    assert nav.route_version > version


def test_manual_drive_still_works_after_navigation(world):
    _nav_to(world, x=3.0, y=0.0)
    _run(world)
    assert world.navigator.status == "arrived"
    start = world.get_robot_state()
    for _ in range(4):
        world.set_velocity_command(0.0, 1.0, 0.5)
        world.step(120)
    assert abs(world.get_robot_state().pose.heading - start.pose.heading) > 0.5
