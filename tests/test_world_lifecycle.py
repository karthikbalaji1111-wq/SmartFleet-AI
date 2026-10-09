"""Physics world initialization, geometry and cleanup."""

from __future__ import annotations

import pytest


def test_initializes_in_direct_mode_with_fixed_timestep(world, pb, config):
    c = world.client_id
    assert world.is_initialized
    assert pb.getConnectionInfo(physicsClientId=c)["connectionMethod"] == pb.DIRECT
    params = pb.getPhysicsEngineParameters(physicsClientId=c)
    assert params["fixedTimeStep"] == pytest.approx(1.0 / config.simulation.physics_hz)
    assert params["numSolverIterations"] == config.simulation.solver_iterations
    assert world.sim_time == 0.0 and world.step_count == 0


def test_world_contains_floor_static_geometry_robot_and_container(world, pb):
    n_bodies = pb.getNumBodies(physicsClientId=world.client_id)
    assert n_bodies == 1 + len(world.static_geometry) + 1 + 1
    assert world.floor_id >= 0 and world.robot_id >= 0 and world.container_id >= 0
    assert set(world.static_body_ids) == {b.id for b in world.static_geometry}


def test_collision_geometry_matches_canonical_boxes(world, pb):
    """Every PyBullet static body sits exactly where the shared geometry says
    (the same list the frontend renders)."""
    for box in world.static_geometry:
        body = world.static_body_ids[box.id]
        pos, _ = pb.getBasePositionAndOrientation(body, physicsClientId=world.client_id)
        assert pos == pytest.approx(box.center, abs=1e-9), box.id
        lo, hi = world.static_body_aabb(box.id)
        exp_lo, exp_hi = box.aabb
        for axis in range(3):
            # Bullet may pad AABBs by a small collision margin; never shrink them.
            assert exp_lo[axis] - 0.05 <= lo[axis] <= exp_lo[axis] + 1e-6, (box.id, axis)
            assert exp_hi[axis] - 1e-6 <= hi[axis] <= exp_hi[axis] + 0.05, (box.id, axis)


def test_step_advances_fixed_time(world, config):
    world.step(48)
    assert world.step_count == 48
    assert world.sim_time == pytest.approx(48 / config.simulation.physics_hz)


def test_shutdown_disconnects_and_is_idempotent(pb, config):
    from simulation.world import SimulationWorld, WorldNotInitializedError

    world = SimulationWorld(config)
    world.initialize()
    client, urdf_dir = world.client_id, world._urdf_dir
    assert urdf_dir is not None and urdf_dir.exists()
    world.shutdown()
    assert not world.is_initialized
    assert not pb.isConnected(physicsClientId=client)
    assert not urdf_dir.exists(), "generated URDF temp dir should be removed"
    world.shutdown()  # second call is a no-op
    with pytest.raises(WorldNotInitializedError):
        world.step()
    with pytest.raises(WorldNotInitializedError):
        world.get_robot_state()


def test_context_manager_and_independent_worlds(pb, config):
    from simulation.world import SimulationWorld

    with SimulationWorld(config) as a, SimulationWorld(config) as b:
        assert a.client_id != b.client_id
        a.set_velocity_command(0.5, 0.0, 1.0)
        a.step(240)
        assert a.get_robot_state().pose.position[0] != pytest.approx(b.get_robot_state().pose.position[0], abs=0.05)
        clients = (a.client_id, b.client_id)
    assert not any(pb.isConnected(physicsClientId=c) for c in clients)


def test_initialize_is_idempotent(world):
    client = world.client_id
    world.initialize()
    assert world.client_id == client
