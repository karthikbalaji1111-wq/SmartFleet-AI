import pytest
from simulation.config import get_config
from simulation.world import SimulationWorld
import pybullet as pb
from tests.test_navigation_sim import _nav_to, _run, _xy

def test_dynamic_obstacle_avoidance(world, pb, config):
    _nav_to(world, "aisle-bc")
    # This obstacle blocks the direct path
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.2, 1.4, 0.5), physicsClientId=world.client_id)
    pb.createMultiBody(0, shape, basePosition=(1.0, 0.25, 0.5), physicsClientId=world.client_id)
    
    # We should detect, replan, and eventually arrive
    _run(world, max_time=40.0)
    events = world._events
            
    assert any("Dynamic obstacle detected. Replanning" in ev[1] for ev in events)
    assert world.navigator.status == "arrived"

def test_dynamic_obstacle_unreachable(world, pb, config):
    _nav_to(world, "aisle-bc")
    # This obstacle completely blocks the aisle in a way that can't be navigated around
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(2.0, 20.0, 0.5), physicsClientId=world.client_id)
    pb.createMultiBody(0, shape, basePosition=(1.0, 0.0, 0.5), physicsClientId=world.client_id)
    
    _run(world, max_time=40.0)
    events = world._events
            
    assert world.navigator.status == "failed"
    assert any("Failed to replan" in ev[1] for ev in events)

def test_dynamic_obstacle_with_cargo(world, pb, config):
    world.container_ids["TEST"] = pb.createMultiBody(
        baseMass=10.0,
        baseCollisionShapeIndex=pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], physicsClientId=world.client_id),
        basePosition=[world.get_robot_state().pose.position[0], world.get_robot_state().pose.position[1], 0.5],
        physicsClientId=world.client_id
    )
    world.attach_container("TEST")
    
    _nav_to(world, "aisle-bc")
    # Small obstacle
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.2, 0.2, 0.5), physicsClientId=world.client_id)
    pb.createMultiBody(0, shape, basePosition=(1.0, 0.25, 0.5), physicsClientId=world.client_id)
    
    _run(world, max_time=40.0)
    events = world._events
            
    assert any("Dynamic obstacle detected" in ev[1] for ev in events)
    assert world.navigator.status == "arrived"

def test_temporary_obstruction(world, pb, config):
    _nav_to(world, "aisle-bc")
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.2, 1.4, 0.5), physicsClientId=world.client_id)
    obs = pb.createMultiBody(0, shape, basePosition=(1.0, 0.25, 0.5), physicsClientId=world.client_id)
    
    # Run partially
    _run(world, max_time=5.0)
    # Remove obstacle
    pb.removeBody(obs, physicsClientId=world.client_id)
    
    # Let it finish
    _run(world, max_time=40.0)
    events = world._events
            
    assert any("Dynamic obstacle detected" in ev[1] for ev in events)
    assert world.navigator.status == "arrived"
    # Ensure it didn't endlessly replan
    replan_count = sum(1 for ev in events if "Replanning" in ev[1])
    assert replan_count < 10

def test_emergency_stop_overrides_replanning(world, pb, config):
    _nav_to(world, "aisle-bc")
    # Add obstacle
    shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(0.2, 1.4, 0.5), physicsClientId=world.client_id)
    pb.createMultiBody(0, shape, basePosition=(1.0, 0.25, 0.5), physicsClientId=world.client_id)
    
    _run(world, max_time=2.0)
    
    # Trigger e-stop
    world.cancel_navigation("emergency stop")
    world.stop(immediate=True)
    
    _run(world, max_time=5.0)
    
    assert world.navigator.status == "cancelled"
