import pytest
from backend.app.schemas import TaskRequest
from simulation.tasks import TaskManager
import pybullet as pb
import math

def run_task_to_completion(world, tm, request: TaskRequest, timeout_steps=5000):
    tm.submit(request)
    for _ in range(timeout_steps):
        world.step(1)
        tm.tick(world.timestep)
        if tm.state == "completed":
            return True
        if tm.state == "failed":
            print(f"Task failed: {tm.error}")
            return False
    print(f"Task timed out in state {tm.state}. Navigator status: {world.navigator.status}, pose: {world.get_robot_state().pose}")
    return False

@pytest.fixture
def test_world(world):
    # 
    world.step(100)
    return world

def get_container_z(world, cid: str):
    c_id = world.container_ids[cid]
    pos, _ = pb.getBasePositionAndOrientation(c_id, physicsClientId=world._client)
    return pos[2]

def get_container_pos(world, cid: str):
    c_id = world.container_ids[cid]
    pos, _ = pb.getBasePositionAndOrientation(c_id, physicsClientId=world._client)
    return pos

def test_multiple_storage_slots(test_world):
    world = test_world
    tm = TaskManager(world)
    
    # 4. Two containers cannot occupy the same slot (Logical check)
    tote1_loc = world.container_locations["TOTE-0002"]
    with pytest.raises(ValueError, match="already occupied"):
        tm.submit(TaskRequest(container_id="TOTE-0001", destination=tote1_loc))
        
    # We will pick TOTE-0001 from inbound and place it in A1-B1-L1-S2
    # Let's say TOTE-0002 is already at A1-B1-L1-S1
    assert tote1_loc == "A1-B1-L1-S1"
    
    req1 = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    success = run_task_to_completion(world, tm, req1, timeout_steps=30000)
    assert success, "Task to place TOTE-0001 in S2 failed"
    
    # Check that TOTE-0001 is at S2
    pos2 = get_container_pos(world, "TOTE-0001")
    # Fetch the actual slots to compare against
    s2_slot = next(s for s in world.config.warehouse.slots if s.id == "A1-B1-L1-S2")
    s1_slot = next(s for s in world.config.warehouse.slots if s.id == "A1-B1-L1-S1")
    
    assert math.isclose(pos2[0], s2_slot.center[0], abs_tol=0.1)
    
    # Check that TOTE-0002 didn't move
    pos1 = get_container_pos(world, "TOTE-0002")
    assert math.isclose(pos1[0], s1_slot.center[0], abs_tol=0.1)
    assert math.isclose(pos1[2], pos2[2], abs_tol=0.05) # Same level
    
    # 3. The second placement does not change the first box's settled position.
    assert pos1[2] > 0.35, "First box was pushed down!"

def test_adjacent_levels_no_intersect(test_world):
    world = test_world
    tm = TaskManager(world)
    
    # Let's place TOTE-0001 in A1-B1-L2-S1 (above TOTE-0002)
    req1 = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1")
    success = run_task_to_completion(world, tm, req1, timeout_steps=30000)
    assert success, "Task failed"
    
    pos2 = get_container_pos(world, "TOTE-0001")
    pos1 = get_container_pos(world, "TOTE-0002")
    
    # They should have different Z
    assert pos2[2] > pos1[2] + 0.2, "Level 2 should be above Level 1"
    
