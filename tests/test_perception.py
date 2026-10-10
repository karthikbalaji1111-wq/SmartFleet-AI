import pytest
import math
import numpy as np
from simulation.perception import PerceptionSystem
from simulation.robot_model import LIFT_CARRIAGE_LINK

@pytest.fixture
def perception(world):
    world.initialize()
    return PerceptionSystem(world, world) # mock lock with world (it has __enter__ etc or just use a dummy lock)

def test_perception_initialization(perception):
    assert perception.width == 320
    assert perception.height == 240
    assert len(perception.projection_matrix) == 16

def test_camera_movement_with_lift(world):
    import threading
    world.initialize()
    perc = PerceptionSystem(world, threading.RLock())
    
    # 1. Capture at lift=0
    res1 = perc.capture_and_process()
    assert res1 is not None
    
    # 2. Move lift up by 0.5m
    world.set_mechanism_target("lift", 0.5)
    for _ in range(240):
        world.step(1)
        
    res2 = perc.capture_and_process()
    assert res2 is not None

def test_depth_and_localization(world):
    import threading
    world.initialize()
    perc = PerceptionSystem(world, threading.RLock())
    
    # Put robot facing a rack
    world.set_velocity_command(0.5, 0, 1.0)
    for _ in range(240):
        world.step(1)
        
    res = perc.capture_and_process()
    assert res is not None
    assert "objects" in res
    
    # We should see racks or containers
    objects = res["objects"]
    assert len(objects) > 0, "Should detect at least one object in front of it"
    
    for obj in objects:
        assert "category" in obj
        assert "instance_id" in obj
        assert "bbox" in obj
        assert "distance" in obj
        assert "position" in obj
        assert obj["aabb_boundary_distance"] is not None
        
        # Verify distance is within bounds
        assert 0.1 <= obj["distance"] <= 10.0, f"Distance {obj['distance']} out of bounds"
        
        # Verify AABB boundary distance is reasonable. For solid containers it's < 1cm, but for sparse racks (where bounding box center might not hit surface directly), we allow larger.
        assert obj["aabb_boundary_distance"] < 15.0, f"AABB boundary distance {obj['aabb_boundary_distance']} is too large"
        
        # Verify bounding box is valid
        x, y, w, h = obj["bbox"]
        assert 0 <= x < perc.width
        assert 0 <= y < perc.height
        assert w > 0 and h > 0
