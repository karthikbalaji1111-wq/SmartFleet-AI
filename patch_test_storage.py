import re

with open("tests/test_storage.py", "r") as f:
    content = f.read()

# Replace hardcoded assert
old_code = """    # S2 should be at center of bay (local_x = 0 from center)
    # A1 center is [-7, 5.5], A1-B1-L1-S2 is [-8.18, 5.5]
    assert math.isclose(pos2[0], -8.18, abs_tol=0.1)
    
    # Check that TOTE-0002 didn't move
    pos1 = get_container_pos(world, "TOTE-0002")
    assert math.isclose(pos1[0], -8.967, abs_tol=0.1)"""

new_code = """    # Fetch the actual slots to compare against
    s2_slot = next(s for s in world.config.warehouse.slots if s.id == "A1-B1-L1-S2")
    s1_slot = next(s for s in world.config.warehouse.slots if s.id == "A1-B1-L1-S1")
    
    assert math.isclose(pos2[0], s2_slot.center[0], abs_tol=0.1)
    
    # Check that TOTE-0002 didn't move
    pos1 = get_container_pos(world, "TOTE-0002")
    assert math.isclose(pos1[0], s1_slot.center[0], abs_tol=0.1)"""

content = content.replace(old_code, new_code)
with open("tests/test_storage.py", "w") as f:
    f.write(content)

print("Patched test_storage.py")
