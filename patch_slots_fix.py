import re

with open("simulation/config.py", "r") as f:
    content = f.read()

# I need to change: slots_per_bay = max(1, int(bay_width / 0.6))
# To use container width + clearance.
# The container width is self.containers[0].size[0]
old_code = """            for bay in range(rt.bays):
                slots_per_bay = max(1, int(bay_width / 0.6))
                sub_slot_width = bay_width / slots_per_bay
                sub_start_offset = -bay_width / 2 + sub_slot_width / 2"""

new_code = """            # Get max container width
            max_cont_width = max([c.size[0] for c in self.containers]) if self.containers else 0.4
            clearance = 0.05
            slot_needed = max_cont_width + clearance
            for bay in range(rt.bays):
                slots_per_bay = max(1, int(bay_width / slot_needed))
                sub_slot_width = bay_width / slots_per_bay
                sub_start_offset = -bay_width / 2 + sub_slot_width / 2"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open("simulation/config.py", "w") as f:
        f.write(content)
    print("Replaced slot logic in config.py")
else:
    print("Could not find old_code")
