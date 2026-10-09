import re

with open('simulation/config.py', 'r') as f:
    content = f.read()

old_code = """            for bay in range(rt.bays):
                for lvl_idx, z in enumerate(rt.levels):
                    slot_id = f"{rack.id}-B{bay+1}-L{lvl_idx+1}"
                    
                    # Local x offset for the bay
                    local_x = start_offset + bay * bay_width
                    
                    if rack.axis == "x":
                        cx = rack.center[0] + local_x
                        cy = rack.center[1]
                    else:
                        cx = rack.center[0]
                        cy = rack.center[1] + local_x
                        
                    slots.append(StorageSlot(
                        id=slot_id,
                        rack_id=rack.id,
                        bay=bay+1,
                        level=lvl_idx+1,
                        center=(cx, cy, z)
                    ))"""

new_code = """            for bay in range(rt.bays):
                slots_per_bay = max(1, int(bay_width / 0.6))
                sub_slot_width = bay_width / slots_per_bay
                sub_start_offset = -bay_width / 2 + sub_slot_width / 2
                
                for lvl_idx, z in enumerate(rt.levels):
                    for s_idx in range(slots_per_bay):
                        slot_id = f"{rack.id}-B{bay+1}-L{lvl_idx+1}-S{s_idx+1}"
                        
                        # Local x offset for the bay
                        bay_center_x = start_offset + bay * bay_width
                        local_x = bay_center_x + sub_start_offset + s_idx * sub_slot_width
                        
                        if rack.axis == "x":
                            cx = rack.center[0] + local_x
                            cy = rack.center[1]
                        else:
                            cx = rack.center[0]
                            cy = rack.center[1] + local_x
                            
                        slots.append(StorageSlot(
                            id=slot_id,
                            rack_id=rack.id,
                            bay=bay+1,
                            level=lvl_idx+1,
                            center=(cx, cy, z)
                        ))"""

content = content.replace(old_code, new_code)
with open('simulation/config.py', 'w') as f:
    f.write(content)
