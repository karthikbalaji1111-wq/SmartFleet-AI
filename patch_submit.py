import re

with open('simulation/tasks.py', 'r') as f:
    content = f.read()

# Add occupancy check
old_code = """        dest_loc = self._find_location(request.destination)
        if not dest_loc:
            raise ValueError(f"Unknown destination {request.destination}")
            
        self.task = request"""

new_code = """        dest_loc = self._find_location(request.destination)
        if not dest_loc:
            raise ValueError(f"Unknown destination {request.destination}")
            
        # Check occupancy
        for cid, loc_id in getattr(self.world, 'container_locations', {}).items():
            if cid != request.container_id and loc_id == request.destination:
                raise ValueError(f"Destination {request.destination} is already occupied by {cid}")

        self.task = request"""

content = content.replace(old_code, new_code)
with open('simulation/tasks.py', 'w') as f:
    f.write(content)
