import re

with open('simulation/world.py', 'r') as f:
    content = f.read()

content = content.replace("self.container_ids: dict[str, int] = {}", "self.container_ids: dict[str, int] = {}\n        self.container_locations: dict[str, str] = {}")

code_to_replace = """        self.container_ids = {}
        for cont in self.config.warehouse.containers:
            cx, cy, cz = container_initial_position(self.config.warehouse, cont)"""

new_code = """        self.container_ids = {}
        self.container_locations = {}
        for cont in self.config.warehouse.containers:
            self.container_locations[cont.id] = cont.location
            cx, cy, cz = container_initial_position(self.config.warehouse, cont)"""
content = content.replace(code_to_replace, new_code)

with open('simulation/world.py', 'w') as f:
    f.write(content)
