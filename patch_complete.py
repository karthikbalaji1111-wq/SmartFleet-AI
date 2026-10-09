import re

with open('simulation/tasks.py', 'r') as f:
    content = f.read()

old_code = """            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0) # Lower lift to travel
                self.state = "completed"
                self.task = None"""

new_code = """            if self._back_away_timer <= 0:
                self.world.set_velocity_command(0.0, 0.0)
                self.world.set_mechanism_target("lift", 0.0) # Lower lift to travel
                # Update logical occupancy
                if hasattr(self.world, 'container_locations') and self.task:
                    self.world.container_locations[self.task.container_id] = self.task.destination
                self.state = "completed"
                self.task = None"""

content = content.replace(old_code, new_code)
with open('simulation/tasks.py', 'w') as f:
    f.write(content)
