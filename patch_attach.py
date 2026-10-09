import re

with open('simulation/world.py', 'r') as f:
    content = f.read()

content = content.replace(
    "ideal_z = c_config.size[2] / 2 + self.config.robot.forks.tine_thickness / 2",
    "ideal_z = c_config.size[2] / 2 + self.config.robot.forks.tine_thickness"
)

with open('simulation/world.py', 'w') as f:
    f.write(content)
