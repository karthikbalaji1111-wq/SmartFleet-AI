import re

with open("tests/test_config.py", "r") as f:
    content = f.read()

old_str = '(lambda raw: [raw["robot"]["limits"].update(max_linear_velocity=50000.0), raw["navigation"]["controller"].update(cruise_speed=50000.0)], "wheel speed")'
new_str = '(lambda raw: raw["robot"]["limits"].update(max_linear_velocity=5.0), "wheel speed")'

content = content.replace(old_str, new_str)

with open("tests/test_config.py", "w") as f:
    f.write(content)

print("Reverted test config")
