import re

with open('tests/test_config.py', 'r') as f:
    content = f.read()

old_code = "(lambda raw: raw[\"robot\"][\"limits\"].update(max_linear_velocity=5.0), \"wheel speed\"),"
new_code = "(lambda raw: [raw[\"robot\"][\"limits\"].update(max_linear_velocity=50000.0), raw[\"navigation\"][\"controller\"].update(cruise_speed=50000.0)], \"wheel speed\"),"

content = content.replace(old_code, new_code)

with open('tests/test_config.py', 'w') as f:
    f.write(content)
