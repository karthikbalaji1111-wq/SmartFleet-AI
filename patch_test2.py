import re

with open('tests/test_storage.py', 'r') as f:
    content = f.read()

content = content.replace('print(f"Task timed out in state {tm.state}")', 'print(f"Task timed out in state {tm.state}. Navigator status: {world.navigator.status}, pose: {world.get_robot_state().pose}")')

with open('tests/test_storage.py', 'w') as f:
    f.write(content)
