import re

with open('tests/test_storage.py', 'r') as f:
    content = f.read()

content = content.replace('print(f"Task failed: {tm.error}")\n            return False\n    return False', 'print(f"Task failed: {tm.error}")\n            return False\n    print(f"Task timed out in state {tm.state}")\n    return False')

with open('tests/test_storage.py', 'w') as f:
    f.write(content)
