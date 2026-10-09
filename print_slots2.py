import json
with open('temp_config2.json', 'r') as f:
    data = json.load(f)
for s in data['slots'][:10]:
    print(s)
