import json
with open('temp_config.json', 'r') as f:
    data = json.load(f)
print(data['config'].keys())
print(data['config']['warehouse'].keys())
for s in data['config']['warehouse'].get('slots', [])[:10]:
    print(s)
