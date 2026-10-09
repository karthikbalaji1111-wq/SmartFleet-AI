import json

with open('config/warehouse.json', 'r') as f:
    config = json.load(f)

# Update limits
config['robot']['limits']['max_linear_velocity'] = 40.0
config['robot']['limits']['max_linear_acceleration'] = 40.0
config['robot']['limits']['max_linear_deceleration'] = 40.0

# Update wheel max velocity to support 40m/s (40 / radius 0.1 = 400 rad/s + turn overhead)
config['robot']['wheels']['max_velocity'] = 500.0

# Update cruise speed
config['navigation']['controller']['cruise_speed'] = 40.0

with open('config/warehouse.json', 'w') as f:
    json.dump(config, f, indent=2)
