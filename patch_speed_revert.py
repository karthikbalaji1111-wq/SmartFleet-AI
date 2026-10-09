import json

with open("config/warehouse.json", "r") as f:
    config = json.load(f)

config["robot"]["limits"]["max_linear_velocity"] = 1.0
config["robot"]["limits"]["max_linear_acceleration"] = 1.0
config["robot"]["limits"]["max_linear_deceleration"] = 2.0
config["robot"]["wheels"]["max_velocity"] = 20.0
config["navigation"]["controller"]["cruise_speed"] = 0.6

with open("config/warehouse.json", "w") as f:
    json.dump(config, f, indent=2)

print("Reverted speed limits in warehouse.json")
