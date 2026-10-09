from simulation.config import get_config

config = get_config()
print(f"Containers: {config.warehouse.containers[0].size}")
for s in config.warehouse.slots:
    if "A1-B1-L1" in s.id:
        print(s)
