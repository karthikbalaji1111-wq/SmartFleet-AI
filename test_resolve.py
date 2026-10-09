import math
from simulation.config import get_config
from simulation.world import SimulationWorld
from simulation.tasks import TaskManager
import pybullet as pb

config = get_config()
world = SimulationWorld(config)
tm = TaskManager(world)

for slot_id in ["A1-B1-L1-S1", "A1-B1-L1-S2", "A1-B1-L1-S3"]:
    loc = tm._find_location(slot_id)
    ax, ay, ayaw = tm._get_approach_pose(loc)
    dest = world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=True)
    print(f"{slot_id} -> loc.center={loc.center} | approach={ax, ay, ayaw} | dest={dest.x, dest.y, dest.yaw} | snapped={dest.snapped}")
