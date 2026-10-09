import asyncio
from backend.app.schemas import TaskRequest
from simulation.world import SimulationWorld
from simulation.config import get_config
from simulation.tasks import TaskManager
import math
import pybullet as pb

config = get_config()
world = SimulationWorld(config, headless=True)

tm = TaskManager(world)

c_state = next(c for c in world.get_container_states() if c.id == "TOTE-0001")
print("Container TOTE-0001 start pos:", c_state.position)

all_locs = list(config.warehouse.stations) + list(config.warehouse.slots)
src_loc = min(all_locs, key=lambda s: math.hypot(s.center[0] - c_state.position[0], s.center[1] - c_state.position[1]))
print("Source loc identified:", src_loc.id, "at", src_loc.center)

ax, ay, ayaw = tm._get_approach_pose(src_loc)
print("Approach pose:", ax, ay, ayaw)
dest = world.navigator.resolve_destination(None, ax, ay, yaw=ayaw, snap=0.5)
print("Resolved destination:", dest)

print("\nRunning task to place TOTE-0001 in A1-B1-L1-S2")
req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
tm.submit(req)

for i in range(20000):
    world.step(1)
    tm.tick(world.timestep)
    if i % 1000 == 0:
        print(f"Step {i}: state={tm.state}, nav_status={world.navigator.status}, pose={world.get_robot_state().pose}")
    if tm.state in ("completed", "failed"):
        print(f"Finished at step {i} with state {tm.state}, error={tm.error}")
        break
