from simulation.config import get_config
from simulation.robot_model import fork_surface_height

config = get_config()
h = fork_surface_height(config.robot, 0.65)
print(h)
