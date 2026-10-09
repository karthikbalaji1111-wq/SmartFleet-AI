import re

with open('simulation/tasks.py', 'r') as f:
    content = f.read()

# Add import
if 'from .robot_model import fork_surface_height' not in content:
    content = content.replace('from .state import RobotState, BodyState', 
                              'from .state import RobotState, BodyState\nfrom .robot_model import fork_surface_height')

# Add _calc_lift_target method to TaskManager
if 'def _calc_lift_target' not in content:
    replacement = """
    def _calc_lift_target(self, desired_surface_z: float) -> float:
        baseline = fork_surface_height(self.world.config.robot, 0.0)
        return max(0.0, desired_surface_z - baseline)

    def submit("""
    content = content.replace('    def submit(', replacement)

# Replace all occurrences where we set lift target using _src_z or _dest_z directly
content = content.replace(
    'self.world.set_mechanism_target("lift", self._src_z - 0.05)',
    'self.world.set_mechanism_target("lift", self._calc_lift_target(self._src_z - 0.05))'
)
content = content.replace(
    'self.world.set_mechanism_target("lift", self._src_z + 0.05)',
    'self.world.set_mechanism_target("lift", self._calc_lift_target(self._src_z + 0.05))'
)
content = content.replace(
    'self.world.set_mechanism_target("lift", self._dest_z - 0.05)',
    'self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z - 0.05))'
)
content = content.replace(
    'self.world.set_mechanism_target("lift", self._dest_z + 0.05)',
    'self.world.set_mechanism_target("lift", self._calc_lift_target(self._dest_z + 0.05))'
)

with open('simulation/tasks.py', 'w') as f:
    f.write(content)
