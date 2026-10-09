with open('simulation/tasks.py', 'r') as f:
    content = f.read()

# Replace src_z - 0.05 with src_z + 0.05
content = content.replace('self._calc_lift_target(self._src_z - 0.05)', 'self._calc_lift_target(self._src_z + 0.05)')
# Replace src_z + 0.05 with src_z + 0.15 (since we need to lift higher to clear the rack)
content = content.replace('self._calc_lift_target(self._src_z + 0.05)', 'self._calc_lift_target(self._src_z + 0.15)')

# Wait, the second replacement will ALSO replace the first one if I'm not careful!
# Let's do it safely.
