import re

with open('simulation/world.py', 'r') as f:
    content = f.read()

replacement = """        # Disable collisions between the forks and containers so the forks can slide under/through them
        for cid in self.container_ids.values():
            pb.setCollisionFilterPair(self.robot_id, cid, self.model.links[FORK_LINK], -1, 0, physicsClientId=c)
            pb.setCollisionFilterPair(self.robot_id, cid, self.model.links[LIFT_CARRIAGE_LINK], -1, 0, physicsClientId=c)"""

content = re.sub(
    r"        # Disable collisions between the forks and containers so the forks can slide under/through them\n        for cid in self\.container_ids\.values\(\):\n            pb\.setCollisionFilterPair\(self\.robot_id, cid, self\.model\.links\[FORK_LINK\], -1, 0, physicsClientId=c\)",
    replacement,
    content
)

with open('simulation/world.py', 'w') as f:
    f.write(content)
