import re

with open('simulation/world.py', 'r') as f:
    content = f.read()

# Remove the bad patch from _configure_robot
content = re.sub(
    r"        # Disable collisions between the forks and containers so the forks can slide under/through them\n        for cid in self\.container_ids\.values\(\):\n            pb\.setCollisionFilterPair\(body, cid, links\[FORK_LINK\], -1, 0, physicsClientId=c\)\n",
    "",
    content
)

# Add the good patch inside _build_world, AFTER containers are created
new_code = """        for _ in range(sim.settle_steps):"""
replacement = """
        # Disable collisions between the forks and containers so the forks can slide under/through them
        for cid in self.container_ids.values():
            pb.setCollisionFilterPair(self.robot_id, cid, self.model.links[FORK_LINK], -1, 0, physicsClientId=c)

        for _ in range(sim.settle_steps):"""
content = content.replace(new_code, replacement)

with open('simulation/world.py', 'w') as f:
    f.write(content)

