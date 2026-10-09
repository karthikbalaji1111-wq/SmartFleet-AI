import re

with open('simulation/world.py', 'r') as f:
    content = f.read()

# Revert previous patch
content = re.sub(
    r"        # Disable collisions between forks and static environment to allow picking from flat shelves\n.*?(?=        # Lift and forks start at their default positions)",
    "",
    content,
    flags=re.DOTALL
)

# Add patch to disable collisions between robot (forks) and containers
new_code = """        # Disable collisions between the forks and containers so the forks can slide under/through them
        for cid in self.container_ids.values():
            pb.setCollisionFilterPair(body, cid, links[FORK_LINK], -1, 0, physicsClientId=c)

        # Lift and forks start at their default positions (placed once while the"""
content = content.replace("        # Lift and forks start at their default positions (placed once while the", new_code)

with open('simulation/world.py', 'w') as f:
    f.write(content)
