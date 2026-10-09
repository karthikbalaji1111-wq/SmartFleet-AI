with open('simulation/world.py', 'r') as f:
    content = f.read()

content = content.replace("        # Lift and forks start at their default positions\n (placed once while the", 
                          "        # Lift and forks start at their default positions (placed once while the")

with open('simulation/world.py', 'w') as f:
    f.write(content)
