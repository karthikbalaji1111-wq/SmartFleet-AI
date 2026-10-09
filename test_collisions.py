import asyncio
from backend.app.main import create_app
import pybullet as pb
from simulation.robot_model import FORK_LINK, LIFT_CARRIAGE_LINK

async def main():
    app = create_app()
    service = app.state.service
    await service.startup()
    world = service.world

    cid = world.container_ids["TOTE-0001"]
    
    # Check contact points
    contacts = pb.getContactPoints(bodyA=world.robot_id, bodyB=cid, physicsClientId=world._client)
    print("Contacts:", len(contacts))
    for c in contacts:
        linkA = c[3]
        linkB = c[4]
        print(f"Contact between link {linkA} of robot and link {linkB} of container")

    await service.shutdown()

asyncio.run(main())
