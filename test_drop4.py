import asyncio
from backend.app.main import create_app
import pybullet as pb
from simulation.robot_model import FORK_LINK, LIFT_CARRIAGE_LINK

async def main():
    app = create_app()
    service = app.state.service
    await service.startup()
    world = service.world

    world.step(10)
    pb.resetBasePositionAndOrientation(world.robot_id, (9.5, 4.0, 0.1), pb.getQuaternionFromEuler((0,0,0)), physicsClientId=world._client)

    world.set_mechanism_target("lift", 0.36)
    world.step(120)
    world.set_mechanism_target("forks", 0.6)
    world.step(120)

    world.attach_container("TOTE-0001")
    world.step(60)

    world.set_mechanism_target("lift", 0.46)
    world.step(60)

    cid = world.container_ids["TOTE-0001"]
    world.detach_container("TOTE-0001")
    
    for _ in range(10):
        world.step(1)

    contacts = pb.getContactPoints(bodyA=world.robot_id, bodyB=cid, physicsClientId=world._client)
    print("Contacts after detach:", len(contacts))
    for c in contacts:
        linkA = c[3]
        linkB = c[4]
        # get link name for robot
        for name, idx in world.model.links.items():
            if idx == linkA:
                print(f"Contact between {name} (link {linkA}) of robot and container")

    # Are there any other contacts with the container?
    all_contacts = pb.getContactPoints(bodyA=cid, physicsClientId=world._client)
    print("All contacts on container:", len(all_contacts))
    for c in all_contacts:
        bB = c[2]
        if bB == world.robot_id:
            pass # already printed
        else:
            print(f"Contact with body {bB}")

    await service.shutdown()

asyncio.run(main())
