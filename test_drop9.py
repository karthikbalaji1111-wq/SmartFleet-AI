import asyncio
from backend.app.main import create_app
import pybullet as pb

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
    
    print("DETACHED!")
    for i in range(30):
        world.step(1)
        contacts = pb.getContactPoints(bodyA=cid, physicsClientId=world._client)
        if len(contacts) > 0:
            for c in contacts:
                bB = c[2]
                if bB == world.robot_id:
                    link_idx = c[4]
                    print(f"Step {i}: Contact with robot link {link_idx}")
                else:
                    print(f"Step {i}: Contact with body {bB}")

    await service.shutdown()

asyncio.run(main())
