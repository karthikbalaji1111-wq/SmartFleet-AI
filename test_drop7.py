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
    
    for i in range(10):
        world.step(1)
        vel = pb.getBaseVelocity(cid, physicsClientId=world._client)
        print(f"Step {i} Vel Z: {vel[0][2]}")

    await service.shutdown()

asyncio.run(main())
