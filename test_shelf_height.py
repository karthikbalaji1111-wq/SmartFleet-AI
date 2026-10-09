import asyncio
from backend.app.main import create_app
import pybullet as pb

async def main():
    app = create_app()
    service = app.state.service
    await service.startup()
    world = service.world

    cid = world.container_ids["TOTE-0001"]
    pos, _ = pb.getBasePositionAndOrientation(cid, physicsClientId=world._client)
    print("Initial container Z:", pos[2])
    
    # Let's drop it to see where it lands on its own?
    await service.shutdown()

asyncio.run(main())
