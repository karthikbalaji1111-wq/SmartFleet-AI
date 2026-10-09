import asyncio
from backend.app.main import create_app
import pybullet as pb

async def main():
    app = create_app()
    service = app.state.service
    await service.startup()
    world = service.world

    cid = world.container_ids["TOTE-0001"]
    dyn = pb.getDynamicsInfo(cid, -1, physicsClientId=world._client)
    print("Mass:", dyn[0])
    
    await service.shutdown()

asyncio.run(main())
