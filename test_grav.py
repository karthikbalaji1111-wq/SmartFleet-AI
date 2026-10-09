import asyncio
from backend.app.main import create_app
import pybullet as pb

async def main():
    app = create_app()
    service = app.state.service
    await service.startup()
    world = service.world

    print("Gravity:", pb.getPhysicsEngineParameters(physicsClientId=world._client))
    
    await service.shutdown()

asyncio.run(main())
