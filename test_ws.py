import asyncio
import websockets
async def test():
    async with websockets.connect("ws://127.0.0.1:8000/ws/telemetry") as ws:
        await ws.recv()
asyncio.run(test())
