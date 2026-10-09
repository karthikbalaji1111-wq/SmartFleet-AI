import asyncio
from backend.app.main import create_app
from backend.app.schemas import TelemetryMessage
import backend.app.service as service

async def test():
    service.init()
    snapshot = service.snapshot()
    try:
        msg = TelemetryMessage(snapshot=snapshot, events=[], route_update=None)
        json_data = msg.model_dump_json()
        print("Success, length:", len(json_data))
    except Exception as e:
        import traceback
        traceback.print_exc()

asyncio.run(test())
