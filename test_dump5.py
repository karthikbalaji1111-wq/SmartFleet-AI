from backend.app.service import SimulationService
from backend.app.schemas import TelemetryMessage
import traceback

def test():
    service = SimulationService()
    snapshot = service.snapshot()
    try:
        msg = TelemetryMessage(snapshot=snapshot, events=[], route_update=None)
        msg.model_dump_json()
        print("Success")
    except Exception as e:
        traceback.print_exc()

test()
