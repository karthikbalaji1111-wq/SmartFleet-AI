from backend.app.main import create_app
import backend.app.service as service
from backend.app.schemas import TelemetryMessage

app = create_app()
try:
    snapshot = service.snapshot()
    msg = TelemetryMessage(snapshot=snapshot, events=[], route_update=None)
    print("Dumping...")
    msg.model_dump_json()
    print("Done!")
except Exception as e:
    import traceback
    traceback.print_exc()
