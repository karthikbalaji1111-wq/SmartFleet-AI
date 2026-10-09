import asyncio
from backend.app.main import create_app
from fastapi.testclient import TestClient

app = create_app()
with TestClient(app) as client:
    with client.websocket_connect("/ws/telemetry") as ws:
        msg = ws.receive_text()
        print("Received:", msg[:100])
