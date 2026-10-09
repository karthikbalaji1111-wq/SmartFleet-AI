import pytest
from backend.app.main import create_app
from fastapi.testclient import TestClient

@pytest.fixture
def client():
    app = create_app()
    with TestClient(app) as c:
        yield c, app

def test_submit_task(client):
    c, app = client
    # Let's see if world exists
    world = app.state.service.world
    print("Container IDs:", world.container_ids)
    
    # Just a simple smoke test
    res = c.post("/api/tasks/submit", json={"container_id": "TOTE-0001", "destination": "A1-B1-L1-S2"})
    print(res.json())
    assert res.status_code == 200
    assert "Task submitted" in res.json()["message"]
    
    # Cancel it
    res = c.post("/api/tasks/cancel")
    assert res.status_code == 200
