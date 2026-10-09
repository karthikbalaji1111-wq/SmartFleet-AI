"""Backend health, configuration, simulation-state and control endpoints."""

from __future__ import annotations

import math
import time

import pytest
from fastapi.testclient import TestClient

from backend.app.main import create_app
from simulation.geometry import build_static_geometry
from simulation.robot_model import EXPECTED_JOINTS
from simulation.world import pybullet_available

from .conftest import PYBULLET_SKIP_REASON

needs_physics = pytest.mark.skipif(not pybullet_available(), reason=PYBULLET_SKIP_REASON)


@pytest.fixture
def client():
    with TestClient(create_app()) as c:
        yield c


def _pose(client):
    robot = client.get("/api/simulation/state").json()["world"]["robot"]
    return robot["pose"]["position"], robot["pose"]["heading"]


# --------------------------------------------------------------------------- #
# Always available (no physics required)
# --------------------------------------------------------------------------- #
def test_health(client):
    r = client.get("/api/health")
    assert r.status_code == 200
    body = r.json()
    assert body["status"] == "ok" and body["service"] == "smartfleet-backend"


def test_ready_reflects_physics_availability(client):
    r = client.get("/api/ready")
    body = r.json()
    assert body["pybullet_available"] is pybullet_available()
    if pybullet_available():
        assert r.status_code == 200 and body["ready"] is True and body["world_initialized"] is True
    else:
        assert r.status_code == 503 and body["ready"] is False and "PyBullet" in body["detail"]


def test_warehouse_config_serves_canonical_geometry(client, config):
    r = client.get("/api/config/warehouse")
    assert r.status_code == 200
    body = r.json()
    assert body["config"]["warehouse"]["name"] == config.warehouse.name
    served = {b["id"]: b for b in body["static_geometry"]}
    expected = build_static_geometry(config.warehouse)
    assert len(served) == len(expected)
    for box in expected:
        assert served[box.id]["center"] == pytest.approx(list(box.center))
        assert served[box.id]["size"] == pytest.approx(list(box.size))
        assert served[box.id]["kind"] == box.kind
    joints = {j["name"]: j for j in body["robot_model"]["joints"]}
    assert set(joints) == set(EXPECTED_JOINTS)
    assert joints["lift_joint"]["upper"] == config.robot.lift.upper
    assert body["config"]["robot"]["limits"]["max_linear_velocity"] == config.robot.limits.max_linear_velocity


def test_openapi_documents_endpoints(client):
    paths = client.get("/openapi.json").json()["paths"]
    for path in (
        "/api/health",
        "/api/ready",
        "/api/config/warehouse",
        "/api/simulation/state",
        "/api/simulation/start",
        "/api/simulation/pause",
        "/api/simulation/reset",
        "/api/robot/velocity",
        "/api/robot/stop",
    ):
        assert path in paths


@pytest.mark.skipif(pybullet_available(), reason="checks the no-physics fallback")
def test_simulation_endpoints_return_503_without_physics(client):
    assert client.post("/api/simulation/start").status_code == 503
    assert client.get("/api/simulation/state").json()["ready"] is False


# --------------------------------------------------------------------------- #
# Physics-backed endpoints
# --------------------------------------------------------------------------- #
@needs_physics
def test_initial_state_is_stopped_at_start_pose(client, config):
    r = client.get("/api/simulation/state")
    assert r.status_code == 200
    snap = r.json()
    assert snap["status"] == "stopped" and snap["ready"] is True
    assert snap["physics_hz"] == config.simulation.physics_hz
    pos = snap["world"]["robot"]["pose"]["position"]
    assert pos[0] == pytest.approx(config.robot.start_pose.x, abs=1e-3)
    assert pos[1] == pytest.approx(config.robot.start_pose.y, abs=1e-3)
    assert snap["world"]["robot"]["command"]["active"] is False


@needs_physics
def test_commands_rejected_unless_running(client):
    r = client.post("/api/robot/velocity", json={"linear": 0.5, "angular": 0.0})
    assert r.status_code == 409
    assert client.post("/api/simulation/pause").status_code == 409  # cannot pause when stopped


@needs_physics
@pytest.mark.parametrize(
    "payload",
    [
        {"linear": 3.0, "angular": 0.0},
        {"linear": 0.0, "angular": 9.0},
        {"linear": "fast", "angular": 0.0},
        {"linear": 0.5},
        {"linear": 0.5, "angular": 0.0, "extra": 1},
        {"linear": 0.5, "angular": 0.0, "duration": 60},
        [0.5, 0.0],
    ],
)
def test_malformed_commands_rejected_with_422(client, payload):
    client.post("/api/simulation/start")
    r = client.post("/api/robot/velocity", json=payload)
    assert r.status_code == 422
    state = client.get("/api/simulation/state").json()
    assert state["world"]["robot"]["command"]["active"] is False


@needs_physics
def test_nan_command_rejected(client):
    client.post("/api/simulation/start")
    r = client.post(
        "/api/robot/velocity", content='{"linear": NaN, "angular": 0.0}', headers={"content-type": "application/json"}
    )
    assert r.status_code == 422


@needs_physics
def test_start_drive_pause_reset_cycle_moves_the_physical_robot(client, config):
    start_pos, start_heading = _pose(client)
    r = client.post("/api/simulation/start")
    assert r.status_code == 200 and r.json()["snapshot"]["status"] == "running"

    # Hold a forward request for ~1.5 s of wall time, like the UI does.
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        cmd = client.post("/api/robot/velocity", json={"linear": 0.6, "angular": 0.0, "duration": 0.5})
        assert cmd.status_code == 200
        body = cmd.json()
        assert body["accepted"] is True
        assert body["wheel_targets"]["left"] == pytest.approx(0.6 / config.robot.wheels.radius)
        time.sleep(0.15)

    pos, heading = _pose(client)
    moved = math.dist(start_pos[:2], pos[:2])
    assert moved > 0.4, f"robot only moved {moved:.3f} m in physics"
    # Moved along its heading (start yaw = pi -> towards -X).
    assert (pos[0] - start_pos[0]) * math.cos(start_heading) > 0.4

    r = client.post("/api/simulation/pause")
    assert r.status_code == 200 and r.json()["snapshot"]["status"] == "paused"
    t1 = client.get("/api/simulation/state").json()["world"]["sim_time"]
    time.sleep(0.2)
    t2 = client.get("/api/simulation/state").json()["world"]["sim_time"]
    assert t1 == t2, "physics must not advance while paused"
    assert client.post("/api/robot/velocity", json={"linear": 0.5, "angular": 0.0}).status_code == 409

    r = client.post("/api/simulation/reset")
    assert r.status_code == 200
    snap = r.json()["snapshot"]
    assert snap["status"] == "stopped"
    assert snap["world"]["sim_time"] == 0.0
    reset_pos = snap["world"]["robot"]["pose"]["position"]
    assert reset_pos[0] == pytest.approx(config.robot.start_pose.x, abs=1e-3)
    assert reset_pos[1] == pytest.approx(config.robot.start_pose.y, abs=1e-3)


@needs_physics
def test_turn_command_rotates_the_physical_robot(client):
    _, heading0 = _pose(client)
    client.post("/api/simulation/start")
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        assert client.post("/api/robot/velocity", json={"linear": 0.0, "angular": 1.0}).status_code == 200
        time.sleep(0.15)
    _, heading1 = _pose(client)
    turned = (heading1 - heading0 + math.pi) % (2 * math.pi) - math.pi
    assert turned > 0.5, f"robot turned only {turned:.3f} rad"


@needs_physics
def test_stop_endpoint(client):
    client.post("/api/simulation/start")
    client.post("/api/robot/velocity", json={"linear": 0.5, "angular": 0.0, "duration": 2.0})
    r = client.post("/api/robot/stop")
    assert r.status_code == 200
    assert r.json()["snapshot"]["world"]["robot"]["command"]["active"] is False


@needs_physics
def test_websocket_streams_telemetry_and_events(client):
    with client.websocket_connect("/ws/telemetry") as ws:
        first = ws.receive_json()
        assert first["type"] == "telemetry"
        robot = first["snapshot"]["world"]["robot"]
        assert len(robot["pose"]["position"]) == 3 and "heading" in robot["pose"]
        assert {"lift", "forks", "wheels", "velocity", "command"} <= set(robot)
        assert any("Physics ready" in e["message"] for e in first["events"])
        second = ws.receive_json()
        assert second["snapshot"]["status"] == "stopped"
        assert all(e["id"] > first["events"][-1]["id"] for e in second["events"])
