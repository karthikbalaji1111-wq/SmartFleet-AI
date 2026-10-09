"""Lift/fork REST endpoints and WebSocket telemetry against the real physics loop."""

from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from backend.app.main import create_app
from simulation.world import pybullet_available

from .conftest import PYBULLET_SKIP_REASON

pytestmark = pytest.mark.skipif(not pybullet_available(), reason=PYBULLET_SKIP_REASON)


@pytest.fixture
def client():
    with TestClient(create_app()) as c:
        yield c


def _robot(client):
    return client.get("/api/simulation/state").json()["world"]["robot"]


def _wait_for(client, predicate, timeout=6.0):
    """Poll real (wall-clock) simulation state until ``predicate(robot)`` holds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        robot = _robot(client)
        if predicate(robot):
            return robot
        time.sleep(0.05)
    raise AssertionError(f"condition not reached within {timeout} s; last state: {robot['lift']} {robot['forks']}")


def test_config_serves_lift_presets(client, config):
    presets = client.get("/api/config/warehouse").json()["lift_presets"]
    ids = [p["id"] for p in presets]
    assert ids[0] == "home" and "shelf-1" in ids and "stand-1" in ids
    for p in presets:
        assert config.robot.lift.lower <= p["position"] <= config.robot.lift.upper


def test_mechanism_commands_require_running_simulation(client):
    assert client.post("/api/robot/lift/target", json={"position": 0.5}).status_code == 409
    assert client.post("/api/robot/forks/jog", json={"delta": 0.1}).status_code == 409
    # Stopping is always allowed (safe in any state).
    r = client.post("/api/robot/mechanisms/stop")
    assert r.status_code == 200 and len(r.json()["stopped"]) == 2


@pytest.mark.parametrize(
    ("path", "payload", "hint"),
    [
        ("/api/robot/lift/target", {"position": 1.7}, "less than or equal to 1.6"),
        ("/api/robot/lift/target", {"position": -0.1}, "greater than or equal to 0"),
        ("/api/robot/forks/target", {"position": 0.65}, "less than or equal to 0.6"),
        ("/api/robot/forks/target", {"position": "0.3"}, "valid number"),
        ("/api/robot/forks/target", {"position": 0.3, "speed": 9}, "Extra inputs"),
        ("/api/robot/lift/target", {}, "Field required"),
        ("/api/robot/lift/jog", {"delta": 0.5}, "less than or equal to 0.25"),
        ("/api/robot/forks/jog", {"delta": 0.0}, "non-zero"),
    ],
)
def test_invalid_mechanism_commands_rejected(client, path, payload, hint):
    client.post("/api/simulation/start")
    r = client.post(path, json=payload)
    assert r.status_code == 422
    assert hint in str(r.json()["detail"])
    robot = _robot(client)
    assert robot["lift"]["target"] == 0.0 and robot["forks"]["target"] == 0.0


def test_nan_target_rejected(client):
    client.post("/api/simulation/start")
    r = client.post(
        "/api/robot/lift/target", content='{"position": NaN}', headers={"content-type": "application/json"}
    )
    assert r.status_code == 422


def test_lift_command_moves_physical_joint(client, config):
    client.post("/api/simulation/start")
    r = client.post("/api/robot/lift/target", json={"position": 0.6})
    assert r.status_code == 200
    body = r.json()
    assert body["accepted"] and body["mechanism"] == "lift" and body["target"] == 0.6
    assert "raising" in body["message"]

    moving = _wait_for(client, lambda rb: 0.1 < rb["lift"]["position"] < 0.5)
    assert moving["lift"]["state"] == "moving" and moving["lift"]["velocity"] > 0.2
    done = _wait_for(client, lambda rb: rb["lift"]["at_target"])
    assert done["lift"]["position"] == pytest.approx(0.6, abs=config.robot.lift.position_tolerance)
    assert done["lift"]["state"] == "holding"
    assert done["fork_surface_height"] == pytest.approx(0.94, abs=0.01)


def test_forks_extend_and_retract_via_api(client, config):
    client.post("/api/simulation/start")
    assert client.post("/api/robot/forks/target", json={"position": 0.45}).status_code == 200
    done = _wait_for(client, lambda rb: rb["forks"]["at_target"] and rb["forks"]["position"] > 0.4)
    assert done["forks"]["position"] == pytest.approx(0.45, abs=config.robot.forks.position_tolerance)
    r = client.post("/api/robot/forks/jog", json={"delta": -0.2})
    assert r.status_code == 200 and r.json()["target"] == pytest.approx(0.25)
    assert "retracting" in r.json()["message"]
    assert client.post("/api/robot/forks/target", json={"position": 0.0}).status_code == 200
    done = _wait_for(client, lambda rb: rb["forks"]["at_target"] and rb["forks"]["position"] < 0.01)
    assert done["forks"]["state"] == "holding"


def test_stop_endpoint_halts_lift_mid_travel(client):
    client.post("/api/simulation/start")
    client.post("/api/robot/lift/target", json={"position": 1.5})
    _wait_for(client, lambda rb: rb["lift"]["position"] > 0.2)
    r = client.post("/api/robot/lift/stop")
    assert r.status_code == 200
    held = r.json()["stopped"][0]["target"]
    assert held < 1.0
    time.sleep(0.6)
    lift = _robot(client)["lift"]
    assert lift["target"] == held
    assert lift["position"] == pytest.approx(held, abs=0.01), "lift kept moving after stop"
    assert lift["state"] == "holding"


def test_drive_release_does_not_interrupt_lift(client):
    """Regression: releasing a drive key sends /api/robot/stop; that must not
    freeze a lift or fork move in progress."""
    client.post("/api/simulation/start")
    client.post("/api/robot/lift/target", json={"position": 0.6})
    client.post("/api/robot/velocity", json={"linear": 0.3, "angular": 0.0, "duration": 0.3})
    _wait_for(client, lambda rb: rb["lift"]["position"] > 0.05)
    assert client.post("/api/robot/stop").status_code == 200
    lift = _robot(client)["lift"]
    assert lift["target"] == 0.6 and lift["state"] == "moving"
    done = _wait_for(client, lambda rb: rb["lift"]["at_target"])
    assert done["lift"]["position"] == pytest.approx(0.6, abs=0.005)


def test_estop_and_pause_hold_mechanisms(client):
    client.post("/api/simulation/start")
    client.post("/api/robot/forks/target", json={"position": 0.6})
    _wait_for(client, lambda rb: rb["forks"]["position"] > 0.1)
    snap = client.post("/api/robot/estop").json()["snapshot"]
    forks = snap["world"]["robot"]["forks"]
    assert forks["target"] < 0.5 and forks["target"] == pytest.approx(forks["position"], abs=0.01)
    assert snap["world"]["robot"]["command"]["active"] is False

    client.post("/api/robot/lift/target", json={"position": 1.0})
    _wait_for(client, lambda rb: rb["lift"]["position"] > 0.1)
    snap = client.post("/api/simulation/pause").json()["snapshot"]
    lift = snap["world"]["robot"]["lift"]
    assert lift["target"] < 0.9 and lift["target"] == pytest.approx(lift["position"], abs=0.01)


def test_websocket_streams_actual_mechanism_positions(client):
    client.post("/api/simulation/start")
    with client.websocket_connect("/ws/telemetry") as ws:
        robot = ws.receive_json()["snapshot"]["world"]["robot"]
        for name in ("lift", "forks"):
            assert {"position", "velocity", "target", "error", "at_target", "state", "fault", "applied_force"} <= set(
                robot[name]
            )
        assert "fork_surface_height" in robot
        assert client.post("/api/robot/lift/target", json={"position": 0.5}).status_code == 200
        seen = []
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            lift = ws.receive_json()["snapshot"]["world"]["robot"]["lift"]
            seen.append(lift["position"])
            if lift["at_target"] and lift["target"] == 0.5:
                break
        assert seen[-1] == pytest.approx(0.5, abs=0.005)
        assert any(0.05 < p < 0.45 for p in seen), "intermediate positions streamed while moving"
        assert all(b >= a - 1e-6 for a, b in zip(seen, seen[1:]))


def test_mechanism_events_logged(client):
    client.post("/api/simulation/start")
    client.post("/api/robot/lift/target", json={"position": 0.3})
    client.post("/api/robot/lift/target", json={"position": 5.0})
    with client.websocket_connect("/ws/telemetry") as ws:
        events = ws.receive_json()["events"]
    messages = [e["message"] for e in events]
    assert any("Lift target 0.300 m" in m for m in messages)
    assert any(e["level"] == "warning" and "Rejected malformed request to /api/robot/lift/target" in e["message"]
               for e in events)  # fmt: skip
