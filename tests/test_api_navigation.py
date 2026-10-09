"""Navigation REST endpoints and WebSocket telemetry against the real physics loop."""

from __future__ import annotations

import base64
import math
import time

import pytest
from fastapi.testclient import TestClient

from backend.app.main import create_app
from simulation.navigation.grid import grid_for
from simulation.world import pybullet_available

from .conftest import PYBULLET_SKIP_REASON

pytestmark = pytest.mark.skipif(not pybullet_available(), reason=PYBULLET_SKIP_REASON)


@pytest.fixture
def client():
    with TestClient(create_app()) as c:
        yield c


def _state(client):
    return client.get("/api/simulation/state").json()["world"]


def _wait(client, predicate, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        world = _state(client)
        if predicate(world):
            return world
        time.sleep(0.05)
    raise AssertionError(f"timed out; navigation: {world['navigation']}")


def test_navigation_state_and_destinations(client, config):
    body = client.get("/api/navigation").json()
    assert body["navigation"]["status"] == "idle" and body["route"] is None
    assert [d["id"] for d in body["destinations"]] == [d.id for d in config.navigation.destinations]


def test_occupancy_grid_endpoint_matches_planner_grid(client, config):
    body = client.get("/api/navigation/grid").json()
    grid = grid_for(config)
    cells = base64.b64decode(body["cells"])
    assert (body["width"], body["height"]) == (grid.width, grid.height)
    assert len(cells) == grid.width * grid.height and cells == bytes(grid.cells)
    assert set(cells) <= {0, 1, 2}
    assert body["inflation_radius"] == pytest.approx(grid.inflation_radius)


def test_plan_by_destination_id_returns_route(client):
    r = client.post("/api/navigation/plan", json={"destination_id": "aisle-ab"})
    assert r.status_code == 200
    body = r.json()
    route = body["route"]
    assert body["navigation"]["status"] == "planned"
    assert route["destination"]["id"] == "aisle-ab"
    assert len(route["waypoints"]) >= 3 and len(route["path"]) > len(route["waypoints"])
    assert route["planner"]["status"] == "success" and route["planner"]["expanded"] > 0
    assert route["planner"]["length"] == pytest.approx(
        sum(math.dist((a["x"], a["y"]), (b["x"], b["y"])) for a, b in zip(route["waypoints"], route["waypoints"][1:]))
    )
    assert body["navigation"]["route_version"] == route["version"]


@pytest.mark.parametrize(
    ("payload", "hint"),
    [
        ({"x": 0.0, "y": 2.0, "snap": False}, "clearance zone"),
        ({"x": 0.0, "y": 2.0}, "no navigable floor"),
        ({"x": 40.0, "y": 0.0}, "outside the warehouse"),
        ({"destination_id": "nowhere"}, "unknown destination"),
    ],
)
def test_invalid_destinations_rejected(client, payload, hint):
    r = client.post("/api/navigation/plan", json=payload)
    assert r.status_code == 422
    assert hint in r.json()["detail"]
    assert r.json()["code"] == "invalid_destination"
    assert client.get("/api/navigation").json()["route"] is None


@pytest.mark.parametrize(
    "payload",
    [{}, {"destination_id": "home", "x": 1.0, "y": 1.0}, {"x": 1.0}, {"x": "1", "y": 0.0}, {"x": 1.0, "y": 0.0, "z": 1}],
)
def test_malformed_plan_requests_rejected(client, payload):
    assert client.post("/api/navigation/plan", json=payload).status_code == 422


def test_floor_point_is_snapped_to_navigable_floor(client):
    r = client.post("/api/navigation/plan", json={"x": 0.0, "y": 1.4})
    assert r.status_code == 200
    dest = r.json()["route"]["destination"]
    assert dest["snapped"] and dest["requested_y"] == 1.4 and dest["y"] < 1.4
    assert "snapped" in r.json()["message"]


def test_start_requires_running_simulation_and_a_route(client):
    assert client.post("/api/navigation/start").status_code == 409  # simulation stopped
    client.post("/api/simulation/start")
    r = client.post("/api/navigation/start")
    assert r.status_code == 409 and "plan a route first" in r.json()["detail"]
    assert client.post("/api/navigation/pause").status_code == 409
    assert client.post("/api/navigation/cancel").status_code == 409


def test_interlock_rejects_start_with_forks_extended(client):
    client.post("/api/simulation/start")
    client.post("/api/robot/forks/target", json={"position": 0.3})
    _wait(client, lambda w: w["robot"]["forks"]["at_target"] and w["robot"]["forks"]["position"] > 0.25)
    assert client.post("/api/navigation/plan", json={"destination_id": "aisle-bc"}).status_code == 200
    r = client.post("/api/navigation/start")
    assert r.status_code == 409
    body = r.json()
    assert body["code"] == "interlock" and any("forks extended" in reason for reason in body["reasons"])
    world = _state(client)
    assert world["navigation"]["status"] == "planned" and world["navigation"]["interlock"]
    assert world["robot"]["forks"]["position"] > 0.25, "navigation request did not move the forks"


def test_autonomous_drive_reaches_point_and_streams_progress(client, config):
    client.post("/api/simulation/start")
    assert client.post("/api/navigation/plan", json={"x": 3.0, "y": 0.0}).status_code == 200
    with client.websocket_connect("/ws/telemetry") as ws:
        first = ws.receive_json()
        assert first["route_update"]["route"]["destination"]["x"] == 3.0
        assert client.post("/api/navigation/start").status_code == 200
        statuses, positions, distances = [], [], []
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            msg = ws.receive_json()
            assert msg["route_update"] is None, "route is not resent while unchanged"
            nav = msg["snapshot"]["world"]["navigation"]
            statuses.append(nav["status"])
            positions.append(msg["snapshot"]["world"]["robot"]["pose"]["position"][0])
            distances.append(nav["distance_to_goal"])
            if nav["status"] == "arrived":
                break
    assert "navigating" in statuses and statuses[-1] == "arrived"
    assert positions[-1] == pytest.approx(3.0, abs=config.navigation.controller.goal_tolerance)
    assert positions[0] - positions[-1] > 1.8, "robot drove west ~2 m in physics"
    assert distances[-1] <= config.navigation.controller.goal_tolerance < distances[0]
    world = _state(client)
    assert abs(world["robot"]["velocity"]["forward"]) < 0.02


def test_manual_drive_takes_over_and_estop_cancels(client):
    client.post("/api/simulation/start")
    client.post("/api/navigation/plan", json={"destination_id": "aisle-bc"})
    assert client.post("/api/navigation/start").status_code == 200
    _wait(client, lambda w: w["robot"]["velocity"]["forward"] > 0.2)
    assert client.post("/api/robot/lift/target", json={"position": 0.5}).json()["code"] == "navigation_active"
    r = client.post("/api/robot/velocity", json={"linear": 0.0, "angular": 0.5, "duration": 0.3})
    assert r.status_code == 200
    nav = _state(client)["navigation"]
    assert nav["status"] == "paused" and nav["reason"] == "manual drive takeover"
    r = client.post("/api/navigation/resume")  # a brief turn on the spot leaves the robot on its route
    assert r.status_code == 200 and r.json()["navigation"]["status"] == "navigating"
    client.post("/api/robot/estop")
    nav = _state(client)["navigation"]
    assert nav["status"] == "cancelled" and nav["reason"] == "emergency stop"
    world = _wait(client, lambda w: abs(w["robot"]["velocity"]["forward"]) < 0.02, timeout=5)
    assert not world["robot"]["command"]["active"]


def test_navigation_events_logged(client):
    client.post("/api/simulation/start")
    client.post("/api/navigation/plan", json={"destination_id": "aisle-ab"})
    client.post("/api/navigation/plan", json={"x": 0.0, "y": 2.0})
    with client.websocket_connect("/ws/telemetry") as ws:
        events = ws.receive_json()["events"]
    msgs = [(e["source"], e["message"]) for e in events]
    assert any(src == "navigation" and m.startswith("Route planned to Aisle A/B") for src, m in msgs)
    assert any(src == "navigation" and "Rejected" in m and "no navigable floor" in m for src, m in msgs)
