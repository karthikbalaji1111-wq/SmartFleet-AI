"""Navigation telemetry / route models (served by the API and streamed over WebSocket)."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict

NavStatus = Literal["idle", "planning", "planned", "navigating", "paused", "arrived", "cancelled", "failed"]


class _Model(BaseModel):
    model_config = ConfigDict(frozen=True)


class PlannerMetrics(_Model):
    status: str  # planner status: success / goal_blocked / unreachable / ...
    message: str
    length: float  # simplified route length, m
    raw_length: float  # A* cell route length, m
    expanded: int  # A* nodes expanded
    time_ms: float
    waypoint_count: int
    raw_point_count: int
    start_snapped: bool


class DestinationModel(_Model):
    id: str | None  # configured destination id, or None for a picked floor point
    label: str
    x: float
    y: float
    yaw: float | None
    requested_x: float  # what the operator asked for (before snapping)
    requested_y: float
    snapped: bool
    snap_distance: float


class RouteWaypoint(_Model):
    x: float
    y: float
    heading: float


class RouteModel(_Model):
    version: int
    destination: DestinationModel
    start: tuple[float, float]
    waypoints: list[RouteWaypoint]  # simplified route the controller follows
    path: list[tuple[float, float]]  # raw A* route (exact start, cell centres, exact goal)
    planner: PlannerMetrics


class NavigationTelemetry(_Model):
    status: NavStatus
    destination: DestinationModel | None
    route_version: int  # changes whenever the route changes; fetch the route only then
    waypoint_index: int | None  # active target waypoint in route.waypoints
    waypoint_count: int
    phase: str | None  # follower phase: rotate / drive / align / settling
    distance_to_goal: float | None  # straight line from the measured chassis position, m
    remaining_distance: float | None  # along the route, m
    route_length: float | None
    progress: float | None  # 0..1 along the route
    heading_error: float | None
    cross_track_error: float | None
    command_linear: float  # last autonomous request, m/s
    command_angular: float  # rad/s
    elapsed: float  # s of sim time since navigation started
    planner: PlannerMetrics | None
    reason: str | None  # why it failed / was cancelled / paused, or the last planning error
    interlock: list[str]  # live reasons the robot may not travel autonomously now
