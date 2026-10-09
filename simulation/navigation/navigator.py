"""Navigation state machine: destination -> A* route -> path following.

The navigator holds no robot pose of its own: every decision uses the
``RobotState`` measured by PyBullet that the world passes in. It never moves
the lift or forks; it only checks them (travel interlock).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..config import AppConfig
from .follower import FollowerCommand, PathFollower
from .grid import OccupancyGrid
from .models import DestinationModel, NavigationTelemetry, NavStatus, PlannerMetrics, RouteModel, RouteWaypoint
from .planner import PlanResult, plan_path, validate_path

if TYPE_CHECKING:
    from ..state import RobotState

STOPPED_SPEED = 0.02  # m/s: "stationary" for arrival
STOPPED_TURN_RATE = 0.05  # rad/s
START_TOLERANCE = 0.30  # m: robot may have drifted this far from the planned start


class NavigationError(RuntimeError):
    """A navigation request that cannot be honoured.

    ``code``: ``invalid_destination`` | ``unreachable`` | ``interlock`` |
    ``state`` | ``route_invalid``.
    """

    def __init__(self, code: str, message: str, reasons: list[str] | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.reasons = reasons or []


class MotionConflictError(RuntimeError):
    """A command that conflicts with active autonomous navigation (HTTP 409)."""


@dataclass(frozen=True)
class Destination:
    x: float
    y: float
    yaw: float | None
    label: str
    id: str | None
    requested: tuple[float, float]
    snapped: bool = False

    def model(self) -> DestinationModel:
        return DestinationModel(
            id=self.id, label=self.label, x=self.x, y=self.y, yaw=self.yaw,
            requested_x=self.requested[0], requested_y=self.requested[1],
            snapped=self.snapped, snap_distance=math.dist(self.requested, (self.x, self.y)),
        )  # fmt: skip


def _metrics(plan: PlanResult) -> PlannerMetrics:
    return PlannerMetrics(
        status=plan.status, message=plan.message, length=plan.length, raw_length=plan.raw_length,
        expanded=plan.expanded, time_ms=plan.time_ms, waypoint_count=len(plan.waypoints),
        raw_point_count=len(plan.path), start_snapped=plan.start_snapped,
    )  # fmt: skip


class Navigator:
    def __init__(self, config: AppConfig, grid: OccupancyGrid) -> None:
        self.config = config
        self.grid = grid
        self.cfg = config.navigation.controller
        self._events: list[tuple[str, str]] = []
        self.route_version = 0
        self.reset()

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        """Back to idle with no route (world reset)."""
        self.status: NavStatus = "idle"
        self.destination: Destination | None = None
        self.plan: PlanResult | None = None
        self.last_metrics: PlannerMetrics | None = None
        self.follower: PathFollower | None = None
        self.last_command: FollowerCommand | None = None
        self.reason: str | None = None
        self.elapsed = 0.0
        self._settling = False
        self._bump_route()

    def drain_events(self) -> list[tuple[str, str]]:
        events, self._events = self._events, []
        return events

    @property
    def active(self) -> bool:
        """True while the navigator is commanding the wheels."""
        return self.status == "navigating"

    # ------------------------------------------------------------------ #
    # Destinations and planning
    # ------------------------------------------------------------------ #
    def resolve_destination(
        self,
        destination_id: str | None = None,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        snap: bool = True,
    ) -> Destination:
        """A configured destination by id, or a floor point (snapped to the
        nearest navigable cell within ``goal_snap_radius`` if ``snap``)."""
        nav = self.config.navigation
        if destination_id is not None:
            d = nav.destination(destination_id)
            if d is None:
                raise NavigationError("invalid_destination", f"unknown destination {destination_id!r}")
            return Destination(d.x, d.y, d.yaw, d.label, d.id, (d.x, d.y))
        if x is None or y is None or not (math.isfinite(x) and math.isfinite(y)):
            raise NavigationError("invalid_destination", "destination needs a configured id or finite x and y")
        if yaw is not None and not math.isfinite(yaw):
            raise NavigationError("invalid_destination", "destination heading must be finite")
        cell = self.grid.world_to_cell(x, y)
        if cell is None:
            raise NavigationError("invalid_destination", f"({x:.2f}, {y:.2f}) is outside the warehouse floor")
        label = f"Floor point ({x:.2f}, {y:.2f})"
        if self.grid.is_free(*cell):
            return Destination(x, y, yaw, label, None, (x, y))
        near = self.grid.obstacle_near(*cell) or "the warehouse boundary"
        if not snap:
            raise NavigationError(
                "invalid_destination", f"({x:.2f}, {y:.2f}) is inside the clearance zone of {near}"
            )
        free = self.grid.nearest_free(x, y, nav.grid.goal_snap_radius)
        if free is None:
            raise NavigationError(
                "invalid_destination",
                f"({x:.2f}, {y:.2f}) is too close to {near}; no navigable floor within "
                f"{nav.grid.goal_snap_radius:.1f} m",
            )
        sx, sy = self.grid.cell_to_world(*free)
        return Destination(sx, sy, yaw, label, None, (x, y), snapped=True)

    def plan_route(self, start: tuple[float, float], destination: Destination) -> PlanResult:
        """Plan from the measured robot position. Replaces any previous route."""
        if self.status == "navigating":
            raise NavigationError("state", "navigation is active; pause or cancel it before planning a new route")
        self.status = "planning"
        plan = plan_path(
            self.grid, start, (destination.x, destination.y), final_yaw=destination.yaw,
            start_snap_radius=self.config.navigation.grid.start_snap_radius,
        )  # fmt: skip
        self.last_metrics = _metrics(plan)
        self.follower = None
        self.last_command = None
        self.elapsed = 0.0
        if not plan.success:
            self.status, self.plan, self.destination = "idle", None, None
            self.reason = plan.message
            self._bump_route()
            code = "unreachable" if plan.status == "unreachable" else "invalid_destination"
            raise NavigationError(code, plan.message)
        self.status, self.plan, self.destination, self.reason = "planned", plan, destination, None
        self._bump_route()
        self._events.append(
            ("info", f"Route planned to {destination.label}: {plan.length:.2f} m, {len(plan.waypoints)} waypoints, "
                     f"{plan.expanded} nodes expanded in {plan.time_ms:.1f} ms")
        )  # fmt: skip
        return plan

    # ------------------------------------------------------------------ #
    # Travel interlock
    # ------------------------------------------------------------------ #
    def interlock_reasons(self, robot: RobotState) -> list[str]:
        """Conservative travel interlock: forks retracted, lift at travel height,
        neither moving nor faulted. Never changes the mechanisms itself."""
        travel = self.config.navigation.travel
        reasons = []
        if robot.forks.position > travel.max_fork_extension or robot.forks.target > travel.max_fork_extension:
            reasons.append(
                f"forks extended {robot.forks.position:.3f} m (target {robot.forks.target:.3f} m); "
                f"retract them to ≤ {travel.max_fork_extension:.2f} m"
            )
        if robot.lift.position > travel.max_lift or robot.lift.target > travel.max_lift:
            reasons.append(
                f"lift at {robot.lift.position:.3f} m (target {robot.lift.target:.3f} m); "
                f"lower it to ≤ {travel.max_lift:.2f} m"
            )
        for name, mech in (("lift", robot.lift), ("forks", robot.forks)):
            if mech.state == "blocked":
                reasons.append(f"{name} is blocked ({mech.fault}); send it a new target first")
            elif mech.state == "moving":
                reasons.append(f"{name} is still moving")
        return reasons

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def start(self, robot: RobotState) -> None:
        if self.status == "navigating":
            raise NavigationError("state", "navigation is already running")
        if self.status == "paused":
            raise NavigationError("state", "navigation is paused; resume or cancel it")
        if self.plan is None:
            raise NavigationError("state", "plan a route first")
        if self.status != "planned":
            raise NavigationError("state", f"this route has {self.status}; plan it again")
        self._check_interlock(robot)
        x, y = robot.pose.position[0], robot.pose.position[1]
        drift = math.dist((x, y), self.plan.start)
        if drift > START_TOLERANCE:
            raise NavigationError(
                "route_invalid", f"the robot has moved {drift:.2f} m since the route was planned; plan again"
            )
        self.follower = PathFollower(
            [(w.x, w.y) for w in self.plan.waypoints], self.cfg, self.config.robot.limits, self.destination.yaw
        )
        self.status, self.reason, self.elapsed, self._settling = "navigating", None, 0.0, False
        self._events.append(("info", f"Navigation started to {self.destination.label}"))

    def pause(self, reason: str = "paused by operator") -> bool:
        if self.status != "navigating":
            return False
        self.status, self.reason = "paused", reason
        self._events.append(("info", f"Navigation paused: {reason}"))
        return True

    def resume(self, robot: RobotState) -> None:
        if self.status != "paused" or self.follower is None:
            raise NavigationError("state", "navigation is not paused")
        self._check_interlock(robot)
        x, y = robot.pose.position[0], robot.pose.position[1]
        pts = self.follower.points
        i = self.follower.index
        # The route is static and was validated; the robot must still be on it and
        # have a clear straight run to its active waypoint.
        off = min(_point_segment_distance((x, y), a, b) for a, b in zip(pts[i - 1 :], pts[i:]))
        if off > self.cfg.max_cross_track_error:
            raise NavigationError("route_invalid", f"the robot is {off:.2f} m off the route; plan again")
        cell = self.grid.world_to_cell(x, y)
        if cell is None or (self.grid.is_free(*cell) and not self.grid.line_of_sight((x, y), pts[i])):
            raise NavigationError("route_invalid", "no clear path back to the route; plan again")
        self.status, self.reason = "navigating", None
        self.follower._since_progress = 0.0  # pause time is not lack of progress
        self._events.append(("info", "Navigation resumed"))

    def cancel(self, reason: str = "cancelled by operator") -> bool:
        if self.status not in ("planned", "navigating", "paused"):
            return False
        self.status, self.reason = "cancelled", reason
        self._events.append(("warning" if "emergency" in reason else "info", f"Navigation cancelled: {reason}"))
        return True

    def fail(self, reason: str) -> None:
        self.status, self.reason = "failed", reason
        self._events.append(("warning", f"Navigation failed: {reason}"))

    # ------------------------------------------------------------------ #
    # Control tick (called by the world at controller rate while navigating)
    # ------------------------------------------------------------------ #
    def tick(self, robot: RobotState, dt: float) -> FollowerCommand | None:
        """Return the drive request for this control period, or None when the
        navigator has just stopped commanding (arrived / failed)."""
        if self.status != "navigating" or self.follower is None:
            return None
        self.elapsed += dt
        reasons = self.interlock_reasons(robot)
        if reasons:
            self.fail("travel interlock: " + "; ".join(reasons))
            return None
        x, y, _ = robot.pose.position
        cmd = self.follower.update(x, y, robot.pose.heading, dt)
        if cmd.phase == "failed":
            self.last_command = cmd
            self.fail(cmd.failure or "path follower failure")
            return None
        if cmd.phase == "arrived":
            # Wheels commanded to zero; report arrival once the chassis is still.
            self._settling = True
            stopped = abs(robot.velocity.forward) < STOPPED_SPEED and abs(robot.velocity.yaw_rate) < STOPPED_TURN_RATE
            if stopped:
                self.last_command = cmd
                self.status, self.reason = "arrived", None
                self._events.append(
                    ("info", f"Arrived at {self.destination.label}: {cmd.distance_to_goal * 100:.1f} cm from goal "
                             f"after {self.elapsed:.1f} s")
                )  # fmt: skip
                return None
        self.last_command = cmd
        return cmd

    # ------------------------------------------------------------------ #
    # Telemetry
    # ------------------------------------------------------------------ #
    def route_model(self) -> RouteModel | None:
        if self.plan is None or self.destination is None or self.last_metrics is None:
            return None
        return RouteModel(
            version=self.route_version,
            destination=self.destination.model(),
            start=self.plan.start,
            waypoints=[RouteWaypoint(x=w.x, y=w.y, heading=w.heading) for w in self.plan.waypoints],
            path=list(self.plan.path),
            planner=self.last_metrics,
        )

    def telemetry(self, robot: RobotState) -> NavigationTelemetry:
        x, y = robot.pose.position[0], robot.pose.position[1]
        plan, follower, cmd = self.plan, self.follower, self.last_command
        d_goal = remaining = progress = None
        if plan is not None and self.destination is not None:
            d_goal = math.dist((x, y), (self.destination.x, self.destination.y))
            if follower is not None and self.status in ("navigating", "paused"):
                remaining = follower.remaining_from(x, y)
            elif self.status == "arrived":
                remaining = 0.0
            else:
                remaining = plan.length
            progress = 1.0 if plan.length <= 0 else max(0.0, min(1.0, 1.0 - remaining / plan.length))
        moving = cmd is not None and self.status == "navigating"
        return NavigationTelemetry(
            status=self.status,
            destination=self.destination.model() if self.destination else None,
            route_version=self.route_version,
            waypoint_index=follower.index if follower is not None else None,
            waypoint_count=len(plan.waypoints) if plan is not None else 0,
            phase=("settling" if self._settling and moving else cmd.phase) if moving else None,
            distance_to_goal=d_goal,
            remaining_distance=remaining,
            route_length=plan.length if plan is not None else None,
            progress=progress,
            heading_error=cmd.heading_error if moving else None,
            cross_track_error=cmd.cross_track_error if cmd is not None else None,
            command_linear=cmd.linear if moving else 0.0,
            command_angular=cmd.angular if moving else 0.0,
            elapsed=self.elapsed,
            planner=self.last_metrics,
            reason=self.reason,
            interlock=self.interlock_reasons(robot),
        )

    # ------------------------------------------------------------------ #
    def _check_interlock(self, robot: RobotState) -> None:
        reasons = self.interlock_reasons(robot)
        if reasons:
            raise NavigationError("interlock", "unsafe travel configuration: " + "; ".join(reasons), reasons)

    def _bump_route(self) -> None:
        self.route_version += 1


def _point_segment_distance(p, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


__all__ = [
    "Destination",
    "MotionConflictError",
    "NavigationError",
    "Navigator",
    "validate_path",
]
