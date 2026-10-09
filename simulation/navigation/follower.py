"""Differential-drive path following (pure control logic, no physics calls).

Each update takes the *measured* chassis pose and returns a body-velocity
request (v, w). The caller sends it through the robot's existing drive
interface, which applies the acceleration/braking ramps and the dead-man
timeout, so the robot only ever moves through its simulated wheel dynamics.

Strategy: drive straight at the active waypoint, turning on the spot when the
heading error is large (the robot is a differential drive and the planner
keeps the turning circle clear). Speed is capped by heading error and by a
braking profile towards corners that need a stop and towards the goal.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal

from ..config import FollowerConfig, MotionLimits

Phase = Literal["rotate", "drive", "align", "arrived", "failed"]
Point = tuple[float, float]


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class FollowerCommand:
    linear: float
    angular: float
    phase: Phase
    target_index: int
    heading_error: float
    cross_track_error: float
    distance_to_goal: float  # straight line, m
    remaining_distance: float  # along the route, m
    failure: str | None = None


class PathFollower:
    def __init__(
        self,
        waypoints: Sequence[Point],
        cfg: FollowerConfig,
        limits: MotionLimits,
        final_yaw: float | None = None,
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError("a route needs at least two waypoints")
        self.points = [(float(x), float(y)) for x, y in waypoints]
        self.cfg = cfg
        self.limits = limits
        self.final_yaw = final_yaw
        self.index = 1  # active target waypoint
        self._segment_lengths = [math.dist(a, b) for a, b in zip(self.points, self.points[1:])]
        self._rotating = False
        self._position_reached = False
        self._best_remaining = math.inf
        self._best_heading = math.inf
        self._since_progress = 0.0
        self.done = False
        self.failure: str | None = None

    # ------------------------------------------------------------------ #
    def remaining_from(self, x: float, y: float) -> float:
        """Distance along the route from (x, y) via the active waypoint to the goal."""
        rest = sum(self._segment_lengths[self.index :])
        return math.dist((x, y), self.points[self.index]) + rest

    def update(self, x: float, y: float, yaw: float, dt: float) -> FollowerCommand:
        cfg = self.cfg
        last = len(self.points) - 1
        goal = self.points[last]

        # 1. Advance past reached / overtaken intermediate waypoints. A corner
        #    sharp enough to turn on the spot must be reached precisely: cutting
        #    it early would leave the clearance-checked route segments.
        while self.index < last:
            a, b = self.points[self.index - 1], self.points[self.index]
            sharp = _turn_angle(a, b, self.points[self.index + 1]) > cfg.rotate_in_place_threshold
            tolerance = cfg.goal_tolerance if sharp else cfg.waypoint_tolerance
            if math.dist((x, y), b) < tolerance or _projection(a, b, (x, y)) >= 1.0:
                self.index += 1
                self._rotating = False
            else:
                break

        a, b = self.points[self.index - 1], self.points[self.index]
        cross = _distance_to_segment(a, b, (x, y))
        d_goal = math.dist((x, y), goal)
        remaining = self.remaining_from(x, y)

        def command(v: float, w: float, phase: Phase, err: float, failure: str | None = None) -> FollowerCommand:
            return FollowerCommand(v, w, phase, self.index, err, cross, d_goal, remaining, failure)

        if self.done:
            return command(0.0, 0.0, "failed" if self.failure else "arrived", 0.0, self.failure)

        # 2. Safety: stay on the planned (clearance-checked) corridor.
        if cross > cfg.max_cross_track_error and not self._position_reached:
            return self._fail(command, f"robot left the planned route ({cross:.2f} m off)")

        # 3. Final position and heading.
        if self.index == last and (d_goal <= cfg.goal_tolerance or (self._position_reached and d_goal <= 2 * cfg.goal_tolerance)):
            self._position_reached = True
            if self.final_yaw is not None:
                err = wrap_angle(self.final_yaw - yaw)
                if abs(err) > cfg.heading_tolerance:
                    self._track_progress(remaining, abs(err), dt)
                    if self._since_progress > cfg.progress_timeout:
                        return self._fail(command, "no progress aligning to the final heading")
                    return command(0.0, self._turn_rate(err), "align", err)
            self.done = True
            return command(0.0, 0.0, "arrived", 0.0)
        self._position_reached = False

        # 4. Heading to the active waypoint.
        desired = math.atan2(b[1] - y, b[0] - x)
        err = wrap_angle(desired - yaw)
        threshold = cfg.rotate_in_place_threshold
        if abs(err) > threshold or (self._rotating and abs(err) > threshold / 2):
            self._rotating = True
            self._track_progress(remaining, abs(err), dt)
            if self._since_progress > cfg.progress_timeout:
                return self._fail(command, f"no progress for {cfg.progress_timeout:.1f} s while turning")
            return command(0.0, self._turn_rate(err), "rotate", err)
        self._rotating = False

        # 5. Speed: cruise, reduced by heading error, braking for sharp corners and the goal.
        v = cfg.cruise_speed * (1.0 - abs(err) / threshold)
        decel = cfg.braking_fraction * self.limits.max_linear_deceleration
        d_corner = math.dist((x, y), b)
        if self.index < last:
            v_corner = cfg.cruise_speed * max(0.0, 1.0 - _turn_angle(a, b, self.points[self.index + 1]) / threshold)
            v = min(v, math.sqrt(v_corner**2 + 2 * decel * d_corner))
        v = min(v, math.sqrt(2 * decel * remaining))
        w = self._turn_rate(err)

        self._track_progress(remaining, math.inf, dt)
        if self._since_progress > cfg.progress_timeout:
            return self._fail(command, f"no progress for {cfg.progress_timeout:.1f} s (blocked?)")
        return command(max(0.0, v), w, "drive", err)

    # ------------------------------------------------------------------ #
    def _turn_rate(self, err: float) -> float:
        rate = self.cfg.heading_gain * err
        return max(-self.cfg.max_turn_rate, min(self.cfg.max_turn_rate, rate))

    def _track_progress(self, remaining: float, heading_error: float, dt: float) -> None:
        """Progress = route distance shrinking, or heading error shrinking while turning."""
        progressed = False
        if remaining < self._best_remaining - self.cfg.min_progress:
            self._best_remaining = remaining
            progressed = True
        if heading_error < self._best_heading - self.cfg.min_progress:
            self._best_heading = heading_error
            progressed = True
        if heading_error == math.inf:
            self._best_heading = math.inf  # driving: the next turn is tracked afresh
        if progressed:
            self._since_progress = 0.0
        else:
            self._since_progress += dt

    def _fail(self, command, reason: str) -> FollowerCommand:
        self.done = True
        self.failure = reason
        return command(0.0, 0.0, "failed", 0.0, reason)


def _projection(a: Point, b: Point, p: Point) -> float:
    """Normalised position of p's projection along segment a-b (0 at a, 1 at b)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return 1.0
    return ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2


def _distance_to_segment(a: Point, b: Point, p: Point) -> float:
    t = max(0.0, min(1.0, _projection(a, b, p)))
    return math.dist(p, (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))


def _turn_angle(a: Point, b: Point, c: Point) -> float:
    """Heading change at b when going a -> b -> c (0 = straight on)."""
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    return abs(wrap_angle(h2 - h1))
