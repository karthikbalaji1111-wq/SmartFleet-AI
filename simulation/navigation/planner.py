"""A* path planning on the static occupancy grid.

Independent of PyBullet, FastAPI and the frontend so it can be tested directly.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Literal

from .grid import SQRT2, Cell, OccupancyGrid, Point

PlanStatus = Literal[
    "success",
    "invalid_input",
    "start_out_of_bounds",
    "goal_out_of_bounds",
    "start_blocked",
    "goal_blocked",
    "unreachable",
]


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    heading: float  # direction of travel leaving this waypoint (arrival heading for the last one), rad


@dataclass(frozen=True)
class PlanResult:
    status: PlanStatus
    message: str
    start: Point
    goal: Point
    start_cell: Cell | None = None
    goal_cell: Cell | None = None
    path: tuple[Point, ...] = ()  # raw A* route: exact start, cell centres, exact goal
    waypoints: tuple[Waypoint, ...] = ()  # simplified route the robot follows
    length: float = 0.0  # along the waypoints, m
    raw_length: float = 0.0  # along the raw path, m
    expanded: int = 0  # nodes popped from the open set
    time_ms: float = 0.0
    start_snapped: bool = False  # start was inside the inflated zone; first segment escapes it
    obstacle: str | None = None  # obstacle blocking an invalid start/goal, if known
    cells: tuple[Cell, ...] = field(default=(), repr=False)

    @property
    def success(self) -> bool:
        return self.status == "success"


def octile(a: Cell, b: Cell, resolution: float) -> float:
    """Admissible, consistent heuristic for 8-connected grids (metres)."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return resolution * (max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy))


def astar(grid: OccupancyGrid, start: Cell, goal: Cell, dynamic_blocked_cells: set[int] = frozenset()) -> tuple[list[Cell] | None, int]:
    """A* over free cells. Returns (cells from start to goal or None, expanded)."""
    if start == goal:
        return [start], 0
    width, res, penalty = grid.width, grid.resolution, grid.penalty
    heuristic = octile if grid.allow_diagonal else (
        lambda a, b, r: r * (abs(a[0] - b[0]) + abs(a[1] - b[1]))
    )  # fmt: skip
    g = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed = bytearray(width * grid.height)
    counter = 0
    open_heap: list[tuple[float, float, int, Cell]] = [(heuristic(start, goal, res), 0.0, counter, start)]
    expanded = 0
    while open_heap:
        cell = heapq.heappop(open_heap)[3]
        i = cell[1] * width + cell[0]
        if closed[i]:
            continue  # stale heap entry
        closed[i] = 1
        g_cur = g[cell]
        expanded += 1
        if cell == goal:
            route = [cell]
            while route[-1] in parent:
                route.append(parent[route[-1]])
            route.reverse()
            return route, expanded
        for nx, ny, cost in grid.neighbors(*cell):
            j = ny * width + nx
            if j in dynamic_blocked_cells:
                continue
            if closed[j]:
                continue
            nxt = (nx, ny)
            # Proximity penalty only ever increases cost, so the heuristic stays admissible.
            candidate = g_cur + cost * (1.0 + penalty[j])
            if candidate < g.get(nxt, math.inf) - 1e-12:
                g[nxt] = candidate
                parent[nxt] = cell
                counter += 1
                h = heuristic(nxt, goal, res)
                # Tie-break on larger g (deeper nodes first), then insertion order: deterministic.
                heapq.heappush(open_heap, (candidate + h, -candidate, counter, nxt))
    return None, expanded


def path_length(points: list[Point] | tuple[Point, ...]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def simplify(grid: OccupancyGrid, points: list[Point], first_free_index: int = 0, dynamic_blocked_cells: set[int] = frozenset()) -> list[Point]:
    """Greedy line-of-sight simplification that never reduces clearance.

    Points ``i+1..j-1`` are replaced by the straight segment ``i -> j`` only if
    every cell that segment touches (supercover traversal) is free — the hard
    footprint clearance — *and* no more than ``1.5`` cells closer to obstacles
    than the closest of the replaced points. Shortcuts therefore cannot cut
    through obstacles, and only trim A*'s cell staircase around corners rather
    than hugging them. Points before ``first_free_index`` (an escape segment out
    of the inflated zone) are kept unchanged.
    """
    if len(points) <= 2:
        return list(points)
    # Clearance beyond the preferred band is "open floor": any shortcut keeping
    # at least that much clearance is as good as the original route there.
    cap = grid.inflation_radius + grid.preferred_clearance
    slack = 1.5 * grid.resolution
    clear = [min(grid.cell_clearance(x, y), cap) - slack for x, y in points]
    out = list(points[: first_free_index + 1])
    i = first_free_index
    last = len(points) - 1
    while i < last:
        j = i + 1  # the next raw point is always reachable (adjacent cells)
        floor = min(clear[i], clear[j])
        while j + 1 <= last:
            needed = min(floor, clear[j + 1])
            if not grid.line_of_sight(points[i], points[j + 1], min_clearance=needed - 1e-9) or any((cx + cy * grid.width) in dynamic_blocked_cells for cx, cy in grid.traverse(points[i], points[j + 1])):
                break
            j += 1
            floor = needed
        out.append(points[j])
        i = j
    return out


def validate_path(grid: OccupancyGrid, points: list[Point] | tuple[Point, ...], skip_first_segment: bool = False, dynamic_blocked_cells: set[int] = frozenset()) -> bool:
    """Every segment must stay on free cells (the planner's own contract)."""
    segments = list(zip(points, points[1:]))
    if skip_first_segment:
        segments = segments[1:]
    return all(grid.line_of_sight(a, b) and not any((cx + cy * grid.width) in dynamic_blocked_cells for cx, cy in grid.traverse(a, b)) for a, b in segments)


def _headings(points: list[Point], final_yaw: float | None) -> tuple[Waypoint, ...]:
    out = []
    for k, (x, y) in enumerate(points):
        if k + 1 < len(points):
            nx, ny = points[k + 1]
            heading = math.atan2(ny - y, nx - x)
        elif final_yaw is not None:
            heading = final_yaw
        elif k > 0:
            px, py = points[k - 1]
            heading = math.atan2(y - py, x - px)
        else:
            heading = 0.0
        out.append(Waypoint(x, y, heading))
    return tuple(out)


def plan_path(
    grid: OccupancyGrid,
    start: Point,
    goal: Point,
    *,
    final_yaw: float | None = None,
    start_snap_radius: float = 0.0,
    simplify_path: bool = True,
    dynamic_blocked_cells: set[int] = frozenset(),
) -> PlanResult:
    """Plan from a world start point to a world goal point.

    The goal must lie in a free cell (snapping a picked point to free space is
    the caller's decision). A start inside the inflated zone — e.g. after
    manual driving near a rack — may be moved to the nearest free cell within
    ``start_snap_radius``; the route then begins with that short escape segment.
    """
    t0 = time.perf_counter()

    def result(status: PlanStatus, message: str, **kw) -> PlanResult:
        return PlanResult(status, message, start, goal, time_ms=(time.perf_counter() - t0) * 1000, **kw)

    try:
        sx, sy, gx, gy = (float(v) for v in (*start, *goal))
    except (TypeError, ValueError):
        return result("invalid_input", "start and goal must be (x, y) pairs of numbers")
    if not all(math.isfinite(v) for v in (sx, sy, gx, gy)):
        return result("invalid_input", "start and goal coordinates must be finite")
    if final_yaw is not None and not math.isfinite(final_yaw):
        return result("invalid_input", "final heading must be finite")

    start_cell = grid.world_to_cell(sx, sy)
    if start_cell is None:
        return result("start_out_of_bounds", f"start ({sx:.2f}, {sy:.2f}) is outside the warehouse floor")
    goal_cell = grid.world_to_cell(gx, gy)
    if goal_cell is None:
        return result("goal_out_of_bounds", f"goal ({gx:.2f}, {gy:.2f}) is outside the warehouse floor")
    if not grid.is_free(*goal_cell):
        near = grid.obstacle_near(*goal_cell)
        where = f" (too close to {near})" if near else ""
        return result(
            "goal_blocked", f"goal ({gx:.2f}, {gy:.2f}) is not navigable{where}",
            start_cell=start_cell, goal_cell=goal_cell, obstacle=near,
        )  # fmt: skip

    snapped = False
    route_start = start_cell
    if not grid.is_free(*start_cell):
        near = grid.obstacle_near(*start_cell)
        candidate = grid.nearest_free(sx, sy, start_snap_radius) if start_snap_radius > 0 else None
        if candidate is None:
            where = f" (too close to {near})" if near else ""
            return result(
                "start_blocked",
                f"robot position ({sx:.2f}, {sy:.2f}) is inside the obstacle clearance zone{where}; "
                "drive it into open floor first",
                start_cell=start_cell, goal_cell=goal_cell, obstacle=near,
            )  # fmt: skip
        route_start, snapped = candidate, True

    if grid.component(*route_start) != grid.component(*goal_cell):
        return result(
            "unreachable", f"goal ({gx:.2f}, {gy:.2f}) is not reachable from the robot's position",
            start_cell=start_cell, goal_cell=goal_cell, start_snapped=snapped,
        )  # fmt: skip

    cells, expanded = astar(grid, route_start, goal_cell, dynamic_blocked_cells)
    if cells is None:  # pragma: no cover - components make this a safety net
        return result("unreachable", "no route found", start_cell=start_cell, goal_cell=goal_cell, expanded=expanded)

    centres = [grid.cell_to_world(*c) for c in cells]
    raw = [(sx, sy), *centres, (gx, gy)]
    escape = 1 if snapped else 0
    points = simplify(grid, raw, escape, dynamic_blocked_cells) if simplify_path else raw
    # Drop zero-length duplicates (e.g. start exactly at a cell centre).
    dedup = [points[0]] + [p for prev, p in zip(points, points[1:]) if math.dist(prev, p) > 1e-9]
    if len(dedup) == 1:
        dedup = [dedup[0], dedup[0]]  # start == goal: a degenerate one-segment route
    return result(
        "success",
        "route found" if len(cells) > 1 else "robot is already at the goal",
        start_cell=start_cell, goal_cell=goal_cell,
        path=tuple(raw), waypoints=_headings(dedup, final_yaw),
        length=path_length(dedup), raw_length=path_length(raw),
        expanded=expanded, start_snapped=snapped, cells=tuple(cells),
    )  # fmt: skip
