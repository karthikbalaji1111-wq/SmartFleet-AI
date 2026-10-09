"""Occupancy grid and A* planner (pure algorithm tests, no physics)."""

from __future__ import annotations

import heapq
import json
import math
import random

import pytest

from simulation.config import DEFAULT_CONFIG_PATH, AppConfig
from simulation.geometry import build_static_geometry
from simulation.navigation import FREE, INFLATED, OCCUPIED, OccupancyGrid, astar, plan_path, validate_path
from simulation.navigation.grid import SQRT2
from simulation.robot_model import footprint_half_extents


def _raw() -> dict:
    return json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def grid(config) -> OccupancyGrid:
    return OccupancyGrid.from_config(config)


def _points(result):
    return [(w.x, w.y) for w in result.waypoints]


def _min_box_distance(config, x, y):
    best = math.inf
    for box in build_static_geometry(config.warehouse):
        (lx, ly, _), (hx, hy, _) = box.aabb
        best = min(best, math.hypot(max(lx - x, 0, x - hx), max(ly - y, 0, y - hy)))
    return best


def _sample(a, b, step=0.02):
    n = max(1, math.ceil(math.dist(a, b) / step))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]


# --------------------------------------------------------------------------- #
# Grid construction and coordinates
# --------------------------------------------------------------------------- #
def test_world_to_grid_conversion_round_trip(grid, config):
    sx, sy = config.warehouse.floor.size_x, config.warehouse.floor.size_y
    assert (grid.width, grid.height) == (round(sx / grid.resolution), round(sy / grid.resolution))
    assert grid.world_to_cell(-sx / 2, -sy / 2) == (0, 0)
    assert grid.world_to_cell(sx / 2 - 1e-6, sy / 2 - 1e-6) == (grid.width - 1, grid.height - 1)
    assert grid.cell_to_world(0, 0) == pytest.approx((-sx / 2 + grid.resolution / 2, -sy / 2 + grid.resolution / 2))
    for x, y in [(0.0, 0.0), (5.03, -2.27), (-11.95, 7.95), (3.333, 1.111)]:
        cell = grid.world_to_cell(x, y)
        cx, cy = grid.cell_to_world(*cell)
        assert abs(cx - x) <= grid.resolution / 2 + 1e-9 and abs(cy - y) <= grid.resolution / 2 + 1e-9
        assert grid.world_to_cell(cx, cy) == cell
    for outside in [(sx / 2, 0.0), (0.0, -sy / 2 - 0.01), (100.0, 100.0), (math.nan, 0.0), (0.0, math.inf)]:
        assert grid.world_to_cell(*outside) is None


def test_grid_is_built_from_canonical_collision_geometry(grid, config):
    for box in build_static_geometry(config.warehouse):
        cell = grid.world_to_cell(box.center[0], box.center[1])
        if cell is not None:  # wall centres lie just outside the floor
            assert grid.state(*cell) == OCCUPIED, box.id
    # A cell just outside rack B2's footprint is blocked by inflation and attributed to it.
    cell = grid.world_to_cell(0.0, 1.7 - 0.3)
    assert grid.state(*cell) == INFLATED
    assert grid.obstacle_near(*cell).startswith("B2/")
    start = config.robot.start_pose
    assert grid.is_free(*grid.world_to_cell(start.x, start.y))


def test_inflation_matches_robot_footprint_and_margin(grid, config):
    expected = math.hypot(*footprint_half_extents(config.robot)) + config.navigation.grid.clearance_margin
    assert grid.inflation_radius == pytest.approx(expected)
    # Every free cell centre keeps the full turning circle + margin clear of every obstacle.
    boxes = [b.aabb for b in build_static_geometry(config.warehouse)]
    sx, sy = config.warehouse.floor.size_x / 2, config.warehouse.floor.size_y / 2
    for iy in range(grid.height):
        for ix in range(grid.width):
            if grid.cells[iy * grid.width + ix] != FREE:
                continue
            x, y = grid.cell_to_world(ix, iy)
            assert min(x + sx, sx - x, y + sy, sy - y) >= expected - 1e-9
            for (lx, ly, _), (hx, hy, _) in boxes:
                d = math.hypot(max(lx - x, 0, x - hx), max(ly - y, 0, y - hy))
                assert d >= expected - 1e-9


def test_all_configured_destinations_are_navigable(grid, config):
    start = config.robot.start_pose
    home = grid.component(*grid.world_to_cell(start.x, start.y))
    for d in config.navigation.destinations:
        cell = grid.world_to_cell(d.x, d.y)
        assert cell is not None and grid.is_free(*cell), d.id
        assert grid.component(*cell) == home, f"{d.id} not reachable from home"


# --------------------------------------------------------------------------- #
# A* routes in the warehouse
# --------------------------------------------------------------------------- #
def test_direct_route_through_clear_aisle(grid):
    r = plan_path(grid, (-7.0, 0.25), (-1.0, 0.25))
    assert r.success
    assert _points(r) == [(-7.0, 0.25), (-1.0, 0.25)], "a clear aisle needs no intermediate waypoints"
    assert r.length == pytest.approx(6.0)
    assert r.expanded > 0 and r.time_ms >= 0


def test_route_around_rack_blocking_direct_path(grid, config):
    start, goal = (0.0, 3.75), (0.0, 0.25)  # aisle A/B -> aisle B/C; rack B2 in between
    assert not grid.line_of_sight(start, goal)
    r = plan_path(grid, start, goal)
    assert r.success
    assert r.length > math.dist(start, goal) + 4.0, "must detour around the end of rack B2"
    assert validate_path(grid, _points(r))
    xs = [x for x, _ in r.path]
    assert max(xs) > 2.4 + grid.inflation_radius or min(xs) < -2.4 - grid.inflation_radius
    for a, b in zip(_points(r), _points(r)[1:]):
        for x, y in _sample(a, b):
            assert _min_box_distance(config, x, y) >= grid.robot_radius, "footprint must clear every rack"


def test_unreachable_destination_detected():
    raw = _raw()
    # A closed pen of four stands around an open patch of floor east of home.
    raw["warehouse"]["stations"] += [
        {"id": "pen-n", "zone": "home", "center": [9.0, 1.4], "size": [3.0, 0.2, 1.0]},
        {"id": "pen-s", "zone": "home", "center": [9.0, -1.4], "size": [3.0, 0.2, 1.0]},
        {"id": "pen-e", "zone": "home", "center": [10.4, 0.0], "size": [0.2, 2.6, 1.0]},
        {"id": "pen-w", "zone": "home", "center": [7.6, 0.0], "size": [0.2, 2.6, 1.0]},
    ]
    grid = OccupancyGrid.from_config(AppConfig.model_validate(raw))
    inside = grid.world_to_cell(9.0, 0.0)
    assert grid.is_free(*inside), "the pen interior is free floor"
    r = plan_path(grid, (5.0, 0.0), (9.0, 0.0))
    assert r.status == "unreachable" and not r.waypoints
    cells, expanded = astar(grid, grid.world_to_cell(5.0, 0.0), inside)
    assert cells is None and expanded > 1000, "exhaustive search also finds no route"


@pytest.mark.parametrize(
    ("start", "goal", "status"),
    [
        ((0.0, 2.0), (-7.0, 0.25), "start_blocked"),
        ((5.0, 0.0), (0.0, 2.0), "goal_blocked"),
        ((5.0, 0.0), (-11.9, 0.0), "goal_blocked"),  # inside the wall clearance
        ((5.0, 0.0), (12.5, 0.0), "goal_out_of_bounds"),
        ((30.0, 0.0), (5.0, 0.0), "start_out_of_bounds"),
        ((5.0, math.nan), (0.0, 3.75), "invalid_input"),
        ((5.0, 0.0), ("a", 1.0), "invalid_input"),
    ],
)
def test_invalid_starts_and_goals_rejected(grid, start, goal, status):
    r = plan_path(grid, start, goal)
    assert r.status == status and not r.success
    assert not r.waypoints and r.message
    if status.endswith("blocked"):
        assert r.obstacle is not None


def test_start_equal_to_goal(grid):
    r = plan_path(grid, (5.0, 0.0), (5.0, 0.0), final_yaw=1.0)
    assert r.success and r.length == 0.0 and r.expanded == 0
    assert _points(r)[0] == _points(r)[-1] == (5.0, 0.0)
    assert r.waypoints[-1].heading == 1.0


def test_start_slightly_inside_clearance_zone_is_snapped(grid):
    start = (0.0, 1.7 - 0.685 + 0.15)  # 0.15 m inside rack B2's clearance zone
    assert not grid.is_free(*grid.world_to_cell(*start))
    assert plan_path(grid, start, (-7.0, 0.25)).status == "start_blocked"
    r = plan_path(grid, start, (-7.0, 0.25), start_snap_radius=0.35)
    assert r.success and r.start_snapped
    assert _points(r)[0] == start
    assert grid.is_free(*grid.world_to_cell(*_points(r)[1]))
    assert validate_path(grid, _points(r), skip_first_segment=True)


def test_path_reconstruction_order_and_headings(grid):
    r = plan_path(grid, (5.0, 0.0), (-3.5, 6.9), final_yaw=0.5)
    assert r.success
    assert r.cells[0] == grid.world_to_cell(5.0, 0.0) and r.cells[-1] == grid.world_to_cell(-3.5, 6.9)
    for (ax, ay), (bx, by) in zip(r.cells, r.cells[1:]):
        assert max(abs(ax - bx), abs(ay - by)) == 1, "consecutive cells are 8-neighbours"
    assert r.path[0] == (5.0, 0.0) and r.path[-1] == (-3.5, 6.9)
    wps = r.waypoints
    for a, b in zip(wps, wps[1:]):
        assert a.heading == pytest.approx(math.atan2(b.y - a.y, b.x - a.x))
    assert wps[-1].heading == 0.5
    assert r.length <= r.raw_length + 1e-9


def test_simplified_paths_stay_valid_with_clearance(grid, config):
    rng = random.Random(1234)
    free = [(ix, iy) for iy in range(0, grid.height, 3) for ix in range(0, grid.width, 3) if grid.is_free(ix, iy)]
    pairs = [(grid.cell_to_world(*rng.choice(free)), grid.cell_to_world(*rng.choice(free))) for _ in range(25)]
    pairs += [((5.0, 0.0), (d.x, d.y)) for d in config.navigation.destinations]
    for start, goal in pairs:
        r = plan_path(grid, start, goal)
        assert r.success, (start, goal, r.message)
        pts = _points(r)
        assert validate_path(grid, pts), (start, goal)
        assert validate_path(grid, r.path), "raw route is valid too"
        raw_min = min(grid.cell_clearance(x, y) for x, y in r.path)
        for a, b in zip(pts, pts[1:]):
            for x, y in _sample(a, b):
                assert grid.cell_clearance(x, y) >= min(raw_min, grid.inflation_radius) - 1e-9


def test_planning_is_deterministic(grid):
    a = plan_path(grid, (5.0, 0.0), (-3.5, -6.65))
    b = plan_path(grid, (5.0, 0.0), (-3.5, -6.65))
    assert a.waypoints == b.waypoints and a.path == b.path and a.expanded == b.expanded


def test_routes_prefer_aisle_centres(grid):
    r = plan_path(grid, (-7.0, 0.25), (2.0, 0.25))
    # Aisle B/C spans y in [-1.2, 1.7]; plain A* would hug the clearance edge (y = -0.515).
    assert all(-0.3 < y < 0.8 for _, y in r.path)


# --------------------------------------------------------------------------- #
# Synthetic grids: corner cutting, costs, optimality
# --------------------------------------------------------------------------- #
def _synthetic(rows: list[str], res=1.0, diagonal=True) -> OccupancyGrid:
    """rows[0] is the top (largest y); '#' blocked, '.' free."""
    height, width = len(rows), len(rows[0])
    cells = bytearray(width * height)
    for r, row in enumerate(rows):
        iy = height - 1 - r
        for ix, ch in enumerate(row):
            cells[iy * width + ix] = OCCUPIED if ch == "#" else FREE
    return OccupancyGrid(width, height, res, (0.0, 0.0), cells, allow_diagonal=diagonal)


def test_no_diagonal_corner_cutting():
    g = _synthetic(["#.", ".#"])  # free (0,0) and (1,1), only touching at a corner
    cells, _ = astar(g, (0, 0), (1, 1))
    assert cells is None
    assert not g.line_of_sight((0.5, 0.5), (1.5, 1.5)), "segment through the corner touches both blocked cells"
    assert plan_path(g, (0.5, 0.5), (1.5, 1.5)).status == "unreachable"


def test_diagonal_allowed_when_both_sides_free_and_costs_correct():
    g = _synthetic([".....", ".....", ".....", ".....", "....."], res=0.5)
    cells, _ = astar(g, (0, 0), (3, 3))
    assert cells == [(0, 0), (1, 1), (2, 2), (3, 3)]
    r = plan_path(g, g.cell_to_world(0, 0), g.cell_to_world(3, 3), simplify_path=False)
    assert r.raw_length == pytest.approx(3 * SQRT2 * 0.5)
    g4 = _synthetic([".....", ".....", ".....", ".....", "....."], res=0.5, diagonal=False)
    cells4, _ = astar(g4, (0, 0), (3, 3))
    assert len(cells4) == 7 and all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(cells4, cells4[1:]))


def _dijkstra_cost(g: OccupancyGrid, start, goal) -> float:
    dist = {start: 0.0}
    heap = [(0.0, start)]
    while heap:
        d, c = heapq.heappop(heap)
        if c == goal:
            return d
        if d > dist[c]:
            continue
        for nx, ny, cost in g.neighbors(*c):
            nd = d + cost * (1.0 + g.penalty[ny * g.width + nx])
            if nd < dist.get((nx, ny), math.inf) - 1e-12:
                dist[(nx, ny)] = nd
                heapq.heappush(heap, (nd, (nx, ny)))
    return math.inf


def _cost(g, cells):
    total = 0.0
    for a, b in zip(cells, cells[1:]):
        step = g.resolution * (SQRT2 if a[0] != b[0] and a[1] != b[1] else 1.0)
        total += step * (1.0 + g.penalty[b[1] * g.width + b[0]])
    return total


def test_astar_is_optimal_against_dijkstra(grid):
    rng = random.Random(7)
    free = [(ix, iy) for iy in range(grid.height) for ix in range(grid.width) if grid.is_free(ix, iy)]
    for _ in range(8):
        s, t = rng.choice(free), rng.choice(free)
        cells, _ = astar(grid, s, t)
        assert _cost(grid, cells) == pytest.approx(_dijkstra_cost(grid, s, t), rel=1e-9)


def test_maze_route_is_found_and_valid():
    g = _synthetic(
        [
            ".........",
            ".#######.",
            ".#.....#.",
            ".#.###.#.",
            ".#.#...#.",
            ".#.#####.",
            ".#.......",
            ".#######.",
            ".........",
        ],
        res=1.0,
    )
    start, goal = g.cell_to_world(4, 4), g.cell_to_world(0, 0)
    r = plan_path(g, start, goal)
    assert r.success and validate_path(g, _points(r)) and validate_path(g, r.path)
