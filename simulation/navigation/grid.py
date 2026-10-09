"""2-D occupancy grid of the *known static* warehouse.

Built from :func:`simulation.geometry.build_static_geometry` — the very boxes
that exist as PyBullet collision bodies — so the planner never keeps a second
map. This is prior map knowledge, not sensor perception.

Conventions
-----------
* World frame as everywhere else: metres, X east, Y north, floor centre at 0.
* Cell ``(ix, iy)`` covers ``[x0 + ix*res, x0 + (ix+1)*res) x [y0 + iy*res, ...)``
  with ``(x0, y0)`` the floor's south-west corner; its centre is
  ``x0 + (ix + 0.5) * res``.
* Obstacles are inflated by the robot's circumscribed radius (it turns in
  place) plus ``clearance_margin``: a path of *free cell centres* keeps the
  whole footprint clear in every orientation.
"""

from __future__ import annotations

import math
from array import array
from collections import deque
from collections.abc import Iterator

from ..config import AppConfig
from ..geometry import build_static_geometry
from ..robot_model import footprint_half_extents

FREE = 0
INFLATED = 1  # free floor, but the robot centre may not be here
OCCUPIED = 2  # inside an obstacle's footprint

Cell = tuple[int, int]
Point = tuple[float, float]

# 8-connected moves (dx, dy, cost in cells), in a fixed order for determinism.
_STRAIGHT = ((1, 0), (-1, 0), (0, 1), (0, -1))
_DIAGONAL = ((1, 1), (-1, 1), (-1, -1), (1, -1))
SQRT2 = math.sqrt(2.0)


class OccupancyGrid:
    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: Point,
        cells: bytearray,
        *,
        inflation_radius: float = 0.0,
        robot_radius: float = 0.0,
        allow_diagonal: bool = True,
        obstacle_ids: list[str] | None = None,
        owners: array | None = None,
        clearance: array | None = None,
        preferred_clearance: float = 0.0,
        proximity_weight: float = 0.0,
    ) -> None:
        if width <= 0 or height <= 0 or len(cells) != width * height:
            raise ValueError("grid dimensions do not match the cell buffer")
        self.width, self.height = width, height
        self.resolution = resolution
        self.origin = origin
        self.cells = cells
        self.inflation_radius = inflation_radius
        self.robot_radius = robot_radius
        self.allow_diagonal = allow_diagonal
        self.obstacle_ids = obstacle_ids or []
        self.preferred_clearance = preferred_clearance
        n = width * height
        self.owners = owners if owners is not None else array("h", [-1]) * n
        # Distance from each cell centre to the nearest obstacle footprint (m), inf if far.
        self.clearance = clearance if clearance is not None else array("d", [math.inf]) * n
        # Multiplicative extra cost for entering a cell: 0 in open floor, up to
        # proximity_weight at the hard clearance limit. Keeps routes centred in aisles.
        self.penalty = array("d", [0.0]) * n
        if preferred_clearance > 0 and proximity_weight > 0:
            for i in range(n):
                spare = self.clearance[i] - inflation_radius
                if cells[i] == FREE and spare < preferred_clearance:
                    self.penalty[i] = proximity_weight * (1.0 - max(0.0, spare) / preferred_clearance)
        self.components = self._label_components()

    # ------------------------------------------------------------------ #
    # Construction from the canonical warehouse
    # ------------------------------------------------------------------ #
    @classmethod
    def from_config(cls, config: AppConfig) -> OccupancyGrid:
        nav = config.navigation.grid
        floor = config.warehouse.floor
        res = nav.resolution
        width = math.ceil(floor.size_x / res - 1e-9)
        height = math.ceil(floor.size_y / res - 1e-9)
        x0, y0 = -floor.size_x / 2, -floor.size_y / 2
        robot_radius = math.hypot(*footprint_half_extents(config.robot))
        radius = robot_radius + nav.clearance_margin
        reach = radius + nav.preferred_clearance  # distances are only needed this far out

        cells = bytearray(width * height)
        owners = array("h", [-1]) * (width * height)
        clearance = array("d", [math.inf]) * (width * height)
        boxes = build_static_geometry(config.warehouse)
        ids = [b.id for b in boxes]

        for k, box in enumerate(boxes):
            (lx, ly, _), (hx, hy, _) = box.aabb
            ix_lo = max(0, math.floor((lx - reach - x0) / res))
            ix_hi = min(width - 1, math.floor((hx + reach - x0) / res))
            iy_lo = max(0, math.floor((ly - reach - y0) / res))
            iy_hi = min(height - 1, math.floor((hy + reach - y0) / res))
            for iy in range(iy_lo, iy_hi + 1):
                cy = y0 + (iy + 0.5) * res
                dy = max(ly - cy, 0.0, cy - hy)
                if dy >= reach:
                    continue
                row = iy * width
                for ix in range(ix_lo, ix_hi + 1):
                    cx = x0 + (ix + 0.5) * res
                    dx = max(lx - cx, 0.0, cx - hx)
                    d = math.hypot(dx, dy)
                    i = row + ix
                    if d < clearance[i]:
                        clearance[i] = d
                        owners[i] = k
                    if d < radius:
                        state = OCCUPIED if d == 0.0 else INFLATED
                        if state > cells[i]:
                            cells[i] = state

        # The floor boundary is a hard limit even if a config had no walls.
        for iy in range(height):
            cy = y0 + (iy + 0.5) * res
            for ix in range(width):
                cx = x0 + (ix + 0.5) * res
                edge = min(cx - x0, x0 + floor.size_x - cx, cy - y0, y0 + floor.size_y - cy)
                i = iy * width + ix
                if edge < clearance[i]:
                    clearance[i] = edge
                if edge < radius and cells[i] == FREE:
                    cells[i] = INFLATED

        return cls(
            width, height, res, (x0, y0), cells,
            inflation_radius=radius, robot_radius=robot_radius,
            allow_diagonal=nav.allow_diagonal, obstacle_ids=ids, owners=owners, clearance=clearance,
            preferred_clearance=nav.preferred_clearance, proximity_weight=nav.proximity_weight,
        )  # fmt: skip

    # ------------------------------------------------------------------ #
    # Coordinates
    # ------------------------------------------------------------------ #
    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_cell(self, x: float, y: float) -> Cell | None:
        """Cell containing a world point, or ``None`` outside the floor."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        ix = math.floor((x - self.origin[0]) / self.resolution)
        iy = math.floor((y - self.origin[1]) / self.resolution)
        return (ix, iy) if self.in_bounds(ix, iy) else None

    def cell_to_world(self, ix: int, iy: int) -> Point:
        """World coordinates of a cell centre."""
        return (
            self.origin[0] + (ix + 0.5) * self.resolution,
            self.origin[1] + (iy + 0.5) * self.resolution,
        )

    def index(self, ix: int, iy: int) -> int:
        return iy * self.width + ix

    # ------------------------------------------------------------------ #
    # Queries
    # ------------------------------------------------------------------ #
    def state(self, ix: int, iy: int) -> int:
        return self.cells[self.index(ix, iy)] if self.in_bounds(ix, iy) else OCCUPIED

    def is_free(self, ix: int, iy: int) -> bool:
        return self.in_bounds(ix, iy) and self.cells[iy * self.width + ix] == FREE

    def component(self, ix: int, iy: int) -> int:
        """Connected free-space region id (-1 for blocked cells)."""
        return self.components[self.index(ix, iy)] if self.in_bounds(ix, iy) else -1

    def obstacle_near(self, ix: int, iy: int) -> str | None:
        """Id of the obstacle that blocks a cell (``None`` for free / boundary)."""
        if not self.in_bounds(ix, iy):
            return None
        k = self.owners[self.index(ix, iy)]
        return self.obstacle_ids[k] if 0 <= k < len(self.obstacle_ids) else None

    def neighbors(self, ix: int, iy: int) -> Iterator[tuple[int, int, float]]:
        """Free neighbours with move cost in metres. Diagonal moves are only
        allowed when both adjacent orthogonal cells are free (no corner cutting)."""
        res = self.resolution
        for dx, dy in _STRAIGHT:
            nx, ny = ix + dx, iy + dy
            if self.is_free(nx, ny):
                yield nx, ny, res
        if not self.allow_diagonal:
            return
        for dx, dy in _DIAGONAL:
            nx, ny = ix + dx, iy + dy
            if self.is_free(nx, ny) and self.is_free(ix + dx, iy) and self.is_free(ix, iy + dy):
                yield nx, ny, res * SQRT2

    def nearest_free(self, x: float, y: float, max_radius: float, component: int | None = None) -> Cell | None:
        """Free cell whose centre is nearest to ``(x, y)`` within ``max_radius``
        (optionally restricted to one connected region). Deterministic."""
        r = math.ceil(max_radius / self.resolution) + 1
        cx = math.floor((x - self.origin[0]) / self.resolution)
        cy = math.floor((y - self.origin[1]) / self.resolution)
        best: tuple[float, int, int] | None = None
        for iy in range(cy - r, cy + r + 1):
            for ix in range(cx - r, cx + r + 1):
                if not self.is_free(ix, iy):
                    continue
                if component is not None and self.component(ix, iy) != component:
                    continue
                wx, wy = self.cell_to_world(ix, iy)
                d = math.hypot(wx - x, wy - y)
                if d <= max_radius and (best is None or (d, iy, ix) < best):
                    best = (d, iy, ix)
        return None if best is None else (best[2], best[1])

    # ------------------------------------------------------------------ #
    # Line traversal (for line-of-sight / path validation)
    # ------------------------------------------------------------------ #
    def traverse(self, a: Point, b: Point) -> Iterator[Cell]:
        """Every cell a straight segment touches (supercover: when the segment
        passes exactly through a cell corner, both side cells are included).
        Cells outside the grid are yielded too, so callers can reject them."""
        res, (ox, oy) = self.resolution, self.origin
        gx0, gy0 = (a[0] - ox) / res, (a[1] - oy) / res
        gx1, gy1 = (b[0] - ox) / res, (b[1] - oy) / res
        ix, iy = math.floor(gx0), math.floor(gy0)
        ex, ey = math.floor(gx1), math.floor(gy1)
        dx, dy = gx1 - gx0, gy1 - gy0
        step_x = (dx > 0) - (dx < 0)
        step_y = (dy > 0) - (dy < 0)
        t_dx = abs(1.0 / dx) if dx else math.inf
        t_dy = abs(1.0 / dy) if dy else math.inf
        t_x = ((ix + 1 - gx0) / dx if dx > 0 else (gx0 - ix) / -dx) if dx else math.inf
        t_y = ((iy + 1 - gy0) / dy if dy > 0 else (gy0 - iy) / -dy) if dy else math.inf
        yield ix, iy
        for _ in range(abs(ex - ix) + abs(ey - iy) + 2):
            if (ix, iy) == (ex, ey) or min(t_x, t_y) > 1.0:
                return
            if abs(t_x - t_y) < 1e-9:
                yield ix + step_x, iy
                yield ix, iy + step_y
                ix, iy = ix + step_x, iy + step_y
                t_x += t_dx
                t_y += t_dy
            elif t_x < t_y:
                ix += step_x
                t_x += t_dx
            else:
                iy += step_y
                t_y += t_dy
            yield ix, iy

    def line_of_sight(self, a: Point, b: Point, min_clearance: float = 0.0) -> bool:
        """True if every cell touched by segment a-b is free and (optionally)
        at least ``min_clearance`` from the nearest obstacle."""
        for ix, iy in self.traverse(a, b):
            if not self.is_free(ix, iy):
                return False
            if min_clearance > 0 and self.clearance[iy * self.width + ix] < min_clearance:
                return False
        return True

    def cell_clearance(self, x: float, y: float) -> float:
        """Distance from the centre of the cell containing (x, y) to the nearest obstacle."""
        cell = self.world_to_cell(x, y)
        return 0.0 if cell is None else self.clearance[self.index(*cell)]

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _label_components(self) -> array:
        """Label 8-connected free regions using the same move rules as A*."""
        labels = array("i", [-1]) * (self.width * self.height)
        label = 0
        for start in range(self.width * self.height):
            if self.cells[start] != FREE or labels[start] != -1:
                continue
            labels[start] = label
            queue = deque([start])
            while queue:
                i = queue.popleft()
                ix, iy = i % self.width, i // self.width
                for nx, ny, _ in self.neighbors(ix, iy):
                    j = ny * self.width + nx
                    if labels[j] == -1:
                        labels[j] = label
                        queue.append(j)
            label += 1
        return labels

    def free_fraction(self) -> float:
        return self.cells.count(FREE) / len(self.cells)


_GRID_CACHE: dict[int, tuple[AppConfig, OccupancyGrid]] = {}


def grid_for(config: AppConfig) -> OccupancyGrid:
    """Occupancy grid for a config, built once per config object (the grid is
    immutable static-map data, so every world for that config can share it)."""
    entry = _GRID_CACHE.get(id(config))
    if entry is None or entry[0] is not config:
        entry = (config, OccupancyGrid.from_config(config))
        _GRID_CACHE[id(config)] = entry
    return entry[1]
