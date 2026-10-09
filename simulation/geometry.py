"""Static warehouse geometry derived from the canonical configuration.

Every wall, rack post, shelf board and station is reduced to an axis-aligned
box here. The physics world creates one collision body per box and the API
serves the same list to the frontend, which renders exactly these boxes. There
is no second, hand-maintained map anywhere else.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Literal

from .config import ContainerConfig, RackConfig, RackType, Vec3, WarehouseConfig

BoxKind = Literal["wall", "rack_post", "rack_shelf", "station"]


@dataclass(frozen=True)
class StaticBox:
    """An axis-aligned static collision box in world coordinates (metres)."""

    id: str
    kind: BoxKind
    group: str
    center: Vec3
    size: Vec3
    level: int | None = None

    @property
    def half_extents(self) -> Vec3:
        return (self.size[0] / 2, self.size[1] / 2, self.size[2] / 2)

    @property
    def aabb(self) -> tuple[Vec3, Vec3]:
        h = self.half_extents
        c = self.center
        return (c[0] - h[0], c[1] - h[1], c[2] - h[2]), (c[0] + h[0], c[1] + h[1], c[2] + h[2])

    def to_dict(self) -> dict:
        return asdict(self)


def wall_boxes(warehouse: WarehouseConfig) -> list[StaticBox]:
    """Perimeter walls whose inner faces sit exactly on the floor boundary."""
    sx, sy = warehouse.floor.size_x, warehouse.floor.size_y
    t, h = warehouse.walls.thickness, warehouse.walls.height
    return [
        StaticBox("wall-north", "wall", "walls", (0.0, sy / 2 + t / 2, h / 2), (sx + 2 * t, t, h)),
        StaticBox("wall-south", "wall", "walls", (0.0, -sy / 2 - t / 2, h / 2), (sx + 2 * t, t, h)),
        StaticBox("wall-east", "wall", "walls", (sx / 2 + t / 2, 0.0, h / 2), (t, sy, h)),
        StaticBox("wall-west", "wall", "walls", (-sx / 2 - t / 2, 0.0, h / 2), (t, sy, h)),
    ]


def rack_footprint(rack: RackConfig, rack_type: RackType) -> tuple[tuple[float, float], tuple[float, float]]:
    """XY bounding rectangle (min, max) of a rack."""
    along, across = rack_type.length / 2, rack_type.depth / 2
    hx, hy = (along, across) if rack.axis == "x" else (across, along)
    cx, cy = rack.center
    return (cx - hx, cy - hy), (cx + hx, cy + hy)


def rack_boxes(rack: RackConfig, rack_type: RackType) -> list[StaticBox]:
    """Upright posts at every bay boundary plus one shelf board per level.

    Shelf boards span post-centre to post-centre so their end faces are buried
    inside the posts (no coplanar faces) while the posts define the rack's
    outer footprint.
    """
    L, D, H = rack_type.length, rack_type.depth, rack_type.height
    post, t = rack_type.post_size, rack_type.shelf_thickness
    cx, cy = rack.center

    def place(u: float, v: float, z: float, su: float, sv: float, sz: float) -> tuple[Vec3, Vec3]:
        # (u, v) are rack-local coordinates: u along the rack, v across it.
        if rack.axis == "x":
            return (cx + u, cy + v, z), (su, sv, sz)
        return (cx + v, cy + u, z), (sv, su, sz)

    boxes: list[StaticBox] = []
    span = L - post
    v_post = D / 2 - post / 2
    for i in range(rack_type.bays + 1):
        u = -span / 2 + i * span / rack_type.bays
        for side, v in (("front", -v_post), ("back", v_post)):
            center, size = place(u, v, H / 2, post, post, H)
            boxes.append(StaticBox(f"{rack.id}/post-{i}-{side}", "rack_post", rack.id, center, size))
    for level, top in enumerate(rack_type.levels):
        center, size = place(0.0, 0.0, top - t / 2, L - post, D - post, t)
        boxes.append(StaticBox(f"{rack.id}/shelf-{level}", "rack_shelf", rack.id, center, size, level))
    return boxes


def station_boxes(warehouse: WarehouseConfig) -> list[StaticBox]:
    return [
        StaticBox(
            s.id, "station", s.id, (s.center[0], s.center[1], s.size[2] / 2), (s.size[0], s.size[1], s.size[2])
        )
        for s in warehouse.stations
    ]


def build_static_geometry(warehouse: WarehouseConfig) -> list[StaticBox]:
    """All static collision boxes of the warehouse, in a stable order."""
    boxes = wall_boxes(warehouse)
    for rack in warehouse.racks:
        boxes.extend(rack_boxes(rack, warehouse.rack_type(rack)))
    boxes.extend(station_boxes(warehouse))
    return boxes



def container_initial_position(warehouse: WarehouseConfig, container: ContainerConfig) -> Vec3:
    """Centre of the container resting on its configured location (station or slot)."""
    try:
        station = warehouse.station(container.location)
        return (station.center[0], station.center[1], station.size[2] + container.size[2] / 2)
    except StopIteration:
        # Not a station, must be a slot
        slot = next(s for s in warehouse.slots if s.id == container.location)
        return (slot.center[0], slot.center[1], slot.center[2] + container.size[2] / 2)
