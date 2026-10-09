"""Warehouse configuration consistency (no physics required)."""

from __future__ import annotations

import json
import math

import pytest
from pydantic import ValidationError

from simulation.config import DEFAULT_CONFIG_PATH, AppConfig
from simulation.geometry import build_static_geometry, container_initial_position, rack_footprint
from simulation.robot_model import fork_surface_height, fork_x_range, footprint_half_extents


def _raw() -> dict:
    return json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))


def _xy_overlap(a, b, tol: float = 1e-9) -> bool:
    (alo, ahi), (blo, bhi) = a, b
    return alo[0] < bhi[0] - tol and blo[0] < ahi[0] - tol and alo[1] < bhi[1] - tol and blo[1] < ahi[1] - tol


def test_canonical_config_loads(config):
    assert config.schema_version == 1
    assert config.frame.up_axis == "z"
    assert config.simulation.timestep == pytest.approx(1 / 240)
    assert len(config.warehouse.racks) >= 4


def test_static_geometry_counts_match_config(config):
    boxes = build_static_geometry(config.warehouse)
    kinds = {k: [b for b in boxes if b.kind == k] for k in ("wall", "rack_post", "rack_shelf", "station")}
    assert len(kinds["wall"]) == 4
    expected_posts = sum((config.warehouse.rack_type(r).bays + 1) * 2 for r in config.warehouse.racks)
    expected_shelves = sum(len(config.warehouse.rack_type(r).levels) for r in config.warehouse.racks)
    assert len(kinds["rack_post"]) == expected_posts
    assert len(kinds["rack_shelf"]) == expected_shelves
    assert len(kinds["station"]) == len(config.warehouse.stations)
    assert len({b.id for b in boxes}) == len(boxes), "static box ids must be unique"


def test_shelf_levels_are_distinct_heights(config):
    for rack in config.warehouse.racks:
        shelves = [b for b in build_static_geometry(config.warehouse) if b.group == rack.id and b.kind == "rack_shelf"]
        tops = sorted(b.center[2] + b.size[2] / 2 for b in shelves)
        assert tops == pytest.approx(sorted(config.warehouse.rack_type(rack).levels))
        assert all(b - a > 0.3 for a, b in zip(tops, tops[1:])), "shelf levels should be visibly distinct"


def test_static_geometry_inside_floor(config):
    sx, sy = config.warehouse.floor.size_x / 2, config.warehouse.floor.size_y / 2
    for box in build_static_geometry(config.warehouse):
        if box.kind == "wall":
            continue
        lo, hi = box.aabb
        assert -sx <= lo[0] and hi[0] <= sx and -sy <= lo[1] and hi[1] <= sy, box.id
        assert lo[2] >= -1e-9, box.id


def test_walls_enclose_floor(config):
    sx, sy = config.warehouse.floor.size_x / 2, config.warehouse.floor.size_y / 2
    walls = {b.id: b.aabb for b in build_static_geometry(config.warehouse) if b.kind == "wall"}
    assert walls["wall-north"][0][1] == pytest.approx(sy)
    assert walls["wall-south"][1][1] == pytest.approx(-sy)
    assert walls["wall-east"][0][0] == pytest.approx(sx)
    assert walls["wall-west"][1][0] == pytest.approx(-sx)


def test_racks_stations_and_zones_do_not_overlap(config):
    wh = config.warehouse
    footprints = {r.id: rack_footprint(r, wh.rack_type(r)) for r in wh.racks}
    for s in wh.stations:
        footprints[s.id] = (
            (s.center[0] - s.size[0] / 2, s.center[1] - s.size[1] / 2),
            (s.center[0] + s.size[0] / 2, s.center[1] + s.size[1] / 2),
        )
    ids = sorted(footprints)
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            assert not _xy_overlap(footprints[a], footprints[b]), f"{a} overlaps {b}"
    # Painted zones must be free of racks.
    for z in wh.zones:
        zone_fp = ((z.center[0] - z.size[0] / 2, z.center[1] - z.size[1] / 2),
                   (z.center[0] + z.size[0] / 2, z.center[1] + z.size[1] / 2))  # fmt: skip
        for r in wh.racks:
            assert not _xy_overlap(zone_fp, footprints[r.id]), f"zone {z.id} overlaps rack {r.id}"


def test_stations_inside_their_zones(config):
    for s in config.warehouse.stations:
        z = config.warehouse.zone(s.zone)
        assert abs(s.center[0] - z.center[0]) + s.size[0] / 2 <= z.size[0] / 2 + 1e-9
        assert abs(s.center[1] - z.center[1]) + s.size[1] / 2 <= z.size[1] / 2 + 1e-9


def test_robot_start_pose_is_clear_and_inside_home_zone(config):
    robot, wh = config.robot, config.warehouse
    half_len, half_wid = footprint_half_extents(robot)
    radius = math.hypot(half_len, half_wid)
    x, y = robot.start_pose.x, robot.start_pose.y
    assert abs(x) + radius < wh.floor.size_x / 2 and abs(y) + radius < wh.floor.size_y / 2
    for box in build_static_geometry(wh):
        lo, hi = box.aabb
        dx = max(lo[0] - x, 0.0, x - hi[0])
        dy = max(lo[1] - y, 0.0, y - hi[1])
        assert math.hypot(dx, dy) > radius + 0.05, f"robot start pose too close to {box.id}"
    home = next(z for z in wh.zones if z.kind == "home")
    assert abs(x - home.center[0]) <= home.size[0] / 2 and abs(y - home.center[1]) <= home.size[1] / 2



def test_container_rests_on_its_station(config):
    wh = config.warehouse
    container = wh.containers[0]
    station = wh.station(container.location)
    x, y, z = container_initial_position(wh, container)
    assert (x, y) == pytest.approx(station.center)
    assert z - container.size[2] / 2 == pytest.approx(station.size[2])
    assert container.size[0] <= station.size[0] and container.size[1] <= station.size[1]


def test_every_shelf_level_is_within_fork_reach(config):
    lo = fork_surface_height(config.robot, config.robot.lift.lower)
    hi = fork_surface_height(config.robot, config.robot.lift.upper)
    for rack_type in config.warehouse.rack_types.values():
        for level in rack_type.levels:
            assert lo <= level <= hi, f"shelf at {level} m outside fork reach [{lo:.2f}, {hi:.2f}] m"


def test_retracted_forks_stay_within_chassis(config):
    back, front = fork_x_range(config.robot, config.robot.forks.lower)
    assert -config.robot.chassis.length / 2 <= back and front <= config.robot.chassis.length / 2
    _, reach = fork_x_range(config.robot, config.robot.forks.upper)
    assert reach - config.robot.chassis.length / 2 >= config.warehouse.rack_types["tote-shelving-4L"].depth * 0.8


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda raw: raw["warehouse"]["racks"].append(dict(raw["warehouse"]["racks"][0])), "duplicate"),
        (lambda raw: raw["warehouse"]["racks"][0].update(type="missing-type"), "unknown rack type"),
        (lambda raw: raw["robot"]["lift"].update(upper=-1.0), "actuator upper limit must exceed"),
        (lambda raw: raw["robot"]["forks"].update(default_position=0.9), "default_position must lie"),
        (lambda raw: raw["robot"]["lift"].update(max_jog_step=5.0), "max_jog_step"),
        (lambda raw: raw["robot"]["limits"].update(max_linear_velocity=5.0), "wheel speed"),
        (lambda raw: raw["warehouse"]["rack_types"]["tote-shelving-4L"].update(levels=[0.8, 0.4]), "increasing"),
        (lambda raw: raw.update(unexpected_key=1), "Extra inputs"),
    ],
)
def test_invalid_configs_are_rejected(mutate, message):
    raw = _raw()
    mutate(raw)
    with pytest.raises(ValidationError, match=message):
        AppConfig.model_validate(raw)
