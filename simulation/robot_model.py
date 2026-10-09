"""Articulated storage-robot model generated from the canonical robot config.

One :class:`RobotDescription` is built from ``config/warehouse.json``. It is
serialised to URDF for PyBullet and served (as plain data) to the frontend,
which renders the very same primitives. Dimensions therefore cannot drift
between the physics model and the 3D view.

Robot body frame: +X forward, +Y left, +Z up, origin at the midpoint of the
drive-wheel axle. The base link's centre of mass is placed at that origin, so
PyBullet's base position (which is the base COM) equals the body-frame origin.

Kinematic tree::

    base_link ─┬─ left_wheel_joint   (continuous, axis +Y) ── left_wheel
               ├─ right_wheel_joint  (continuous, axis +Y) ── right_wheel
               ├─ caster_front_joint (fixed)               ── caster_front
               ├─ caster_rear_joint  (fixed)               ── caster_rear
               └─ mast_joint         (fixed)               ── mast
                    └─ lift_joint    (prismatic, axis +Z)  ── lift_carriage
                         └─ fork_joint (prismatic, axis +X) ── fork
"""

from __future__ import annotations

import argparse
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Literal

from .config import RobotConfig, Vec3, get_config

# Canonical link and joint names. The physics world validates these after
# loading the URDF and refuses to run if any are missing or mistyped.
BASE_LINK = "base_link"
LEFT_WHEEL_LINK = "left_wheel"
RIGHT_WHEEL_LINK = "right_wheel"
FRONT_CASTER_LINK = "caster_front"
REAR_CASTER_LINK = "caster_rear"
MAST_LINK = "mast"
LIFT_CARRIAGE_LINK = "lift_carriage"
FORK_LINK = "fork"

LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
FRONT_CASTER_JOINT = "caster_front_joint"
REAR_CASTER_JOINT = "caster_rear_joint"
MAST_JOINT = "mast_joint"
LIFT_JOINT = "lift_joint"
FORK_JOINT = "fork_joint"

JointType = Literal["fixed", "continuous", "prismatic"]

EXPECTED_JOINTS: dict[str, tuple[JointType, str]] = {
    LEFT_WHEEL_JOINT: ("continuous", LEFT_WHEEL_LINK),
    RIGHT_WHEEL_JOINT: ("continuous", RIGHT_WHEEL_LINK),
    FRONT_CASTER_JOINT: ("fixed", FRONT_CASTER_LINK),
    REAR_CASTER_JOINT: ("fixed", REAR_CASTER_LINK),
    MAST_JOINT: ("fixed", MAST_LINK),
    LIFT_JOINT: ("prismatic", LIFT_CARRIAGE_LINK),
    FORK_JOINT: ("prismatic", FORK_LINK),
}
EXPECTED_LINKS: tuple[str, ...] = (BASE_LINK, *(child for _, child in EXPECTED_JOINTS.values()))

# Internal design constants for the lift carriage / telescopic fork. They are
# derived details of the handling unit, not tuning parameters.
CARRIAGE_GAP = 0.005  # clearance between mast front face and carriage
CARRIAGE_BASE_Z = 0.03  # carriage frame height above the mast base at lift = 0
CARRIAGE_PLATE_THICKNESS = 0.03
CARRIAGE_HEIGHT = 0.25
STAGE_THICKNESS = 0.02  # fixed fork stage plate under the tines
FORK_BACKBAR_THICKNESS = 0.03
BUMPER_DEPTH = 0.03

MATERIALS: dict[str, tuple[float, float, float, float]] = {
    "chassis": (0.95, 0.72, 0.08, 1.0),
    "bumper": (0.10, 0.10, 0.11, 1.0),
    "tire": (0.07, 0.07, 0.08, 1.0),
    "hub": (0.78, 0.80, 0.83, 1.0),
    "caster": (0.32, 0.33, 0.36, 1.0),
    "mast": (0.56, 0.59, 0.63, 1.0),
    "carriage": (0.16, 0.42, 0.82, 1.0),
    "fork": (0.20, 0.21, 0.23, 1.0),
    "beacon": (1.00, 0.52, 0.05, 1.0),
}

ShapeKind = Literal["box", "cylinder", "sphere"]
_HALF_PI = math.pi / 2


@dataclass(frozen=True)
class Shape:
    """A primitive attached to a link. ``size`` is (x, y, z) for boxes,
    (radius, length) for cylinders (URDF convention: axis along local Z) and
    (radius,) for spheres."""

    name: str
    kind: ShapeKind
    size: tuple[float, ...]
    xyz: Vec3 = (0.0, 0.0, 0.0)
    rpy: Vec3 = (0.0, 0.0, 0.0)
    material: str = "chassis"
    collision: bool = True


@dataclass(frozen=True)
class Inertia:
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


@dataclass(frozen=True)
class Link:
    name: str
    mass: float
    com: Vec3
    inertia: Inertia
    shapes: tuple[Shape, ...]


@dataclass(frozen=True)
class Joint:
    name: str
    type: JointType
    parent: str
    child: str
    xyz: Vec3
    rpy: Vec3 = (0.0, 0.0, 0.0)
    axis: Vec3 = (0.0, 0.0, 0.0)
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None


@dataclass(frozen=True)
class RobotDescription:
    name: str
    links: tuple[Link, ...]
    joints: tuple[Joint, ...]
    materials: dict[str, tuple[float, float, float, float]] = field(default_factory=lambda: dict(MATERIALS))
    base_link: str = BASE_LINK

    def link(self, name: str) -> Link:
        return next(link for link in self.links if link.name == name)

    def joint(self, name: str) -> Joint:
        return next(joint for joint in self.joints if joint.name == name)

    def to_dict(self) -> dict:
        return asdict(self)

    def to_urdf(self) -> str:
        robot = ET.Element("robot", name=self.name)
        for mat_name, rgba in self.materials.items():
            material = ET.SubElement(robot, "material", name=mat_name)
            ET.SubElement(material, "color", rgba=_fmt(rgba))
        for link in self.links:
            el = ET.SubElement(robot, "link", name=link.name)
            inertial = ET.SubElement(el, "inertial")
            ET.SubElement(inertial, "origin", xyz=_fmt(link.com), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=_num(link.mass))
            i = link.inertia
            ET.SubElement(
                inertial,
                "inertia",
                ixx=_num(i.ixx), ixy=_num(i.ixy), ixz=_num(i.ixz),
                iyy=_num(i.iyy), iyz=_num(i.iyz), izz=_num(i.izz),
            )  # fmt: skip
            for shape in link.shapes:
                _shape_element(el, "visual", shape)
                if shape.collision:
                    _shape_element(el, "collision", shape)
        for joint in self.joints:
            el = ET.SubElement(robot, "joint", name=joint.name, type=joint.type)
            ET.SubElement(el, "parent", link=joint.parent)
            ET.SubElement(el, "child", link=joint.child)
            ET.SubElement(el, "origin", xyz=_fmt(joint.xyz), rpy=_fmt(joint.rpy))
            if joint.type != "fixed":
                ET.SubElement(el, "axis", xyz=_fmt(joint.axis))
            if joint.type == "prismatic":
                ET.SubElement(
                    el, "limit",
                    lower=_num(joint.lower), upper=_num(joint.upper),
                    effort=_num(joint.effort), velocity=_num(joint.velocity),
                )  # fmt: skip
            elif joint.type == "continuous":
                ET.SubElement(el, "limit", effort=_num(joint.effort), velocity=_num(joint.velocity))
        ET.indent(robot)
        return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"


# --------------------------------------------------------------------------- #
# Geometry helpers derived from the config (shared with tests and docs)
# --------------------------------------------------------------------------- #
def chassis_bottom_z(robot: RobotConfig) -> float:
    """Chassis underside height in the body frame (axle at z = 0)."""
    return -robot.wheels.radius + robot.chassis.ground_clearance


def chassis_top_z(robot: RobotConfig) -> float:
    return chassis_bottom_z(robot) + robot.chassis.height


def fork_surface_height(robot: RobotConfig, lift: float) -> float:
    """Height of the fork tines' top surface above the floor for a lift position."""
    return robot.wheels.radius + chassis_top_z(robot) + CARRIAGE_BASE_Z + lift + robot.forks.tine_thickness


def fork_x_range(robot: RobotConfig, extension: float) -> tuple[float, float]:
    """Body-frame X extent of the fork tines for a given extension."""
    root = robot.mast.offset_x + robot.mast.upright_size / 2 + CARRIAGE_GAP + CARRIAGE_PLATE_THICKNESS
    return root + extension, root + extension + robot.forks.length


def footprint_half_extents(robot: RobotConfig) -> tuple[float, float]:
    """Half length / half width of the robot's ground footprint (incl. bumpers, wheels)."""
    half_len = robot.chassis.length / 2 + BUMPER_DEPTH
    half_wid = (robot.wheels.track_width + robot.wheels.width) / 2
    return half_len, half_wid


# --------------------------------------------------------------------------- #
# Model construction
# --------------------------------------------------------------------------- #
def build_robot_description(robot: RobotConfig | None = None) -> RobotDescription:
    robot = robot or get_config().robot
    ch, wh, ca, ma, li, fo = robot.chassis, robot.wheels, robot.casters, robot.mast, robot.lift, robot.forks
    r = wh.radius
    z_bot, z_top = chassis_bottom_z(robot), chassis_top_z(robot)
    z_mid = (z_bot + z_top) / 2
    bumper_h = 0.08

    base_shapes = (
        Shape("chassis", "box", (ch.length, ch.width, ch.height), (0.0, 0.0, z_mid), material="chassis"),
        Shape(
            "bumper_front", "box", (BUMPER_DEPTH, ch.width, bumper_h),
            (ch.length / 2 + BUMPER_DEPTH / 2, 0.0, z_bot + bumper_h / 2 + 0.01), material="bumper",
        ),
        Shape(
            "bumper_rear", "box", (BUMPER_DEPTH, ch.width, bumper_h),
            (-ch.length / 2 - BUMPER_DEPTH / 2, 0.0, z_bot + bumper_h / 2 + 0.01), material="bumper",
        ),
    )  # fmt: skip
    # Battery-heavy chassis: COM on the axle midpoint (body-frame origin).
    base = _link(BASE_LINK, ch.mass, base_shapes, com=(0.0, 0.0, 0.0))

    def wheel(name: str) -> Link:
        shapes = (
            Shape("tire", "cylinder", (r, wh.width), rpy=(_HALF_PI, 0.0, 0.0), material="tire"),
            # Visual spoke bar so wheel rotation (reported by physics) is visible.
            Shape("spoke", "box", (1.5 * r, wh.width + 0.006, 0.25 * r), material="hub", collision=False),
        )
        return _link(name, wh.mass, shapes)

    def caster(name: str) -> Link:
        return _link(name, ca.mass, (Shape("ball", "sphere", (ca.radius,), material="caster"),))

    us, sp, mh = ma.upright_size, ma.upright_spacing, ma.height
    mast_shapes = (
        Shape("upright_left", "box", (us, us, mh), (0.0, sp / 2, mh / 2), material="mast"),
        Shape("upright_right", "box", (us, us, mh), (0.0, -sp / 2, mh / 2), material="mast"),
        Shape("crossbar_bottom", "box", (us, sp + us, us), (0.0, 0.0, us / 2), material="mast"),
        Shape("crossbar_top", "box", (us, sp + us, us), (0.0, 0.0, mh - us / 2), material="mast"),
        Shape("beacon", "cylinder", (0.035, 0.05), (0.0, 0.0, mh + 0.025), material="beacon", collision=False),
    )
    mast = _link(MAST_LINK, ma.mass, mast_shapes)

    tine_outer = fo.tine_spacing / 2 + fo.tine_width / 2
    pt = CARRIAGE_PLATE_THICKNESS
    carriage_shapes = (
        Shape("backplate", "box", (pt, sp - us, CARRIAGE_HEIGHT), (pt / 2, 0.0, CARRIAGE_HEIGHT / 2), material="carriage"),
        Shape(
            "fork_stage", "box", (fo.length, 2 * tine_outer + 0.04, STAGE_THICKNESS),
            (pt + fo.length / 2, 0.0, -STAGE_THICKNESS / 2), material="carriage",
        ),
    )  # fmt: skip
    carriage = _link(LIFT_CARRIAGE_LINK, li.carriage_mass, carriage_shapes)

    bb = FORK_BACKBAR_THICKNESS
    fork_shapes = (
        Shape("tine_left", "box", (fo.length, fo.tine_width, fo.tine_thickness),
              (fo.length / 2, fo.tine_spacing / 2, fo.tine_thickness / 2), material="fork"),
        Shape("tine_right", "box", (fo.length, fo.tine_width, fo.tine_thickness),
              (fo.length / 2, -fo.tine_spacing / 2, fo.tine_thickness / 2), material="fork"),
        Shape("backbar", "box", (bb, 2 * tine_outer, 0.08), (bb / 2, 0.0, 0.04), material="fork"),
    )  # fmt: skip
    fork = _link(FORK_LINK, fo.mass, fork_shapes)

    links = (
        base,
        wheel(LEFT_WHEEL_LINK),
        wheel(RIGHT_WHEEL_LINK),
        caster(FRONT_CASTER_LINK),
        caster(REAR_CASTER_LINK),
        mast,
        carriage,
        fork,
    )
    caster_z = -r + ca.radius
    joints = (
        Joint(LEFT_WHEEL_JOINT, "continuous", BASE_LINK, LEFT_WHEEL_LINK, (0.0, wh.track_width / 2, 0.0),
              axis=(0.0, 1.0, 0.0), effort=wh.max_torque, velocity=wh.max_velocity),
        Joint(RIGHT_WHEEL_JOINT, "continuous", BASE_LINK, RIGHT_WHEEL_LINK, (0.0, -wh.track_width / 2, 0.0),
              axis=(0.0, 1.0, 0.0), effort=wh.max_torque, velocity=wh.max_velocity),
        Joint(FRONT_CASTER_JOINT, "fixed", BASE_LINK, FRONT_CASTER_LINK, (ca.offset_x, 0.0, caster_z)),
        Joint(REAR_CASTER_JOINT, "fixed", BASE_LINK, REAR_CASTER_LINK, (-ca.offset_x, 0.0, caster_z)),
        Joint(MAST_JOINT, "fixed", BASE_LINK, MAST_LINK, (ma.offset_x, 0.0, z_top)),
        Joint(LIFT_JOINT, "prismatic", MAST_LINK, LIFT_CARRIAGE_LINK, (us / 2 + CARRIAGE_GAP, 0.0, CARRIAGE_BASE_Z),
              axis=(0.0, 0.0, 1.0), lower=li.lower, upper=li.upper, effort=li.max_force, velocity=li.max_velocity),
        Joint(FORK_JOINT, "prismatic", LIFT_CARRIAGE_LINK, FORK_LINK, (pt, 0.0, 0.0),
              axis=(1.0, 0.0, 0.0), lower=fo.lower, upper=fo.upper, effort=fo.max_force, velocity=fo.max_velocity),
    )  # fmt: skip
    return RobotDescription(name=robot.id.replace("-", "_"), links=links, joints=joints)


def write_urdf(description: RobotDescription, directory: str | Path | None = None) -> Path:
    """Write the URDF atomically and return its path.

    Without ``directory`` a fresh temporary directory is used; the caller owns
    its cleanup (the physics world removes it on shutdown).
    """
    out_dir = Path(directory) if directory is not None else Path(tempfile.mkdtemp(prefix="smartfleet_urdf_"))
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / f"{description.name}.urdf"
    fd, tmp = tempfile.mkstemp(dir=out_dir, suffix=".urdf.tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(description.to_urdf())
    os.replace(tmp, target)
    return target


# --------------------------------------------------------------------------- #
# Mass properties (uniform density over collision shapes)
# --------------------------------------------------------------------------- #
def _link(name: str, mass: float, shapes: tuple[Shape, ...], com: Vec3 | None = None) -> Link:
    solids = [s for s in shapes if s.collision]
    volumes = [_volume(s) for s in solids]
    total = sum(volumes)
    if com is None:
        com = tuple(sum(v * s.xyz[k] for v, s in zip(volumes, solids)) / total for k in range(3))  # type: ignore[assignment]
    ixx = iyy = izz = ixy = ixz = iyz = 0.0
    for v, s in zip(volumes, solids):
        m = mass * v / total
        sx, sy, sz = _principal_inertia(s, m)
        dx, dy, dz = (s.xyz[0] - com[0], s.xyz[1] - com[1], s.xyz[2] - com[2])
        ixx += sx + m * (dy * dy + dz * dz)
        iyy += sy + m * (dx * dx + dz * dz)
        izz += sz + m * (dx * dx + dy * dy)
        ixy -= m * dx * dy
        ixz -= m * dx * dz
        iyz -= m * dy * dz
    return Link(name, mass, com, Inertia(ixx, iyy, izz, ixy, ixz, iyz), shapes)


def _volume(s: Shape) -> float:
    if s.kind == "box":
        return s.size[0] * s.size[1] * s.size[2]
    if s.kind == "cylinder":
        return math.pi * s.size[0] ** 2 * s.size[1]
    return 4.0 / 3.0 * math.pi * s.size[0] ** 3


def _principal_inertia(s: Shape, m: float) -> Vec3:
    if s.kind == "box":
        a, b, c = s.size
        ix, iy, iz = m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12
    elif s.kind == "cylinder":
        rad, h = s.size
        ix = iy = m * (3 * rad * rad + h * h) / 12
        iz = m * rad * rad / 2
    else:
        ix = iy = iz = 0.4 * m * s.size[0] ** 2
    roll, pitch, yaw = s.rpy
    if pitch or yaw or roll not in (0.0, _HALF_PI, -_HALF_PI):
        raise ValueError(f"shape {s.name!r}: only 0 or ±90° roll is supported for inertia computation")
    if roll:
        iy, iz = iz, iy  # a ±90° roll swaps the Y and Z principal axes
    return ix, iy, iz


def _num(value: float | None) -> str:
    if value is None:
        raise ValueError("missing numeric value in robot description")
    return format(float(value), ".9g")


def _fmt(values: tuple[float, ...]) -> str:
    return " ".join(_num(v) for v in values)


def _shape_element(parent: ET.Element, tag: str, shape: Shape) -> None:
    el = ET.SubElement(parent, tag, name=shape.name)
    ET.SubElement(el, "origin", xyz=_fmt(shape.xyz), rpy=_fmt(shape.rpy))
    geometry = ET.SubElement(el, "geometry")
    if shape.kind == "box":
        ET.SubElement(geometry, "box", size=_fmt(shape.size))
    elif shape.kind == "cylinder":
        ET.SubElement(geometry, "cylinder", radius=_num(shape.size[0]), length=_num(shape.size[1]))
    else:
        ET.SubElement(geometry, "sphere", radius=_num(shape.size[0]))
    if tag == "visual":
        ET.SubElement(el, "material", name=shape.material)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export the SmartFleet robot URDF generated from config.")
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent / "generated"), help="output directory")
    args = parser.parse_args()
    path = write_urdf(build_robot_description(), args.out)
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
