"""Lift and fork mechanism commands: validation, presets and status rules.

Both mechanisms are prismatic joints of the robot URDF driven by PyBullet
position motors (force- and velocity-limited). This module holds the pure
logic; the physics lives in :mod:`simulation.world`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .config import ActuatorConfig, AppConfig, RobotConfig
from .diff_drive import CommandError
from .robot_model import FORK_JOINT, LIFT_JOINT, fork_surface_height

MechanismName = Literal["lift", "forks"]
MechanismState = Literal["holding", "moving", "blocked"]
MechanismFault = Literal["stalled", "overload", "tilt"]

MECHANISMS: tuple[MechanismName, ...] = ("lift", "forks")
MECHANISM_JOINTS: dict[MechanismName, str] = {"lift": LIFT_JOINT, "forks": FORK_JOINT}

# Protection: a mechanism short of its target is stopped and held where it is
# (state "blocked") when any of these trips:
#  - stalled:  it has (almost) stopped moving for STALL_TIME;
#  - overload: its motor has been at the force limit for OVERLOAD_TIME;
#  - tilt:     the chassis tilts beyond limits.max_handling_tilt while it moves.
STALL_SPEED = 0.01  # m/s
STALL_TIME = 0.5  # s
OVERLOAD_RATIO = 0.98  # fraction of max_force counted as saturated
OVERLOAD_TIME = 0.1  # s

# Lift presets put the fork surface this far above the shelf / stand top.
PRESET_CLEARANCE = 0.02  # m


def actuator_config(robot: RobotConfig, name: MechanismName) -> ActuatorConfig:
    if name == "lift":
        return robot.lift
    if name == "forks":
        return robot.forks
    raise CommandError(f"unknown mechanism {name!r}; expected one of {list(MECHANISMS)}")


def _check_number(label: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandError(f"{label} must be a number, got {type(value).__name__}")
    if not math.isfinite(value):
        raise CommandError(f"{label} must be finite, got {value!r}")
    return float(value)


def validate_target(name: MechanismName, position: object, cfg: ActuatorConfig) -> float:
    """Reject (never clamp) absolute targets outside the joint limits."""
    value = _check_number(f"{name} target", position)
    if not cfg.lower <= value <= cfg.upper:
        raise CommandError(
            f"{name} target {value:.3f} m is outside the limits [{cfg.lower:.2f}, {cfg.upper:.2f}] m"
        )
    return value


def validate_jog(name: MechanismName, delta: object, cfg: ActuatorConfig) -> float:
    """Incremental moves must be non-zero and no larger than ``max_jog_step``.
    The resulting target saturates at the travel limit (reported as clamped)."""
    value = _check_number(f"{name} jog step", delta)
    if value == 0.0 or abs(value) > cfg.max_jog_step:
        raise CommandError(
            f"{name} jog step {value:+.3f} m must be non-zero and within ±{cfg.max_jog_step:.2f} m"
        )
    return value


@dataclass(frozen=True)
class MechanismCommand:
    """Result of an accepted lift/fork command."""

    mechanism: MechanismName
    target: float
    previous_target: float
    position: float  # measured joint position when the command was applied
    clamped: bool = False


@dataclass(frozen=True)
class LiftPreset:
    id: str
    label: str
    position: float  # lift joint target, m
    surface_height: float  # resulting fork surface height above the floor, m


def lift_presets(config: AppConfig) -> list[LiftPreset]:
    """Lift positions aligning the fork surface with each shelf level and
    station top (plus a small clearance), within the lift's travel."""
    robot, lift = config.robot, config.robot.lift
    base = fork_surface_height(robot, 0.0)
    presets = [
        LiftPreset("home", "Home", lift.default_position, fork_surface_height(robot, lift.default_position))
    ]
    levels = sorted({level for rack_type in config.warehouse.rack_types.values() for level in rack_type.levels})
    targets = [(f"shelf-{i}", f"Shelf L{i}", level) for i, level in enumerate(levels, start=1)]
    stand_heights = sorted({s.size[2] for s in config.warehouse.stations})
    targets += [(f"stand-{i}", "Stand", h) for i, h in enumerate(stand_heights, start=1)]
    seen: set[float] = set()
    for preset_id, label, surface in targets:
        position = round(surface + PRESET_CLEARANCE - base, 4)
        if lift.lower <= position <= lift.upper and position not in seen:
            seen.add(position)
            presets.append(LiftPreset(preset_id, label, position, fork_surface_height(robot, position)))
    return presets
