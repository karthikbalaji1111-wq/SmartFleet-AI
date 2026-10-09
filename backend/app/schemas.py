"""API request/response models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from simulation.config import AppConfig, Vec3, get_config
from simulation.geometry import StaticBox
from simulation.robot_model import RobotDescription
from simulation.state import WorldState

_LIMITS = get_config().robot.limits

SimulationStatus = Literal["stopped", "running", "paused"]
EventLevel = Literal["info", "warning", "error"]


class VelocityCommandRequest(BaseModel):
    """Bounded manual drive request. Out-of-range, non-finite, wrongly-typed or
    unknown fields are rejected with HTTP 422 (never silently clamped)."""

    model_config = ConfigDict(extra="forbid", strict=True)

    linear: float = Field(
        ge=-_LIMITS.max_linear_velocity,
        le=_LIMITS.max_linear_velocity,
        allow_inf_nan=False,
        description="Forward velocity request in m/s (+ forward, - reverse).",
    )
    angular: float = Field(
        ge=-_LIMITS.max_angular_velocity,
        le=_LIMITS.max_angular_velocity,
        allow_inf_nan=False,
        description="Yaw-rate request in rad/s (+ counter-clockwise / left).",
    )
    duration: float = Field(
        default=_LIMITS.default_command_duration,
        ge=_LIMITS.min_command_duration,
        le=_LIMITS.max_command_duration,
        allow_inf_nan=False,
        description="Dead-man timeout in seconds of simulated time; the request is zeroed afterwards.",
    )


class EventModel(BaseModel):
    id: int
    timestamp: str
    sim_time: float | None
    level: EventLevel
    source: str
    message: str


class SimulationSnapshot(BaseModel):
    status: SimulationStatus
    ready: bool
    physics_hz: int
    timestep: float
    world: WorldState | None
    last_event_id: int


class ControlResponse(BaseModel):
    message: str
    snapshot: SimulationSnapshot


class WheelTargets(BaseModel):
    left: float
    right: float


class CommandResponse(BaseModel):
    accepted: bool
    linear: float
    angular: float
    duration: float
    wheel_targets: WheelTargets
    sim_time: float
    expires_at: float


class HealthResponse(BaseModel):
    status: Literal["ok"]
    service: str
    version: str
    timestamp: str


class ReadyResponse(BaseModel):
    ready: bool
    pybullet_available: bool
    world_initialized: bool
    detail: str | None


class WarehouseConfigResponse(BaseModel):
    """Everything the frontend needs to draw the warehouse and the robot.

    ``static_geometry`` is the exact list of boxes that exist as PyBullet
    collision bodies, and ``robot_model`` is the description the URDF was
    generated from.
    """

    config: AppConfig
    static_geometry: list[StaticBox]
    robot_model: RobotDescription
    container_initial_position: Vec3


class TelemetryMessage(BaseModel):
    type: Literal["telemetry"] = "telemetry"
    snapshot: SimulationSnapshot
    events: list[EventModel]
