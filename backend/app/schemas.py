"""API request/response models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from simulation.config import AppConfig, Vec3, get_config
from simulation.geometry import StaticBox
from simulation.mechanisms import LiftPreset, MechanismName
from simulation.robot_model import RobotDescription
from simulation.state import WorldState

_LIMITS = get_config().robot.limits
_LIFT = get_config().robot.lift
_FORKS = get_config().robot.forks

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


class _StrictRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


class LiftTargetRequest(_StrictRequest):
    """Absolute lift (carriage) position. Outside the joint limits -> HTTP 422."""

    position: float = Field(
        ge=_LIFT.lower,
        le=_LIFT.upper,
        allow_inf_nan=False,
        description=f"Lift joint position in m, {_LIFT.lower}..{_LIFT.upper} (0 = fully lowered).",
    )


class ForkTargetRequest(_StrictRequest):
    """Absolute fork extension. Outside the joint limits -> HTTP 422."""

    position: float = Field(
        ge=_FORKS.lower,
        le=_FORKS.upper,
        allow_inf_nan=False,
        description=f"Fork extension in m, {_FORKS.lower}..{_FORKS.upper} (0 = fully retracted).",
    )


class _JogRequest(_StrictRequest):
    @field_validator("delta", check_fields=False)
    @classmethod
    def _non_zero(cls, value: float) -> float:
        if value == 0.0:
            raise ValueError("jog step must be non-zero")
        return value


class LiftJogRequest(_JogRequest):
    """Incremental lift move; the resulting target saturates at the travel limit."""

    delta: float = Field(
        ge=-_LIFT.max_jog_step,
        le=_LIFT.max_jog_step,
        allow_inf_nan=False,
        description="Change of the lift target in m (+ raises, - lowers).",
    )


class ForkJogRequest(_JogRequest):
    """Incremental fork move; the resulting target saturates at the travel limit."""

    delta: float = Field(
        ge=-_FORKS.max_jog_step,
        le=_FORKS.max_jog_step,
        allow_inf_nan=False,
        description="Change of the fork target in m (+ extends, - retracts).",
    )


class MechanismCommandResponse(BaseModel):
    accepted: bool
    mechanism: MechanismName
    target: float
    previous_target: float
    position: float  # measured when the command was applied
    clamped: bool  # jog target saturated at a travel limit
    message: str


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


class MechanismStopResponse(BaseModel):
    message: str
    stopped: list[MechanismCommandResponse]
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
    lift_presets: list[LiftPreset]


class TelemetryMessage(BaseModel):
    type: Literal["telemetry"] = "telemetry"
    snapshot: SimulationSnapshot
    events: list[EventModel]
