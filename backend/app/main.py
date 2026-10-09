"""SmartFleet AI backend: FastAPI app exposing the PyBullet simulation.

Run from the repository root:  uvicorn backend.app.main:app --port 8000
"""

from __future__ import annotations

import asyncio
import base64
import contextlib
from contextlib import asynccontextmanager
from datetime import datetime, timezone

from fastapi import FastAPI, Request, WebSocket
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from simulation.config import AppConfig
from simulation.diff_drive import CommandError
from simulation.geometry import container_initial_position
from simulation.mechanisms import lift_presets
from simulation.navigation.grid import grid_for
from simulation.navigation.navigator import MotionConflictError, NavigationError
from simulation.world import pybullet_available

from . import __version__
from .schemas import (
    CommandResponse,
    ControlResponse,
    ForkJogRequest,
    ForkTargetRequest,
    HealthResponse,
    LiftJogRequest,
    LiftTargetRequest,
    MechanismCommandResponse,
    MechanismStopResponse,
    NavigationPlanRequest,
    NavigationResponse,
    NavigationStateResponse,
    OccupancyGridResponse,
    ReadyResponse,
    RouteUpdate,
    SimulationSnapshot,
    TelemetryMessage,
    VelocityCommandRequest,
    WarehouseConfigResponse,
)
from .service import InvalidStateError, SimulationService, SimulationUnavailableError


def create_app(config: AppConfig | None = None) -> FastAPI:
    service = SimulationService(config)
    cfg = service.config

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        await service.startup()
        try:
            yield
        finally:
            await service.shutdown()

    app = FastAPI(
        title="SmartFleet AI Backend",
        version=__version__,
        description="PyBullet-backed warehouse, storage robot with lift/forks, and A* navigation.",
        lifespan=lifespan,
    )
    app.state.service = service

    # ------------------------------------------------------------------ #
    # Error mapping
    # ------------------------------------------------------------------ #
    @app.exception_handler(SimulationUnavailableError)
    async def _unavailable(_: Request, exc: SimulationUnavailableError) -> JSONResponse:
        return JSONResponse(status_code=503, content={"detail": f"Simulation unavailable: {exc}"})

    @app.exception_handler(InvalidStateError)
    async def _invalid_state(_: Request, exc: InvalidStateError) -> JSONResponse:
        service.events.add("warning", "command", f"Rejected: {exc}")
        return JSONResponse(status_code=409, content={"detail": str(exc)})

    @app.exception_handler(CommandError)
    async def _command_error(_: Request, exc: CommandError) -> JSONResponse:
        service.events.add("warning", "command", f"Rejected command: {exc}")
        return JSONResponse(status_code=422, content={"detail": str(exc)})

    @app.exception_handler(NavigationError)
    async def _navigation_error(_: Request, exc: NavigationError) -> JSONResponse:
        # Bad / unreachable destinations are invalid input; the rest conflict with current state.
        status = 422 if exc.code in ("invalid_destination", "unreachable") else 409
        service.events.add("warning", "navigation", f"Rejected: {exc}")
        return JSONResponse(status_code=status, content={"detail": str(exc), "code": exc.code, "reasons": exc.reasons})

    @app.exception_handler(MotionConflictError)
    async def _motion_conflict(_: Request, exc: MotionConflictError) -> JSONResponse:
        service.events.add("warning", "command", f"Rejected: {exc}")
        return JSONResponse(status_code=409, content={"detail": str(exc), "code": "navigation_active"})

    @app.exception_handler(RequestValidationError)
    async def _validation(request: Request, exc: RequestValidationError) -> JSONResponse:
        # Do not echo the raw input back: values such as NaN are not valid JSON
        # and would turn a clean 422 into a 500.
        errors = [{"loc": list(err.get("loc", ())), "msg": err.get("msg"), "type": err.get("type")} for err in exc.errors()]
        problems = "; ".join(f"{'.'.join(str(p) for p in e['loc'][1:]) or 'body'}: {e['msg']}" for e in errors)
        service.events.add("warning", "command", f"Rejected malformed request to {request.url.path}: {problems}")
        return JSONResponse(status_code=422, content={"detail": errors})

    # ------------------------------------------------------------------ #
    # Health / readiness / configuration
    # ------------------------------------------------------------------ #
    @app.get("/api/health", response_model=HealthResponse, tags=["system"])
    async def health() -> HealthResponse:
        """Liveness: the API process is up (physics may still be unavailable)."""
        return HealthResponse(
            status="ok",
            service="smartfleet-backend",
            version=__version__,
            timestamp=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        )

    @app.get("/api/ready", response_model=ReadyResponse, tags=["system"], responses={503: {"model": ReadyResponse}})
    async def ready() -> JSONResponse:
        """Readiness: physics world initialized and robot model validated."""
        is_ready, detail = service.readiness()
        body = ReadyResponse(
            ready=is_ready,
            pybullet_available=pybullet_available(),
            world_initialized=service.world.is_initialized,
            detail=detail,
        )
        return JSONResponse(status_code=200 if is_ready else 503, content=body.model_dump())

    @app.get("/api/config/warehouse", response_model=WarehouseConfigResponse, tags=["config"])
    async def warehouse_config() -> WarehouseConfigResponse:
        """Canonical warehouse + robot configuration with the derived collision
        geometry and robot model the physics engine uses."""
        return WarehouseConfigResponse(
            config=cfg,
            static_geometry=service.world.static_geometry,
            robot_model=service.world.description,
            container_initial_position=container_initial_position(cfg.warehouse),
            lift_presets=lift_presets(cfg),
        )

    # ------------------------------------------------------------------ #
    # Simulation control
    # ------------------------------------------------------------------ #
    @app.get("/api/simulation/state", response_model=SimulationSnapshot, tags=["simulation"])
    async def simulation_state() -> SimulationSnapshot:
        return service.snapshot()

    @app.post("/api/simulation/start", response_model=ControlResponse, tags=["simulation"])
    async def start() -> ControlResponse:
        message = service.start()
        return ControlResponse(message=message, snapshot=service.snapshot())

    @app.post("/api/simulation/pause", response_model=ControlResponse, tags=["simulation"])
    async def pause() -> ControlResponse:
        message = service.pause()
        return ControlResponse(message=message, snapshot=service.snapshot())

    @app.post("/api/simulation/reset", response_model=ControlResponse, tags=["simulation"])
    async def reset() -> ControlResponse:
        message = service.reset()
        return ControlResponse(message=message, snapshot=service.snapshot())

    # ------------------------------------------------------------------ #
    # Manual robot commands (temporary verification controls)
    # ------------------------------------------------------------------ #
    @app.post("/api/robot/velocity", response_model=CommandResponse, tags=["robot"])
    async def robot_velocity(command: VelocityCommandRequest) -> CommandResponse:
        """Bounded differential-drive request with a dead-man timeout."""
        return service.command_velocity(command.linear, command.angular, command.duration)

    @app.post("/api/robot/stop", response_model=ControlResponse, tags=["robot"])
    async def robot_stop() -> ControlResponse:
        """Cancel the drive request and brake (lift and forks are unaffected)."""
        message = service.stop_robot()
        return ControlResponse(message=message, snapshot=service.snapshot())

    @app.post("/api/robot/estop", response_model=ControlResponse, tags=["robot"])
    async def robot_estop() -> ControlResponse:
        """Stop all robot motion: brake the chassis and hold the lift and forks. Allowed in any state."""
        message = service.emergency_stop()
        return ControlResponse(message=message, snapshot=service.snapshot())

    # ------------------------------------------------------------------ #
    # Lift and forks (PyBullet position motors on prismatic joints)
    # ------------------------------------------------------------------ #
    @app.post("/api/robot/lift/target", response_model=MechanismCommandResponse, tags=["mechanisms"])
    async def lift_target(request: LiftTargetRequest) -> MechanismCommandResponse:
        """Raise or lower the lift carriage to an absolute position (m)."""
        return service.command_mechanism("lift", request.position)

    @app.post("/api/robot/lift/jog", response_model=MechanismCommandResponse, tags=["mechanisms"])
    async def lift_jog(request: LiftJogRequest) -> MechanismCommandResponse:
        """Move the lift target by a bounded increment (saturates at the travel limit)."""
        return service.jog_mechanism("lift", request.delta)

    @app.post("/api/robot/lift/stop", response_model=MechanismStopResponse, tags=["mechanisms"])
    async def lift_stop() -> MechanismStopResponse:
        """Hold the lift at its current measured position."""
        stopped = service.stop_mechanisms(("lift",))
        return MechanismStopResponse(message=stopped[0].message, stopped=stopped, snapshot=service.snapshot())

    @app.post("/api/robot/forks/target", response_model=MechanismCommandResponse, tags=["mechanisms"])
    async def forks_target(request: ForkTargetRequest) -> MechanismCommandResponse:
        """Extend or retract the forks to an absolute position (m)."""
        return service.command_mechanism("forks", request.position)

    @app.post("/api/robot/forks/jog", response_model=MechanismCommandResponse, tags=["mechanisms"])
    async def forks_jog(request: ForkJogRequest) -> MechanismCommandResponse:
        """Move the fork target by a bounded increment (saturates at the travel limit)."""
        return service.jog_mechanism("forks", request.delta)

    @app.post("/api/robot/forks/stop", response_model=MechanismStopResponse, tags=["mechanisms"])
    async def forks_stop() -> MechanismStopResponse:
        """Hold the forks at their current measured position."""
        stopped = service.stop_mechanisms(("forks",))
        return MechanismStopResponse(message=stopped[0].message, stopped=stopped, snapshot=service.snapshot())

    @app.post("/api/robot/mechanisms/stop", response_model=MechanismStopResponse, tags=["mechanisms"])
    async def mechanisms_stop() -> MechanismStopResponse:
        """Hold both the lift and the forks at their current measured positions."""
        stopped = service.stop_mechanisms()
        return MechanismStopResponse(
            message="Lift and forks holding", stopped=stopped, snapshot=service.snapshot()
        )

    # ------------------------------------------------------------------ #
    # Autonomous navigation (A* on the static map + path following)
    # ------------------------------------------------------------------ #
    def _navigation_response(message: str) -> NavigationResponse:
        navigation, route = service.navigation_state()
        return NavigationResponse(message=message, navigation=navigation, route=route)

    @app.get("/api/navigation", response_model=NavigationStateResponse, tags=["navigation"])
    async def navigation_state() -> NavigationStateResponse:
        """Navigation status, the active route and the configured destinations."""
        navigation, route = service.navigation_state()
        return NavigationStateResponse(
            navigation=navigation, route=route, destinations=list(cfg.navigation.destinations)
        )

    @app.get("/api/navigation/grid", response_model=OccupancyGridResponse, tags=["navigation"])
    async def navigation_grid() -> OccupancyGridResponse:
        """The static occupancy grid the planner uses (derived from the collision geometry)."""
        grid = grid_for(cfg)
        return OccupancyGridResponse(
            width=grid.width, height=grid.height, resolution=grid.resolution, origin=grid.origin,
            inflation_radius=grid.inflation_radius, robot_radius=grid.robot_radius,
            encoding="base64-u8-row-major", cells=base64.b64encode(bytes(grid.cells)).decode("ascii"),
        )  # fmt: skip

    @app.post("/api/navigation/plan", response_model=NavigationResponse, tags=["navigation"])
    async def navigation_plan(request: NavigationPlanRequest) -> NavigationResponse:
        """A* route from the robot's measured position to a destination id or floor point."""
        message, _ = service.plan_navigation(request)
        return _navigation_response(message)

    @app.post("/api/navigation/start", response_model=NavigationResponse, tags=["navigation"])
    async def navigation_start() -> NavigationResponse:
        """Follow the planned route (requires a running simulation and a safe travel configuration)."""
        return _navigation_response(service.start_navigation())

    @app.post("/api/navigation/pause", response_model=NavigationResponse, tags=["navigation"])
    async def navigation_pause() -> NavigationResponse:
        return _navigation_response(service.pause_navigation())

    @app.post("/api/navigation/resume", response_model=NavigationResponse, tags=["navigation"])
    async def navigation_resume() -> NavigationResponse:
        """Continue a paused route if the robot is still on it and travel is safe."""
        return _navigation_response(service.resume_navigation())

    @app.post("/api/navigation/cancel", response_model=NavigationResponse, tags=["navigation"])
    async def navigation_cancel() -> NavigationResponse:
        return _navigation_response(service.cancel_navigation())

    # ------------------------------------------------------------------ #
    # Live telemetry
    # ------------------------------------------------------------------ #
    @app.websocket("/ws/telemetry")
    async def telemetry(ws: WebSocket) -> None:
        """Pushes simulation snapshots at ``simulation.telemetry_hz`` plus any
        new events. Server -> client only; commands go through REST."""
        await ws.accept()
        period = 1.0 / cfg.simulation.telemetry_hz
        last_event_id = 0
        route_version: int | None = None  # the full route is sent only when it changes

        async def send_loop() -> None:
            nonlocal last_event_id, route_version
            while True:
                events = service.events.since(last_event_id)
                if events:
                    last_event_id = events[-1].id
                update = service.route_update(route_version) if service.ready else None
                route = None
                if update is not None:
                    route_version = update[0]
                    route = RouteUpdate(version=update[0], route=update[1])
                message = TelemetryMessage(snapshot=service.snapshot(), events=events, route_update=route)
                await ws.send_text(message.model_dump_json())
                await asyncio.sleep(period)

        async def receive_loop() -> None:
            while True:
                incoming = await ws.receive()
                if incoming["type"] == "websocket.disconnect":
                    return

        tasks = [asyncio.create_task(send_loop()), asyncio.create_task(receive_loop())]
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            for task in tasks:
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task

    return app


app = create_app()
