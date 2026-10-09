"""SmartFleet AI backend: FastAPI app exposing the PyBullet simulation.

Run from the repository root:  uvicorn backend.app.main:app --port 8000
"""

from __future__ import annotations

import asyncio
import contextlib
from contextlib import asynccontextmanager
from datetime import datetime, timezone

from fastapi import FastAPI, Request, WebSocket
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from simulation.config import AppConfig
from simulation.diff_drive import CommandError
from simulation.geometry import container_initial_position
from simulation.world import pybullet_available

from . import __version__
from .schemas import (
    CommandResponse,
    ControlResponse,
    HealthResponse,
    ReadyResponse,
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
        description="Milestone 1: PyBullet-backed warehouse and storage robot.",
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
        message = service.stop_robot()
        return ControlResponse(message=message, snapshot=service.snapshot())

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

        async def send_loop() -> None:
            nonlocal last_event_id
            while True:
                events = service.events.since(last_event_id)
                if events:
                    last_event_id = events[-1].id
                message = TelemetryMessage(snapshot=service.snapshot(), events=events)
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
