"""Simulation service: owns the physics world, its run state and the event log."""

from __future__ import annotations

import asyncio
import contextlib
import threading
import time
from collections import deque
from datetime import datetime, timezone

from simulation.config import AppConfig, get_config
from simulation.world import SimulationWorld, pybullet_available

from .schemas import CommandResponse, EventLevel, EventModel, SimulationSnapshot, SimulationStatus, WheelTargets


class SimulationUnavailableError(RuntimeError):
    """Physics is not initialized (e.g. PyBullet missing) -> HTTP 503."""


class InvalidStateError(RuntimeError):
    """Request not allowed in the current simulation state -> HTTP 409."""


class EventLog:
    """Bounded, thread-safe log of notable simulation events."""

    def __init__(self, maxlen: int = 200) -> None:
        self._events: deque[EventModel] = deque(maxlen=maxlen)
        self._next_id = 1
        self._lock = threading.Lock()

    def add(self, level: EventLevel, source: str, message: str, sim_time: float | None = None) -> EventModel:
        with self._lock:
            event = EventModel(
                id=self._next_id,
                timestamp=datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                sim_time=sim_time,
                level=level,
                source=source,
                message=message,
            )
            self._next_id += 1
            self._events.append(event)
            return event

    def since(self, event_id: int) -> list[EventModel]:
        with self._lock:
            return [e for e in self._events if e.id > event_id]

    @property
    def last_id(self) -> int:
        with self._lock:
            return self._next_id - 1


class SimulationService:
    """Runs the fixed-step physics loop in real time and enforces the
    stopped -> running <-> paused state machine. Starts in the safe STOPPED
    state with no motion requested."""

    def __init__(self, config: AppConfig | None = None) -> None:
        self.config = config or get_config()
        self.world = SimulationWorld(self.config)
        self.events = EventLog()
        self.status: SimulationStatus = "stopped"
        self.init_error: str | None = None
        self._lock = threading.RLock()
        self._loop_task: asyncio.Task | None = None
        self._last_logged_command: tuple[float, float] | None = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    @property
    def ready(self) -> bool:
        return self.init_error is None and self.world.is_initialized

    async def startup(self) -> None:
        self.events.add("info", "server", "Backend starting; initializing PyBullet in DIRECT mode")
        try:
            with self._lock:
                self.world.initialize()
        except Exception as exc:  # report, keep serving health/config
            self.init_error = f"{type(exc).__name__}: {exc}"
            self.events.add("error", "physics", f"Physics initialization failed: {self.init_error}")
        else:
            sim = self.config.simulation
            joints = ", ".join(sorted(self.world.model.joints))
            self.events.add(
                "info",
                "physics",
                f"Physics ready: {len(self.world.static_geometry)} static collision boxes, "
                f"robot '{self.config.robot.id}' validated (joints: {joints}); "
                f"fixed timestep 1/{sim.physics_hz} s",
            )
            self.events.add("info", "simulation", "Simulation is STOPPED (safe state). Press Start to run.")
        self._loop_task = asyncio.create_task(self._run_loop(), name="smartfleet-sim-loop")

    async def shutdown(self) -> None:
        if self._loop_task is not None:
            self._loop_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._loop_task
            self._loop_task = None
        with self._lock:
            self.world.shutdown()
            self.status = "stopped"

    # ------------------------------------------------------------------ #
    # Controls
    # ------------------------------------------------------------------ #
    def start(self) -> str:
        with self._lock:
            self._require_ready()
            if self.status == "running":
                return "Simulation already running"
            previous, self.status = self.status, "running"
            message = "Simulation resumed" if previous == "paused" else "Simulation started"
            self.events.add("info", "simulation", message, self.world.sim_time)
            return message

    def pause(self) -> str:
        with self._lock:
            self._require_ready()
            if self.status == "stopped":
                raise InvalidStateError("Cannot pause: the simulation is stopped")
            if self.status == "paused":
                return "Simulation already paused"
            self.world.stop()  # request cancelled; on resume the robot brakes along its profile
            self.status = "paused"
            self._last_logged_command = None
            self.events.add("info", "simulation", "Simulation paused; drive request cancelled", self.world.sim_time)
            return "Simulation paused"

    def reset(self) -> str:
        with self._lock:
            self._require_ready()
            self.world.reset()
            self.status = "stopped"
            self._last_logged_command = None
            p = self.config.robot.start_pose
            message = f"Simulation reset: robot at start pose (x={p.x:.2f}, y={p.y:.2f}, yaw={p.yaw:.2f} rad), STOPPED"
            self.events.add("info", "simulation", message, 0.0)
            return message

    def command_velocity(self, linear: float, angular: float, duration: float) -> CommandResponse:
        with self._lock:
            self._require_ready()
            if self.status != "running":
                raise InvalidStateError(
                    f"Drive commands are only accepted while the simulation is running (status: {self.status})"
                )
            command = self.world.set_velocity_command(linear, angular, duration)
            targets = self.world.kinematics.wheel_speeds(command.linear, command.angular)
            key = (round(command.linear, 3), round(command.angular, 3))
            if key != self._last_logged_command:
                self._last_logged_command = key
                self.events.add(
                    "info",
                    "command",
                    f"Drive request v={command.linear:+.2f} m/s, w={command.angular:+.2f} rad/s",
                    self.world.sim_time,
                )
            return CommandResponse(
                accepted=True,
                linear=command.linear,
                angular=command.angular,
                duration=command.duration,
                wheel_targets=WheelTargets(left=targets.left, right=targets.right),
                sim_time=self.world.sim_time,
                expires_at=self.world.sim_time + command.duration,
            )

    def stop_robot(self) -> str:
        with self._lock:
            self._require_ready()
            self.world.stop()
            if self._last_logged_command not in (None, (0.0, 0.0)):
                self.events.add("info", "command", "Stop: drive request cancelled", self.world.sim_time)
            self._last_logged_command = (0.0, 0.0)
            return "Robot stop requested"

    def snapshot(self) -> SimulationSnapshot:
        with self._lock:
            world = self.world.get_state() if self.ready else None
            return SimulationSnapshot(
                status=self.status,
                ready=self.ready,
                physics_hz=self.config.simulation.physics_hz,
                timestep=self.config.simulation.timestep,
                world=world,
                last_event_id=self.events.last_id,
            )

    def readiness(self) -> tuple[bool, str | None]:
        if self.ready:
            return True, None
        if not pybullet_available():
            return False, self.init_error or "PyBullet is not installed"
        return False, self.init_error or "Physics world not initialized"

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _require_ready(self) -> None:
        if not self.ready:
            raise SimulationUnavailableError(self.init_error or "Physics world not initialized")

    async def _run_loop(self) -> None:
        """Advance physics in real time using a fixed-step accumulator."""
        sim = self.config.simulation
        dt, period = sim.timestep, 1.0 / sim.loop_hz
        accumulator = 0.0
        last = time.perf_counter()
        while True:
            tick_start = time.perf_counter()
            elapsed, last = tick_start - last, tick_start
            with self._lock:
                if self.status == "running" and self.ready:
                    accumulator += elapsed
                    steps = int(accumulator / dt)
                    if steps > sim.max_steps_per_tick:
                        # Fell behind (e.g. a slow reset); drop the backlog rather than spiral.
                        steps, accumulator = sim.max_steps_per_tick, 0.0
                    else:
                        accumulator -= steps * dt
                    try:
                        self.world.step(steps)
                    except Exception as exc:
                        self.status = "paused"
                        self.events.add("error", "physics", f"Physics step failed, simulation paused: {exc}")
                else:
                    accumulator = 0.0
            await asyncio.sleep(max(0.0, period - (time.perf_counter() - tick_start)))
