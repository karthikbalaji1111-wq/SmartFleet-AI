"""Simulation service: owns the physics world, its run state and the event log."""

from __future__ import annotations

import asyncio
import contextlib
import threading
import time
from collections import deque
from datetime import datetime, timezone

from simulation.config import AppConfig, get_config
from simulation.mechanisms import MECHANISMS, MechanismCommand, MechanismName
from simulation.navigation.models import NavigationTelemetry, RouteModel
from simulation.world import SimulationWorld, pybullet_available

from .schemas import (
    CommandResponse,
    EventLevel,
    EventModel,
    MechanismCommandResponse,
    NavigationPlanRequest,
    SimulationSnapshot,
    SimulationStatus,
    WheelTargets,
)

_MOTION_WORDS: dict[MechanismName, tuple[str, str]] = {"lift": ("raising", "lowering"), "forks": ("extending", "retracting")}


def _describe(cmd: MechanismCommand) -> str:
    label = "Lift" if cmd.mechanism == "lift" else "Forks"
    up, down = _MOTION_WORDS[cmd.mechanism]
    if abs(cmd.target - cmd.position) < 1e-4:
        motion = "holding"
    else:
        motion = up if cmd.target > cmd.position else down
    clamped = " (clamped at travel limit)" if cmd.clamped else ""
    return f"{label} target {_m(cmd.target)} m{clamped}: {motion} from {_m(cmd.position)} m"


def _m(value: float) -> str:
    """Metres with 3 decimals, without '-0.000' from tiny negative joint noise."""
    return f"{round(value, 3) + 0.0:.3f}"


def _response(cmd: MechanismCommand, message: str) -> MechanismCommandResponse:
    return MechanismCommandResponse(
        accepted=True,
        mechanism=cmd.mechanism,
        target=cmd.target,
        previous_target=cmd.previous_target,
        position=cmd.position,
        clamped=cmd.clamped,
        message=message,
    )


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
            self.world.pause_navigation("simulation paused")
            self.world.stop()  # request cancelled; on resume the robot brakes along its profile
            self.world.stop_mechanisms()  # lift/forks hold where they are
            self.status = "paused"
            self._last_logged_command = None
            self.events.add(
                "info", "simulation", "Simulation paused; drive request cancelled, lift/forks holding", self.world.sim_time
            )
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
            self._require_running("Drive commands")
            # Priority: e-stop > operator > autonomy. A manual drive request takes
            # over from navigation, which is paused (resume it explicitly).
            if self.world.pause_navigation("manual drive takeover"):
                self._drain_navigation_events()
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
        """Cancel the drive request (the manual-drive release). Lift and forks
        are not affected, so releasing a drive key never interrupts them."""
        with self._lock:
            self._require_ready()
            paused = self.world.pause_navigation("stop requested")
            self.world.stop()
            self._drain_navigation_events()
            if self._last_logged_command not in (None, (0.0, 0.0)):
                self.events.add("info", "command", "Stop: drive request cancelled", self.world.sim_time)
            self._last_logged_command = (0.0, 0.0)
            return "Robot stop requested" + ("; navigation paused" if paused else "")

    def emergency_stop(self) -> str:
        """Stop all robot motion: brake the chassis and hold the lift and forks."""
        with self._lock:
            self._require_ready()
            cancelled_nav = self.world.navigator.active
            self.world.cancel_navigation("emergency stop")
            self.world.stop()
            state = self.world.get_robot_state()
            moving = [name for name in MECHANISMS if getattr(state, name).state == "moving"]
            self.world.stop_mechanisms()
            self._last_logged_command = (0.0, 0.0)
            held = f"; {' and '.join(moving)} stopped" if moving else ""
            nav = "; navigation cancelled" if cancelled_nav else ""
            self.events.add(
                "warning", "command", f"E-stop: drive cancelled, lift and forks holding{held}{nav}", self.world.sim_time
            )
            self._drain_navigation_events()
            return "All robot motion stopped (drive cancelled, lift and forks holding)"

    # ------------------------------------------------------------------ #
    # Autonomous navigation
    # ------------------------------------------------------------------ #
    def plan_navigation(self, request: NavigationPlanRequest) -> tuple[str, RouteModel | None]:
        """Plan a route (allowed while stopped or paused: planning moves nothing)."""
        with self._lock:
            self._require_ready()
            try:
                destination = self.world.navigator.resolve_destination(
                    request.destination_id, request.x, request.y, request.yaw, request.snap
                )
                plan = self.world.plan_navigation(destination)
            finally:
                self._drain_navigation_events()
            snapped = (
                f" (snapped {destination.model().snap_distance:.2f} m to navigable floor)" if destination.snapped else ""
            )
            message = f"Route to {destination.label}{snapped}: {plan.length:.2f} m, {len(plan.waypoints)} waypoints"
            return message, self.world.navigator.route_model()

    def start_navigation(self) -> str:
        with self._lock:
            self._require_running("Navigation")
            try:
                self.world.start_navigation()
            finally:
                self._drain_navigation_events()
            return f"Navigating to {self.world.navigator.destination.label}"

    def pause_navigation(self) -> str:
        with self._lock:
            self._require_ready()
            if not self.world.pause_navigation("paused by operator"):
                raise InvalidStateError(f"Navigation is not running (status: {self.world.navigator.status})")
            self._drain_navigation_events()
            return "Navigation paused"

    def resume_navigation(self) -> str:
        with self._lock:
            self._require_running("Navigation")
            try:
                self.world.resume_navigation()
            finally:
                self._drain_navigation_events()
            return "Navigation resumed"

    def cancel_navigation(self) -> str:
        with self._lock:
            self._require_ready()
            if not self.world.cancel_navigation("cancelled by operator"):
                raise InvalidStateError(f"No route or navigation to cancel (status: {self.world.navigator.status})")
            self._drain_navigation_events()
            return "Navigation cancelled"

    def navigation_state(self) -> tuple[NavigationTelemetry | None, RouteModel | None]:
        with self._lock:
            if not self.ready:
                return None, None
            return self.world.navigator.telemetry(self.world.get_robot_state()), self.world.navigator.route_model()

    def route_update(self, known_version: int | None) -> tuple[int, RouteModel | None] | None:
        """The current route if its version differs from ``known_version``."""
        with self._lock:
            version = self.world.navigator.route_version
            if version == known_version:
                return None
            return version, self.world.navigator.route_model()

    def _drain_navigation_events(self) -> None:
        for level, message in self.world.navigator.drain_events():
            self.events.add(level, "navigation", message, self.world.sim_time)

    # ------------------------------------------------------------------ #
    # Lift and forks
    # ------------------------------------------------------------------ #
    def command_mechanism(self, name: MechanismName, position: float) -> MechanismCommandResponse:
        with self._lock:
            self._require_running("Lift/fork commands")
            cmd = self.world.set_mechanism_target(name, position)
            message = _describe(cmd)
            self.events.add("info", "mechanism", message, self.world.sim_time)
            return _response(cmd, message)

    def jog_mechanism(self, name: MechanismName, delta: float) -> MechanismCommandResponse:
        with self._lock:
            self._require_running("Lift/fork commands")
            cmd = self.world.jog_mechanism(name, delta)
            message = f"Jog {delta:+.3f} m. {_describe(cmd)}"
            self.events.add("warning" if cmd.clamped else "info", "mechanism", message, self.world.sim_time)
            return _response(cmd, message)

    def stop_mechanisms(self, names: tuple[MechanismName, ...] = MECHANISMS) -> list[MechanismCommandResponse]:
        """Hold the given mechanisms at their measured positions (any state)."""
        with self._lock:
            self._require_ready()
            results = []
            for name in names:
                cmd = self.world.stop_mechanism(name)
                label = "Lift" if name == "lift" else "Forks"
                message = f"{label} stopped: holding at {_m(cmd.target)} m"
                self.events.add("info", "mechanism", message, self.world.sim_time)
                results.append(_response(cmd, message))
            return results

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

    def _require_running(self, what: str) -> None:
        self._require_ready()
        if self.status != "running":
            raise InvalidStateError(f"{what} are only accepted while the simulation is running (status: {self.status})")

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
                        for level, message in self.world.drain_events():
                            self.events.add(level, "mechanism", message, self.world.sim_time)
                        self._drain_navigation_events()
                    except Exception as exc:
                        self.status = "paused"
                        self.events.add("error", "physics", f"Physics step failed, simulation paused: {exc}")
                else:
                    accumulator = 0.0
            await asyncio.sleep(max(0.0, period - (time.perf_counter() - tick_start)))
