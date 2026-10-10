"""PyBullet physics world for SmartFleet AI (DIRECT mode, fixed timestep).

The world owns one PyBullet client. Robot pose, velocities and joint positions
are always read back from PyBullet; nothing here integrates a competing state.
Chassis motion is produced only by wheel velocity motors acting through contact
friction; the lift and forks move only through their joint position motors.
"""

from __future__ import annotations

import math
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path

from .config import AppConfig, get_config
from .diff_drive import DiffDriveKinematics, VelocityCommand, WheelSpeeds, ramp, validate_command
from .geometry import StaticBox, build_static_geometry, container_initial_position
from .mechanisms import (
    MECHANISM_JOINTS,
    MECHANISMS,
    OVERLOAD_RATIO,
    OVERLOAD_TIME,
    STALL_SPEED,
    STALL_TIME,
    MechanismCommand,
    MechanismFault,
    MechanismName,
    actuator_config,
    validate_jog,
    validate_target,
)
from .navigation.grid import grid_for
from .navigation.navigator import Destination, MotionConflictError, Navigator
from .navigation.planner import PlanResult
from .robot_model import (
    EXPECTED_JOINTS,
    FORK_JOINT,
    FORK_LINK,
    FRONT_CASTER_LINK,
    LEFT_WHEEL_JOINT,
    LIFT_CARRIAGE_LINK,
    LIFT_JOINT,
    REAR_CASTER_LINK,
    RIGHT_WHEEL_JOINT,
    RobotDescription,
    build_robot_description,
    write_urdf,
)
from .state import (
    BodyState,
    CommandTelemetry,
    JointTelemetry,
    MechanismTelemetry,
    PoseTelemetry,
    RobotState,
    VelocityTelemetry,
    WheelsTelemetry,
    WorldState,
)

try:  # PyBullet is optional at import time so config/API code works without it.
    import pybullet as pb
except ImportError:  # pragma: no cover - depends on the environment
    pb = None

PYBULLET_INSTALL_HINT = (
    "PyBullet is not installed in this Python environment. On Windows there are no PyPI wheels: "
    "either install 'Desktop development with C++' (Visual Studio Build Tools) and run "
    "'pip install pybullet', or use Miniforge and run 'conda install -c conda-forge pybullet'. "
    "See README.md."
)

SPAWN_CLEARANCE = 0.002  # m above the floor when (re)spawning bodies
# Parking brake: with no motion requested, velocity motors at 0 rad/s still let
# loaded wheels creep (the robot slowly spins in place). Once both wheels are
# below this speed they switch to position-hold at their current angle.
BRAKE_ENGAGE_SPEED = 0.05  # rad/s
_ZERO_COMMAND = VelocityCommand(0.0, 0.0, 0.0)


class PyBulletUnavailableError(RuntimeError):
    pass


class RobotModelError(RuntimeError):
    pass


class WorldNotInitializedError(RuntimeError):
    pass


def pybullet_available() -> bool:
    return pb is not None


@dataclass(frozen=True)
class RobotModelIndex:
    """Validated name -> index maps for a loaded robot body (base link = -1)."""

    joints: dict[str, int]
    links: dict[str, int]


def index_robot_model(client: int, body: int, description: RobotDescription) -> RobotModelIndex:
    """Check a loaded body's links/joints against ``description`` and the
    canonical names, returning index maps. Raises :class:`RobotModelError`."""
    type_codes = {"continuous": pb.JOINT_REVOLUTE, "prismatic": pb.JOINT_PRISMATIC, "fixed": pb.JOINT_FIXED}
    errors: list[str] = []

    base_name = pb.getBodyInfo(body, physicsClientId=client)[0].decode("utf-8")
    if base_name != description.base_link:
        errors.append(f"base link is {base_name!r}, expected {description.base_link!r}")

    found: dict[str, tuple] = {}
    link_names: dict[int, str] = {-1: base_name}
    for i in range(pb.getNumJoints(body, physicsClientId=client)):
        info = pb.getJointInfo(body, i, physicsClientId=client)
        name, child = info[1].decode("utf-8"), info[12].decode("utf-8")
        found[name] = (i, info[2], child, info[8], info[9], info[16])
        link_names[i] = child

    expected = {j.name: (j.type, j.child, j.parent, j.lower, j.upper) for j in description.joints}
    for name, (jtype, child) in EXPECTED_JOINTS.items():
        if name not in expected:
            errors.append(f"robot description lacks canonical joint {name!r}")
        elif expected[name][:2] != (jtype, child):
            errors.append(f"description joint {name!r} is {expected[name][:2]}, canonical is {(jtype, child)}")

    for name, (jtype, child, parent, lower, upper) in expected.items():
        if name not in found:
            errors.append(f"joint {name!r} missing from loaded model")
            continue
        _, code, loaded_child, lo, hi, parent_index = found[name]
        if code != type_codes[jtype]:
            errors.append(f"joint {name!r} has PyBullet type {code}, expected {jtype}")
        if loaded_child != child:
            errors.append(f"joint {name!r} drives link {loaded_child!r}, expected {child!r}")
        if link_names.get(parent_index) != parent:
            errors.append(f"joint {name!r} parent is {link_names.get(parent_index)!r}, expected {parent!r}")
        if jtype == "prismatic" and (abs(lo - lower) > 1e-6 or abs(hi - upper) > 1e-6):
            errors.append(f"joint {name!r} limits [{lo}, {hi}] differ from [{lower}, {upper}]")
    for name in sorted(set(found) - set(expected)):
        errors.append(f"unexpected joint {name!r} in loaded model")

    if errors:
        raise RobotModelError("robot model validation failed: " + "; ".join(errors))
    return RobotModelIndex(
        joints={name: v[0] for name, v in found.items()},
        links={base_name: -1, **{v[2]: v[0] for v in found.values()}},
    )


class SimulationWorld:
    """One PyBullet DIRECT-mode world: floor, walls, racks, stations, robot, container."""

    def __init__(self, config: AppConfig | None = None) -> None:
        self.config = config or get_config()
        self.description = build_robot_description(self.config.robot)
        self.kinematics = DiffDriveKinematics.from_config(self.config.robot)
        self.static_geometry: list[StaticBox] = build_static_geometry(self.config.warehouse)
        self.timestep = self.config.simulation.timestep
        # Autonomous navigation on the static map (shared, immutable occupancy grid).
        self.navigator = Navigator(self.config, grid_for(self.config))
        self._nav_steps = max(1, round(self.config.simulation.physics_hz / self.config.navigation.controller.control_hz))
        self._client: int | None = None
        self._urdf_dir: Path | None = None
        self._urdf_path: Path | None = None
        self._clear_runtime_state()

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    @property
    def client_id(self) -> int | None:
        return self._client

    @property
    def is_initialized(self) -> bool:
        return self._client is not None and pb is not None and bool(pb.isConnected(physicsClientId=self._client))

    def initialize(self) -> None:
        """Connect in DIRECT mode and build the world. Idempotent."""
        if pb is None:
            raise PyBulletUnavailableError(PYBULLET_INSTALL_HINT)
        if self.is_initialized:
            return
        client = pb.connect(pb.DIRECT)
        if client < 0:
            raise RuntimeError("failed to connect to PyBullet in DIRECT mode")
        self._client = client
        try:
            self._urdf_dir = Path(tempfile.mkdtemp(prefix="smartfleet_urdf_"))
            self._urdf_path = write_urdf(self.description, self._urdf_dir)
            self._build_world()
        except Exception:
            self.shutdown()
            raise

    def reset(self) -> None:
        """Rebuild the world from config: robot back at its start pose, at rest."""
        self._require_initialized()
        self._build_world()

    def shutdown(self) -> None:
        """Disconnect and release resources. Safe to call repeatedly."""
        if self._client is not None:
            try:
                if pb is not None and pb.isConnected(physicsClientId=self._client):
                    pb.disconnect(physicsClientId=self._client)
            finally:
                self._client = None
        if self._urdf_dir is not None:
            shutil.rmtree(self._urdf_dir, ignore_errors=True)
            self._urdf_dir = None
            self._urdf_path = None
        self._clear_runtime_state()

    def __enter__(self) -> SimulationWorld:
        self.initialize()
        return self

    def __exit__(self, *exc: object) -> None:
        self.shutdown()

    # ------------------------------------------------------------------ #
    # Stepping and commands
    # ------------------------------------------------------------------ #
    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def sim_time(self) -> float:
        return self._step_count * self.timestep

    def step(self, n: int = 1) -> None:
        """Advance the physics by ``n`` fixed timesteps."""
        self._require_initialized()
        if n < 0:
            raise ValueError("step count must be non-negative")
        for _ in range(n):
            if self.navigator.active and self._step_count % self._nav_steps == 0:
                self._navigation_tick()
            self._apply_drive()
            pb.stepSimulation(physicsClientId=self._client)
            self._step_count += 1
            self._monitor_mechanisms()

    def drain_events(self) -> list[tuple[str, str]]:
        """Return and clear (level, message) events raised during stepping,
        e.g. a mechanism that was blocked and stopped."""
        events, self._events = self._events, []
        return events

    def set_velocity_command(self, linear: float, angular: float, duration: float | None = None) -> VelocityCommand:
        """Request a body twist for ``duration`` s of sim time (dead-man timeout).

        Out-of-bounds or non-finite values raise :class:`CommandError`.
        """
        self._require_initialized()
        limits = self.config.robot.limits
        command = validate_command(
            linear, angular, limits.default_command_duration if duration is None else duration, limits
        )
        self._target = command
        self._command_expires_at = self.sim_time + command.duration
        return command

    def stop(self, immediate: bool = False) -> None:
        """Cancel the velocity request. By default the robot brakes along the
        configured deceleration profile (no wheel slip). ``immediate`` zeroes
        the motor targets at once, so the wheels brake at full torque."""
        self._target = _ZERO_COMMAND
        self._command_expires_at = self.sim_time
        if immediate:
            self._linear_cmd = 0.0
            self._angular_cmd = 0.0

    # ------------------------------------------------------------------ #
    # Lift and fork mechanisms (PyBullet position motors on prismatic joints)
    # ------------------------------------------------------------------ #
    def mechanism_position(self, name: MechanismName) -> float:
        """Measured joint position of a mechanism, m."""
        self._require_initialized()
        joint = self.model.joints[MECHANISM_JOINTS[name]]
        return pb.getJointState(self.robot_id, joint, physicsClientId=self._client)[0]

    def set_mechanism_target(self, name: MechanismName, position: float) -> MechanismCommand:
        """Drive a mechanism to an absolute joint position. Targets outside the
        configured limits raise :class:`CommandError` (never clamped)."""
        self._require_initialized()
        self._forbid_during_navigation(name)
        target = validate_target(name, position, actuator_config(self.config.robot, name))
        return self._command_mechanism(name, target)

    def jog_mechanism(self, name: MechanismName, delta: float) -> MechanismCommand:
        """Move a mechanism's target by ``delta``; the result saturates at the
        travel limit (``clamped`` in the result)."""
        self._require_initialized()
        self._forbid_during_navigation(name)
        cfg = actuator_config(self.config.robot, name)
        step = validate_jog(name, delta, cfg)
        wanted = self._mech_targets[name] + step
        target = min(cfg.upper, max(cfg.lower, wanted))
        return self._command_mechanism(name, target, clamped=not math.isclose(target, wanted, abs_tol=1e-9))

    def stop_mechanism(self, name: MechanismName) -> MechanismCommand:
        """Hold a mechanism where it is now (its target becomes the measured position)."""
        self._require_initialized()
        cfg = actuator_config(self.config.robot, name)
        here = min(cfg.upper, max(cfg.lower, self.mechanism_position(name)))
        return self._command_mechanism(name, here)

    def stop_mechanisms(self) -> list[MechanismCommand]:
        return [self.stop_mechanism(name) for name in MECHANISMS]

    # ------------------------------------------------------------------ #
    # Autonomous navigation (A* on the static map + path following)
    # ------------------------------------------------------------------ #
    def plan_navigation(self, destination: Destination) -> PlanResult:
        """Plan a route from the robot's measured position (raises NavigationError)."""
        self._require_initialized()
        x, y, _ = self.get_robot_state().pose.position
        return self.navigator.plan_route((x, y), destination)

    def start_navigation(self) -> None:
        self._require_initialized()
        self.navigator.start(self.get_robot_state())

    def pause_navigation(self, reason: str = "paused by operator") -> bool:
        paused = self.navigator.pause(reason)
        if paused:
            self.stop()
        return paused

    def resume_navigation(self) -> None:
        self._require_initialized()
        self.navigator.resume(self.get_robot_state())

    def cancel_navigation(self, reason: str = "cancelled by operator") -> bool:
        was_active = self.navigator.active
        cancelled = self.navigator.cancel(reason)
        if was_active:
            self.stop()
        return cancelled

    # ------------------------------------------------------------------ #
    # State (always read from PyBullet)
    # ------------------------------------------------------------------ #
    def get_robot_state(self) -> RobotState:
        self._require_initialized()
        c, body, joints = self._client, self.robot_id, self.model.joints
        pos, orn = pb.getBasePositionAndOrientation(body, physicsClientId=c)
        lin, ang = pb.getBaseVelocity(body, physicsClientId=c)
        roll, pitch, yaw = pb.getEulerFromQuaternion(orn)
        cy, sy = math.cos(yaw), math.sin(yaw)
        left, right, lift, fork = pb.getJointStates(
            body,
            [joints[LEFT_WHEEL_JOINT], joints[RIGHT_WHEEL_JOINT], joints[LIFT_JOINT], joints[FORK_JOINT]],
            physicsClientId=c,
        )
        fork_link = pb.getLinkState(
            body, self.model.links[FORK_LINK], computeForwardKinematics=True, physicsClientId=c
        )
        active = self._command_expires_at > self.sim_time
        return RobotState(
            id=self.config.robot.id,
            pose=PoseTelemetry(
                position=tuple(pos),
                orientation=tuple(orn),
                heading=yaw,
                heading_deg=math.degrees(yaw),
                roll=roll,
                pitch=pitch,
            ),
            velocity=VelocityTelemetry(
                forward=lin[0] * cy + lin[1] * sy,
                lateral=-lin[0] * sy + lin[1] * cy,
                yaw_rate=ang[2],
                linear_world=tuple(lin),
                angular_world=tuple(ang),
            ),
            wheels=WheelsTelemetry(
                left=JointTelemetry(position=left[0], velocity=left[1], applied_effort=left[3]),
                right=JointTelemetry(position=right[0], velocity=right[1], applied_effort=right[3]),
                target_left=self._wheel_targets.left,
                target_right=self._wheel_targets.right,
                brake_engaged=self._brake_engaged,
            ),
            lift=self._mechanism_telemetry("lift", lift),
            forks=self._mechanism_telemetry("forks", fork),
            # Fork link frame sits at the tines' underside; add the tine thickness.
            fork_surface_height=fork_link[4][2] + self.config.robot.forks.tine_thickness,
            command=CommandTelemetry(
                active=active,
                target_linear=self._target.linear if active else 0.0,
                target_angular=self._target.angular if active else 0.0,
                linear=self._linear_cmd,
                angular=self._angular_cmd,
                remaining=max(0.0, self._command_expires_at - self.sim_time),
            ),
        )  # fmt: skip


    def attach_container(self, container_id: str) -> None:
        self._require_initialized()
        if container_id in self._cargo_constraints:
            return
            
        c_body = self.container_ids[container_id]
        
        c_pos, c_orn = pb.getBasePositionAndOrientation(c_body, physicsClientId=self._client)
        f_state = pb.getLinkState(self.robot_id, self.model.links[FORK_LINK], physicsClientId=self._client)
        f_pos, f_orn = f_state[0], f_state[1]
        
        inv_f_pos, inv_f_orn = pb.invertTransform(f_pos, f_orn)
        rel_pos, rel_orn = pb.multiplyTransforms(inv_f_pos, inv_f_orn, c_pos, c_orn)

        c_config = next((c for c in self.config.warehouse.containers if c.id == container_id), None)
        if c_config:
            ideal_x = self.config.robot.forks.length / 2
            ideal_z = c_config.size[2] / 2 + self.config.robot.forks.tine_thickness
            rel_pos = (ideal_x, 0.0, ideal_z)
            rel_orn = (0.0, 0.0, 0.0, 1.0)
            
        constraint_id = pb.createConstraint(

            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.model.links[FORK_LINK],
            childBodyUniqueId=c_body,
            childLinkIndex=-1,
            jointType=pb.JOINT_FIXED,
            jointAxis=(0, 0, 0),
            parentFramePosition=rel_pos,
            childFramePosition=(0, 0, 0),
            parentFrameOrientation=rel_orn,
            childFrameOrientation=(0, 0, 0, 1),
            physicsClientId=self._client,
        )
        pb.changeConstraint(constraint_id, maxForce=10000, physicsClientId=self._client)
        self._cargo_constraints[container_id] = constraint_id

    def detach_container(self, container_id: str) -> None:
        self._require_initialized()
        if container_id in self._cargo_constraints:
            constraint_id = self._cargo_constraints.pop(container_id)
            pb.removeConstraint(constraint_id, physicsClientId=self._client)

    def get_container_states(self) -> list[BodyState]:
        self._require_initialized()
        states = []
        for c in self.config.warehouse.containers:
            cid = self.container_ids[c.id]
            pos, orn = pb.getBasePositionAndOrientation(cid, physicsClientId=self._client)
            states.append(BodyState(id=c.id, position=tuple(pos), orientation=tuple(orn)))
        return states
        return BodyState(id=self.config.warehouse.container.id, position=tuple(pos), orientation=tuple(orn))

    def get_state(self) -> WorldState:
        robot = self.get_robot_state()
        return WorldState(
            sim_time=self.sim_time,
            step=self._step_count,
            robot=robot,
            containers=self.get_container_states(),
            navigation=self.navigator.telemetry(robot),
        )

    def static_body_aabb(self, box_id: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """World AABB PyBullet reports for a static box (used to verify geometry)."""
        self._require_initialized()
        lo, hi = pb.getAABB(self.static_body_ids[box_id], physicsClientId=self._client)
        return tuple(lo), tuple(hi)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _require_initialized(self) -> None:
        if not self.is_initialized:
            raise WorldNotInitializedError("simulation world is not initialized")

    def _clear_runtime_state(self) -> None:
        self.robot_id = -1
        self.floor_id = -1
        self.container_ids: dict[str, int] = {}
        self.container_locations: dict[str, str] = {}
        self._cargo_constraints: dict[str, int] = {}
        self.static_body_ids: dict[str, int] = {}
        self.model: RobotModelIndex | None = None
        self._step_count = 0
        self._target = _ZERO_COMMAND
        self._command_expires_at = 0.0
        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._wheel_targets = WheelSpeeds(0.0, 0.0)
        self._brake_engaged = False
        self._mech_targets: dict[MechanismName, float] = {
            name: actuator_config(self.config.robot, name).default_position for name in MECHANISMS
        }
        self._mech_stall_time: dict[MechanismName, float] = {name: 0.0 for name in MECHANISMS}
        self._mech_overload_time: dict[MechanismName, float] = {name: 0.0 for name in MECHANISMS}
        self._mech_fault: dict[MechanismName, MechanismFault | None] = {name: None for name in MECHANISMS}
        self._events: list[tuple[str, str]] = []

    def _build_world(self) -> None:
        c, sim, robot = self._client, self.config.simulation, self.config.robot
        pb.resetSimulation(physicsClientId=c)
        self._clear_runtime_state()
        self.navigator.reset()
        pb.setGravity(0.0, 0.0, -sim.gravity, physicsClientId=c)
        pb.setPhysicsEngineParameter(
            fixedTimeStep=sim.timestep,
            numSolverIterations=sim.solver_iterations,
            numSubSteps=0,
            deterministicOverlappingPairs=1,
            physicsClientId=c,
        )

        plane = pb.createCollisionShape(pb.GEOM_PLANE, physicsClientId=c)
        self.floor_id = pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=plane, physicsClientId=c)
        pb.changeDynamics(self.floor_id, -1, lateralFriction=sim.floor_friction, physicsClientId=c)

        shapes: dict[tuple[float, float, float], int] = {}
        for box in self.static_geometry:
            if box.size not in shapes:
                shapes[box.size] = pb.createCollisionShape(
                    pb.GEOM_BOX, halfExtents=box.half_extents, physicsClientId=c
                )
            self.static_body_ids[box.id] = pb.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=shapes[box.size], basePosition=box.center, physicsClientId=c
            )

        start = robot.start_pose
        self.robot_id = pb.loadURDF(
            str(self._urdf_path),
            basePosition=(start.x, start.y, robot.wheels.radius + SPAWN_CLEARANCE),
            baseOrientation=pb.getQuaternionFromEuler((0.0, 0.0, start.yaw)),
            useFixedBase=False,
            flags=pb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=c,
        )
        self.model = index_robot_model(c, self.robot_id, self.description)
        self._configure_robot()


        self.container_ids = {}
        self.container_locations = {}
        for cont in self.config.warehouse.containers:
            self.container_locations[cont.id] = cont.location
            cx, cy, cz = container_initial_position(self.config.warehouse, cont)
            cshape = pb.createCollisionShape(
                pb.GEOM_BOX, halfExtents=[s / 2 for s in cont.size], physicsClientId=c
            )
            cid = pb.createMultiBody(
                baseMass=cont.mass,
                baseCollisionShapeIndex=cshape,
                basePosition=(cx, cy, cz + SPAWN_CLEARANCE),
                physicsClientId=c,
            )
            pb.changeDynamics(cid, -1, lateralFriction=0.8, physicsClientId=c)
            self.container_ids[cont.id] = cid

        # Let contacts settle with all motors holding; the clock starts afterwards.

        # Disable collisions between the forks and containers so the forks can slide under/through them
        for cid in self.container_ids.values():
            pb.setCollisionFilterPair(self.robot_id, cid, self.model.links[FORK_LINK], -1, 0, physicsClientId=c)
            pb.setCollisionFilterPair(self.robot_id, cid, self.model.links[LIFT_CARRIAGE_LINK], -1, 0, physicsClientId=c)

        for _ in range(sim.settle_steps):
            self._apply_drive()
            pb.stepSimulation(physicsClientId=c)
        self._step_count = 0

    def _configure_robot(self) -> None:
        c, body, robot = self._client, self.robot_id, self.config.robot
        joints, links = self.model.joints, self.model.links
        pb.changeDynamics(body, -1, linearDamping=0.0, angularDamping=0.0, physicsClientId=c)
        for name in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT):
            pb.changeDynamics(
                body, joints[name], lateralFriction=robot.wheels.lateral_friction,
                spinningFriction=0.0, rollingFriction=0.0, physicsClientId=c,
            )  # fmt: skip
            pb.setJointMotorControl2(
                body, joints[name], pb.VELOCITY_CONTROL, targetVelocity=0.0,
                force=robot.wheels.max_torque, physicsClientId=c,
            )  # fmt: skip
        # Casters are ideal omni-directional supports: frictionless contact.
        for name in (FRONT_CASTER_LINK, REAR_CASTER_LINK):
            pb.changeDynamics(
                body, links[name], lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0,
                physicsClientId=c,
            )  # fmt: skip

        # Lift and forks start at their default positions (placed once while the
        # world is built); from then on only their position motors move them.
        for name in MECHANISMS:
            pb.resetJointState(body, joints[MECHANISM_JOINTS[name]], self._mech_targets[name], physicsClientId=c)
            self._apply_mechanism_motor(name)

    def _forbid_during_navigation(self, name: MechanismName) -> None:
        if self.navigator.active:
            raise MotionConflictError(
                f"{name} commands are blocked while the robot is navigating; pause or cancel navigation first"
            )

    def _detect_dynamic_obstacles(self) -> set[int]:
        obstacles = set()
        state = self.get_robot_state()
        x, y, _ = state.pose.position
        yaw = state.pose.heading
        
        num_rays = 5
        spread_angle = 1.047 # 60 degrees
        reach = 2.5
        
        ray_from = []
        ray_to = []
        
        start_z = 0.2
        for i in range(num_rays):
            angle = yaw + (i - (num_rays - 1) / 2.0) * (spread_angle / max(1, num_rays - 1))
            offset = 0.45
            ray_from.append([x + offset * __import__('math').cos(yaw), y + offset * __import__('math').sin(yaw), start_z])
            ray_to.append([x + reach * 3.14159/3.14159 * __import__('math').cos(angle), y + reach * __import__('math').sin(angle), start_z])
            
        results = pb.rayTestBatch(ray_from, ray_to, physicsClientId=self._client)
        
        detected_ids = set()
        static_ids = set(self.static_body_ids.values())
        cargo_ids = {self.container_ids[c] for c in self._cargo_constraints}
        
        for res in results:
            hit_object_id = res[0]
            if hit_object_id >= 0:
                if hit_object_id in static_ids or hit_object_id == self.robot_id or hit_object_id in cargo_ids:
                    continue
                detected_ids.add(hit_object_id)
                
        if not detected_ids:
            return obstacles
        
        print("Detected dynamic object IDs:", detected_ids)
            
        grid = self.navigator.grid
        inflation = grid.inflation_radius + grid.preferred_clearance
        r_cells = __import__('math').ceil(inflation / grid.resolution)
        
        for obj_id in detected_ids:
            aabb = pb.getAABB(obj_id, physicsClientId=self._client)
            min_pos, max_pos = aabb[0], aabb[1]
            
            # Convert to grid coordinates manually to avoid out-of-bounds returning None
            min_cx = int((min_pos[0] - grid.origin[0]) / grid.resolution)
            min_cy = int((min_pos[1] - grid.origin[1]) / grid.resolution)
            max_cx = int((max_pos[0] - grid.origin[0]) / grid.resolution)
            max_cy = int((max_pos[1] - grid.origin[1]) / grid.resolution)
            
            min_cx, max_cx = min(min_cx, max_cx), max(min_cx, max_cx)
            min_cy, max_cy = min(min_cy, max_cy), max(min_cy, max_cy)
            
            for iy in range(min_cy - r_cells, max_cy + r_cells + 1):
                for ix in range(min_cx - r_cells, max_cx + r_cells + 1):
                    if grid.in_bounds(ix, iy):
                            # Distance from cell center to AABB
                            wx, wy = grid.cell_to_world(ix, iy)
                            dx = max(min_pos[0] - wx, 0.0, wx - max_pos[0])
                            dy = max(min_pos[1] - wy, 0.0, wy - max_pos[1])
                            if __import__('math').hypot(dx, dy) <= inflation:
                                obstacles.add(iy * grid.width + ix)
                                
        return obstacles

    def _navigation_tick(self) -> None:
        nav = self.navigator
        if nav.status == "navigating":
            obstacles = self._detect_dynamic_obstacles()
            if obstacles:
                state = self.get_robot_state()
                route_blocked = True
                if nav.plan and nav.plan.waypoints:
                    idx = nav.follower.index if nav.follower else 0
                    pts = [(state.pose.position[0], state.pose.position[1])] + [(w.x, w.y) for w in nav.plan.waypoints[idx:]]
                    from .navigation.planner import validate_path
                    if validate_path(nav.grid, pts, dynamic_blocked_cells=obstacles):
                        route_blocked = False
                    else:
                        print("Route blocked! Pts:", pts)
                        for cx, cy in nav.grid.traverse(pts[0], pts[1]):
                            if (cx + cy * nav.grid.width) in obstacles:
                                print(f"Segment blocked at {cx}, {cy}!")
                                break
                
                if route_blocked:
                    nav.pause("dynamic obstacle detected")
                    self._events.append(("warning", "Dynamic obstacle detected. Replanning..."))
                    if nav.destination:
                        try:
                            nav.plan_route(
                                (state.pose.position[0], state.pose.position[1]), 
                                nav.destination, 
                                dynamic_blocked_cells=obstacles
                            )
                            nav.start(state)
                            self._events.append(("info", "Successfully replanned around dynamic obstacle."))
                        except Exception as e:
                            nav.fail(f"Replan failed: {str(e)}")
                            self._events.append(("warning", f"Failed to replan: {str(e)}"))

        cmd = nav.tick(self.get_robot_state(), self._nav_steps * self.timestep)
        if cmd is None or cmd.phase == "arrived":
            self.stop()
            return
        lim = self.config.robot.limits
        v = max(-lim.max_linear_velocity, min(lim.max_linear_velocity, cmd.linear))
        w = max(-lim.max_angular_velocity, min(lim.max_angular_velocity, cmd.angular))
        self.set_velocity_command(v, w, nav.cfg.command_duration)

    def _apply_mechanism_motor(self, name: MechanismName) -> None:
        """Force- and velocity-limited PyBullet position motor towards the target."""
        cfg = actuator_config(self.config.robot, name)
        pb.setJointMotorControl2(
            self.robot_id, self.model.joints[MECHANISM_JOINTS[name]], pb.POSITION_CONTROL,
            targetPosition=self._mech_targets[name], targetVelocity=0.0,
            force=cfg.max_force, maxVelocity=cfg.max_velocity, physicsClientId=self._client,
        )  # fmt: skip

    def _command_mechanism(self, name: MechanismName, target: float, clamped: bool = False) -> MechanismCommand:
        previous = self._mech_targets[name]
        self._mech_targets[name] = target
        self._mech_stall_time[name] = 0.0
        self._mech_overload_time[name] = 0.0
        self._mech_fault[name] = None
        self._apply_mechanism_motor(name)
        return MechanismCommand(name, target, previous, self.mechanism_position(name), clamped)

    def _monitor_mechanisms(self) -> None:
        """Protection for the lift and forks. A mechanism short of its target
        is stopped where it is and flagged as blocked (until its next command)
        when it stalls, saturates its motor force, or tilts the chassis."""
        joints = [self.model.joints[MECHANISM_JOINTS[name]] for name in MECHANISMS]
        states = pb.getJointStates(self.robot_id, joints, physicsClientId=self._client)
        _, orientation = pb.getBasePositionAndOrientation(self.robot_id, physicsClientId=self._client)
        roll, pitch, _ = pb.getEulerFromQuaternion(orientation)
        tilted = max(abs(roll), abs(pitch)) > self.config.robot.limits.max_handling_tilt
        for name, (position, velocity, _, force) in zip(MECHANISMS, states):
            cfg = actuator_config(self.config.robot, name)
            if self._mech_fault[name] or abs(self._mech_targets[name] - position) <= cfg.position_tolerance:
                self._mech_stall_time[name] = self._mech_overload_time[name] = 0.0
                continue
            stalled = abs(velocity) < STALL_SPEED
            self._mech_stall_time[name] = self._mech_stall_time[name] + self.timestep if stalled else 0.0
            saturated = abs(force) >= OVERLOAD_RATIO * cfg.max_force
            self._mech_overload_time[name] = self._mech_overload_time[name] + self.timestep if saturated else 0.0
            if tilted:
                self._trip_mechanism(name, position, "tilt", f"chassis tilted {math.degrees(max(abs(roll), abs(pitch))):.1f}°")
            elif self._mech_overload_time[name] >= OVERLOAD_TIME:
                self._trip_mechanism(name, position, "overload", f"motor at its {cfg.max_force:.0f} N limit")
            elif self._mech_stall_time[name] >= STALL_TIME:
                self._trip_mechanism(name, position, "stalled", "no motion")

    def _trip_mechanism(self, name: MechanismName, position: float, fault: MechanismFault, detail: str) -> None:
        """Hold a mechanism where it is and report why (and what it is touching)."""
        cfg = actuator_config(self.config.robot, name)
        wanted = self._mech_targets[name]
        self._mech_targets[name] = min(cfg.upper, max(cfg.lower, position))
        self._apply_mechanism_motor(name)
        self._mech_fault[name] = fault
        obstacle = self._mechanism_obstacle(name)
        by = f" by {obstacle}" if obstacle else ""
        self._events.append(
            (
                "warning",
                f"{name} blocked{by} ({fault}: {detail}) at {position:.3f} m before reaching {wanted:.3f} m; "
                "holding position",
            )
        )

    def _mechanism_obstacle(self, name: MechanismName) -> str | None:
        """Name what a blocked mechanism is touching, from PyBullet contact points."""
        moving_links = [FORK_LINK] if name == "forks" else [LIFT_CARRIAGE_LINK, FORK_LINK]
        names = {body: box_id for box_id, body in self.static_body_ids.items()}
        for cid, id_str in self.container_ids.items():
            names[cid] = id_str
        found: list[str] = []
        for link in moving_links:
            for contact in pb.getContactPoints(
                bodyA=self.robot_id, linkIndexA=self.model.links[link], physicsClientId=self._client
            ):
                body = contact[2]
                if body == self.robot_id:
                    continue
                label = names.get(body, f"body {body}")
                if label not in found:
                    found.append(label)
        return ", ".join(found) or None

    def _mechanism_telemetry(self, name: MechanismName, joint_state: tuple) -> MechanismTelemetry:
        cfg = actuator_config(self.config.robot, name)
        position, velocity, _, applied = joint_state
        target = self._mech_targets[name]
        at_target = abs(target - position) <= cfg.position_tolerance
        fault = self._mech_fault[name]
        state = "blocked" if fault else "holding" if at_target else "moving"
        return MechanismTelemetry(
            position=position, velocity=velocity, target=target, error=target - position,
            at_target=at_target, state=state, fault=fault, applied_force=applied,
            lower=cfg.lower, upper=cfg.upper, default=cfg.default_position, max_velocity=cfg.max_velocity,
        )  # fmt: skip

    def _apply_drive(self) -> None:
        """Dead-man timeout + acceleration limiting, then wheel motor targets
        (or the parking brake once the robot has come to rest)."""
        limits, dt = self.config.robot.limits, self.timestep
        if self._command_expires_at > self.sim_time:
            target_v, target_w = self._target.linear, self._target.angular
        else:
            target_v = target_w = 0.0
        self._linear_cmd = ramp(
            self._linear_cmd, target_v, limits.max_linear_acceleration, limits.max_linear_deceleration, dt
        )
        self._angular_cmd = ramp(
            self._angular_cmd, target_w, limits.max_angular_acceleration, limits.max_angular_deceleration, dt
        )
        self._wheel_targets = self.kinematics.wheel_speeds(self._linear_cmd, self._angular_cmd)

        wheels = [self.model.joints[LEFT_WHEEL_JOINT], self.model.joints[RIGHT_WHEEL_JOINT]]
        torque = self.config.robot.wheels.max_torque
        idle = target_v == target_w == self._linear_cmd == self._angular_cmd == 0.0
        if idle:
            if self._brake_engaged:
                return  # position-hold motors persist between steps
            (left_q, left_qd, *_), (right_q, right_qd, *_) = pb.getJointStates(
                self.robot_id, wheels, physicsClientId=self._client
            )
            if max(abs(left_qd), abs(right_qd)) < BRAKE_ENGAGE_SPEED:
                self._brake_engaged = True
                pb.setJointMotorControlArray(
                    self.robot_id, wheels, pb.POSITION_CONTROL,
                    targetPositions=[left_q, right_q], targetVelocities=[0.0, 0.0],
                    forces=[torque, torque], physicsClientId=self._client,
                )  # fmt: skip
                return
        self._brake_engaged = False
        pb.setJointMotorControlArray(
            self.robot_id,
            wheels,
            pb.VELOCITY_CONTROL,
            targetVelocities=[self._wheel_targets.left, self._wheel_targets.right],
            forces=[torque, torque],
            physicsClientId=self._client,
        )
