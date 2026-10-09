"""PyBullet physics world for SmartFleet AI (DIRECT mode, fixed timestep).

The world owns one PyBullet client. Robot pose and velocities are always read
back from PyBullet; nothing here integrates a competing pose. Motion is
produced only by wheel velocity motors acting through contact friction.
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
from .robot_model import (
    EXPECTED_JOINTS,
    FORK_JOINT,
    FRONT_CASTER_LINK,
    LEFT_WHEEL_JOINT,
    LIFT_JOINT,
    REAR_CASTER_LINK,
    RIGHT_WHEEL_JOINT,
    RobotDescription,
    build_robot_description,
    write_urdf,
)
from .state import (
    ActuatorTelemetry,
    BodyState,
    CommandTelemetry,
    JointTelemetry,
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
            self._apply_drive()
            pb.stepSimulation(physicsClientId=self._client)
            self._step_count += 1

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
        lift_cfg, fork_cfg = self.config.robot.lift, self.config.robot.forks
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
            lift=ActuatorTelemetry(
                position=lift[0], velocity=lift[1], lower=lift_cfg.lower, upper=lift_cfg.upper,
                target=self._lift_target, mode="hold",
            ),
            forks=ActuatorTelemetry(
                position=fork[0], velocity=fork[1], lower=fork_cfg.lower, upper=fork_cfg.upper,
                target=self._fork_target, mode="hold",
            ),
            command=CommandTelemetry(
                active=active,
                target_linear=self._target.linear if active else 0.0,
                target_angular=self._target.angular if active else 0.0,
                linear=self._linear_cmd,
                angular=self._angular_cmd,
                remaining=max(0.0, self._command_expires_at - self.sim_time),
            ),
        )  # fmt: skip

    def get_container_state(self) -> BodyState:
        self._require_initialized()
        pos, orn = pb.getBasePositionAndOrientation(self.container_id, physicsClientId=self._client)
        return BodyState(id=self.config.warehouse.container.id, position=tuple(pos), orientation=tuple(orn))

    def get_state(self) -> WorldState:
        return WorldState(
            sim_time=self.sim_time,
            step=self._step_count,
            robot=self.get_robot_state(),
            container=self.get_container_state(),
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
        self.container_id = -1
        self.static_body_ids: dict[str, int] = {}
        self.model: RobotModelIndex | None = None
        self._step_count = 0
        self._target = _ZERO_COMMAND
        self._command_expires_at = 0.0
        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._wheel_targets = WheelSpeeds(0.0, 0.0)
        self._brake_engaged = False
        self._lift_target = self.config.robot.lift.lower
        self._fork_target = self.config.robot.forks.lower

    def _build_world(self) -> None:
        c, sim, robot = self._client, self.config.simulation, self.config.robot
        pb.resetSimulation(physicsClientId=c)
        self._clear_runtime_state()
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

        cont = self.config.warehouse.container
        cx, cy, cz = container_initial_position(self.config.warehouse)
        cshape = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=[s / 2 for s in cont.size], physicsClientId=c
        )
        self.container_id = pb.createMultiBody(
            baseMass=cont.mass,
            baseCollisionShapeIndex=cshape,
            basePosition=(cx, cy, cz + SPAWN_CLEARANCE),
            physicsClientId=c,
        )
        pb.changeDynamics(self.container_id, -1, lateralFriction=0.8, physicsClientId=c)

        # Let contacts settle with all motors holding; the clock starts afterwards.
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
        # Lift and forks are physically actuated position joints, held at their
        # targets. Commanded motion for them is deferred to a later milestone.
        for name, target, cfg in (
            (LIFT_JOINT, self._lift_target, robot.lift),
            (FORK_JOINT, self._fork_target, robot.forks),
        ):
            pb.resetJointState(body, joints[name], target, physicsClientId=c)
            pb.setJointMotorControl2(
                body, joints[name], pb.POSITION_CONTROL, targetPosition=target,
                force=cfg.max_force, maxVelocity=cfg.max_velocity, physicsClientId=c,
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
