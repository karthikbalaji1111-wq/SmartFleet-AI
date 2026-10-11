import math
import pytest
from simulation.alignment import (
    TargetAcquisition,
    TargetObservation,
    world_to_robot_body,
    derive_pickup_orientation,
    calculate_target_relative_pose,
    normalize_angle,
)
from simulation.config import get_config


@pytest.fixture
def target_acquisition():
    return TargetAcquisition(get_config(), max_observation_age=1.5)


def test_target_identification_success(target_acquisition):
    current_time = 100.0
    perception_data = {
        "timestamp": 99.8,
        "latency_ms": 15.0,
        "rgb_base64": "",
        "objects": [
            {
                "category": "Container",
                "instance_id": "TOTE-0001",
                "bbox": [100, 80, 50, 40],
                "distance": 1.2,
                "position": {"x": 10.4, "y": 4.0, "z": 0.8},
                "aabb_boundary_distance": 0.5,
            }
        ],
    }

    obs, err = target_acquisition.acquire_target("TOTE-0001", perception_data, current_time)
    assert err is None
    assert obs is not None
    assert obs.container_id == "TOTE-0001"
    assert obs.world_pos == (10.4, 4.0, 0.8)
    assert obs.distance == 1.2
    assert math.isclose(obs.age, 0.2, abs_tol=1e-3)
    assert obs.aabb_boundary_distance == 0.5


def test_target_identification_unknown_container(target_acquisition):
    obs, err = target_acquisition.acquire_target("TOTE-9999", {}, 100.0)
    assert obs is None
    assert "not a valid warehouse container" in err


def test_target_identification_missing_observation(target_acquisition):
    obs, err = target_acquisition.acquire_target("TOTE-0001", None, 100.0)
    assert obs is None
    assert "No perception observation available" in err


def test_target_identification_stale_observation(target_acquisition):
    current_time = 100.0
    perception_data = {
        "timestamp": 98.0,  # age 2.0s > 1.5s limit
        "objects": [
            {
                "category": "Container",
                "instance_id": "TOTE-0001",
                "bbox": [100, 80, 50, 40],
                "distance": 1.2,
                "position": {"x": 10.4, "y": 4.0, "z": 0.8},
            }
        ],
    }
    obs, err = target_acquisition.acquire_target("TOTE-0001", perception_data, current_time)
    assert obs is None
    assert "Observation is stale" in err


def test_target_identification_distractor_discrimination(target_acquisition):
    current_time = 100.0
    perception_data = {
        "timestamp": 99.8,
        "objects": [
            {
                "category": "Container",
                "instance_id": "TOTE-0002",
                "bbox": [50, 80, 50, 40],
                "distance": 1.5,
                "position": {"x": -7.0, "y": 2.0, "z": 0.5},
            }
        ],
    }
    # We ask for TOTE-0001, but only TOTE-0002 is visible
    obs, err = target_acquisition.acquire_target("TOTE-0001", perception_data, current_time)
    assert obs is None
    assert "not in camera view; visible containers: TOTE-0002" in err


def test_target_identification_invalid_depth(target_acquisition):
    current_time = 100.0
    # Depth below minimum (e.g. 0.05m < 0.1m)
    perception_data = {
        "timestamp": 99.9,
        "objects": [
            {
                "category": "Container",
                "instance_id": "TOTE-0001",
                "bbox": [100, 80, 50, 40],
                "distance": 0.05,
                "position": {"x": 10.4, "y": 4.0, "z": 0.8},
            }
        ],
    }
    obs, err = target_acquisition.acquire_target("TOTE-0001", perception_data, current_time)
    assert obs is None
    assert "Target depth 0.05m is outside valid sensor range" in err


def test_target_identification_out_of_bounds_coordinates(target_acquisition):
    current_time = 100.0
    perception_data = {
        "timestamp": 99.9,
        "objects": [
            {
                "category": "Container",
                "instance_id": "TOTE-0001",
                "bbox": [100, 80, 50, 40],
                "distance": 2.0,
                "position": {"x": 50.0, "y": 4.0, "z": 0.8},  # floor X is +/- 12
            }
        ],
    }
    obs, err = target_acquisition.acquire_target("TOTE-0001", perception_data, current_time)
    assert obs is None
    assert "outside warehouse boundaries" in err


def test_coordinate_transforms_robot_heading_zero():
    # Robot at (0, 0) facing east (yaw=0)
    # Target straight ahead at (2.0, 0.0, 0.5)
    xr, yr, zr = world_to_robot_body(0.0, 0.0, 0.0, 2.0, 0.0, 0.5, 0.0)
    assert math.isclose(xr, 2.0, abs_tol=1e-5)
    assert math.isclose(yr, 0.0, abs_tol=1e-5)
    assert math.isclose(zr, 0.5, abs_tol=1e-5)

    # Target to the left (+Y in world = +Y in body)
    xr, yr, _ = world_to_robot_body(0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0)
    assert math.isclose(xr, 1.0, abs_tol=1e-5)
    assert math.isclose(yr, 0.5, abs_tol=1e-5)

    # Target to the right (-Y in world = -Y in body)
    xr, yr, _ = world_to_robot_body(0.0, 0.0, 0.0, 1.0, -0.5, 0.0, 0.0)
    assert math.isclose(xr, 1.0, abs_tol=1e-5)
    assert math.isclose(yr, -0.5, abs_tol=1e-5)


def test_coordinate_transforms_robot_rotated():
    # Robot at (0, 0) facing north (yaw = pi/2)
    # Target at (0, 2.0) in world -> directly ahead in body frame
    xr, yr, _ = world_to_robot_body(0.0, 0.0, math.pi / 2.0, 0.0, 2.0, 0.0, 0.0)
    assert math.isclose(xr, 2.0, abs_tol=1e-5)
    assert math.isclose(yr, 0.0, abs_tol=1e-5)

    # Target at (0, 0) facing south (yaw = -pi/2)
    # Target at (0, -2.0) in world -> directly ahead in body frame
    xr, yr, _ = world_to_robot_body(0.0, 0.0, -math.pi / 2.0, 0.0, -2.0, 0.0, 0.0)
    assert math.isclose(xr, 2.0, abs_tol=1e-5)
    assert math.isclose(yr, 0.0, abs_tol=1e-5)


def test_bearing_and_orientation_calculations():
    obs = TargetObservation(
        container_id="TOTE-0001",
        category="Container",
        world_pos=(10.0, 4.2, 0.8),
        distance=1.0,
        bbox=(0, 0, 10, 10),
        timestamp=100.0,
        age=0.1,
    )
    # Robot at (9.0, 4.0), heading 0.0 (facing east). Desired heading is 0.0 (east).
    # Target is at dx = 1.0, dy = 0.2.
    rel_pose = calculate_target_relative_pose(
        robot_x=9.0,
        robot_y=4.0,
        robot_z=0.1,
        robot_yaw=0.0,
        observation=obs,
        desired_heading=0.0,
    )
    assert math.isclose(rel_pose.x_rel, 1.0, abs_tol=1e-5)
    assert math.isclose(rel_pose.y_rel, 0.2, abs_tol=1e-5)
    assert math.isclose(rel_pose.bearing_error, math.atan2(0.2, 1.0), abs_tol=1e-5)
    assert math.isclose(rel_pose.orientation_error, 0.0, abs_tol=1e-5)

    # If robot is tilted 0.1 rad to the left (yaw=0.1):
    rel_pose_tilted = calculate_target_relative_pose(
        robot_x=9.0,
        robot_y=4.0,
        robot_z=0.1,
        robot_yaw=0.1,
        observation=obs,
        desired_heading=0.0,
    )
    assert math.isclose(rel_pose_tilted.orientation_error, -0.1, abs_tol=1e-5)


def test_derive_pickup_orientation():
    wh = get_config().warehouse
    # Inbound stand is at x=10.4 > 8.0 -> yaw = 0.0
    assert derive_pickup_orientation("inbound-stand", wh) == 0.0

    # Rack A1 slots are at y=5.5 > aisle 3.75 -> robot in aisle faces north (pi/2)
    assert math.isclose(derive_pickup_orientation("A1-B1-L1-S1", wh), math.pi / 2.0, abs_tol=1e-5)

    # Rack B1 slots are at y=2.0 < aisle 3.75 -> robot in aisle faces south (-pi/2)
    assert math.isclose(derive_pickup_orientation("B1-B1-L1-S1", wh), -math.pi / 2.0, abs_tol=1e-5)


from simulation.alignment import (
    calculate_staging_pose,
    derive_fork_insertion_target,
    AlignmentController,
    AlignmentTolerances,
    TargetRelativePose,
)


def test_calculate_staging_pose():
    wh = get_config().warehouse

    # Station staging: x=10.4 - 1.0 = 9.4, y=4.0, yaw=0.0
    sx, sy, syaw = calculate_staging_pose("inbound-stand", wh, standoff_distance=1.0)
    assert math.isclose(sx, 9.4, abs_tol=1e-5)
    assert math.isclose(sy, 4.0, abs_tol=1e-5)
    assert math.isclose(syaw, 0.0, abs_tol=1e-5)

    # Rack A1 slot staging: slot at y=5.5 > aisle 3.75 -> sy = 5.5 - 1.0 = 4.5, yaw = pi/2
    slot = next(s for s in wh.slots if s.id == "A1-B1-L1-S1")
    sx, sy, syaw = calculate_staging_pose("A1-B1-L1-S1", wh, standoff_distance=1.0)
    assert math.isclose(sx, slot.center[0], abs_tol=1e-5)
    assert math.isclose(sy, slot.center[1] - 1.0, abs_tol=1e-5)
    assert math.isclose(syaw, math.pi / 2.0, abs_tol=1e-5)


def test_derive_fork_insertion_target():
    robot_cfg = get_config().robot
    # Shelf at z=0.40m, container height 0.22m
    lift_target, fork_target = derive_fork_insertion_target(
        robot_config=robot_cfg,
        container_height=0.22,
        shelf_surface_z=0.40,
        insertion_depth=0.58,
    )
    assert robot_cfg.lift.lower <= lift_target <= robot_cfg.lift.upper
    assert math.isclose(fork_target, 0.58, abs_tol=1e-5)


def test_alignment_controller_convergence():
    tol = AlignmentTolerances(
        lateral_tol=0.035,
        heading_tol=0.040,
        distance_min=0.65,
        distance_max=0.85,
    )
    controller = AlignmentController(tolerances=tol)

    # 1. Start misaligned: heading error 0.2 rad, distance 1.2m
    pose1 = TargetRelativePose(
        container_id="TOTE-0001",
        x_rel=1.20,
        y_rel=0.08,
        z_rel=0.2,
        distance_2d=1.20,
        distance_3d=1.22,
        bearing_error=0.06,
        desired_heading=0.0,
        orientation_error=0.20,
        observation_age=0.1,
    )
    v, w, aligned, msg = controller.update(pose1, dt=0.1)
    assert not aligned
    assert v == 0.0
    assert w > 0.0  # Turns to correct heading

    # 2. Now heading aligned, but forward distance still large (1.1m)
    pose2 = TargetRelativePose(
        container_id="TOTE-0001",
        x_rel=1.10,
        y_rel=0.02,
        z_rel=0.2,
        distance_2d=1.10,
        distance_3d=1.12,
        bearing_error=0.01,
        desired_heading=0.0,
        orientation_error=0.01,
        observation_age=0.1,
    )
    v, w, aligned, msg = controller.update(pose2, dt=0.1)
    assert not aligned
    assert v > 0.0  # Moves forward

    # 3. Target now within all tolerances
    pose_aligned = TargetRelativePose(
        container_id="TOTE-0001",
        x_rel=0.75,
        y_rel=0.01,
        z_rel=0.2,
        distance_2d=0.75,
        distance_3d=0.77,
        bearing_error=0.01,
        desired_heading=0.0,
        orientation_error=0.01,
        observation_age=0.1,
    )
    v, w, aligned, msg = controller.update(pose_aligned, dt=0.1)
    assert aligned
    assert v == 0.0
    assert w == 0.0
    assert "Aligned!" in msg


def test_alignment_controller_timeout():
    tol = AlignmentTolerances(timeout_s=0.5)
    controller = AlignmentController(tolerances=tol)

    pose = TargetRelativePose(
        container_id="TOTE-0001",
        x_rel=1.5,
        y_rel=0.5,
        z_rel=0.0,
        distance_2d=1.5,
        distance_3d=1.5,
        bearing_error=0.3,
        desired_heading=0.0,
        orientation_error=0.5,
        observation_age=0.1,
    )
    controller.update(pose, dt=0.3)
    v, w, aligned, msg = controller.update(pose, dt=0.3)  # total 0.6s > 0.5s
    assert not aligned
    assert controller.state.phase == "failed"
    assert "timed out" in msg


from simulation.tasks import TaskManager, TaskRequest
import pybullet as pb


def test_task_manager_perception_pickup_lifecycle(world):
    world.initialize()
    world.step(50)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    assert tm.state == "approach_source"
    assert tm.pickup_telemetry_data.target_id == "TOTE-0001"

    # Step until pickup verification is reached
    verified = False
    for _ in range(6000):
        world.step(1)
        tm.tick(world.timestep)
        if tm.pickup_telemetry_data.verified_pickup:
            verified = True
            break
        if tm.state == "failed":
            pytest.fail(f"Task failed prematurely: {tm.error}")

    assert verified, f"Task did not verify pickup. Final state: {tm.state}, err: {tm.error}"
    assert tm.pickup_telemetry_data.cargo_state == "attached"
    assert "TOTE-0001" in world._cargo_constraints

    # Confirm physical elevation of the container
    c_id = world.container_ids["TOTE-0001"]
    pos, _ = pb.getBasePositionAndOrientation(c_id, physicsClientId=world._client)
    # Original stand top was 0.70m, container initial center ~0.81m. When elevated, pos[2] > 0.86m
    assert pos[2] > 0.86, f"Container was not physically lifted: z={pos[2]:.3f}m"


def test_task_manager_cancellation(world):
    world.initialize()
    world.step(20)

    tm = TaskManager(world)
    req = TaskRequest(container_id="TOTE-0001", destination="A1-B1-L1-S2")
    tm.submit(req)

    # Step a few frames
    for _ in range(30):
        world.step(1)
        tm.tick(world.timestep)

    # Cancel task
    tm.cancel()
    assert tm.state == "cancelled"
    assert tm.task is None
    assert tm.pickup_telemetry_data.stage == "cancelled"
    assert not world.navigator.active
