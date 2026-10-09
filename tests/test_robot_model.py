"""Robot URDF generation and loading/validation in PyBullet."""

from __future__ import annotations

import dataclasses
import xml.etree.ElementTree as ET

import pytest

from simulation.robot_model import (
    EXPECTED_JOINTS,
    EXPECTED_LINKS,
    FORK_JOINT,
    LIFT_JOINT,
    build_robot_description,
    write_urdf,
)


@pytest.fixture
def description(config):
    return build_robot_description(config.robot)


def test_description_has_canonical_links_and_joints(description):
    assert {link.name for link in description.links} == set(EXPECTED_LINKS)
    for name, (jtype, child) in EXPECTED_JOINTS.items():
        joint = description.joint(name)
        assert (joint.type, joint.child) == (jtype, child)


def test_generated_urdf_is_well_formed(description, config):
    root = ET.fromstring(description.to_urdf())
    assert root.tag == "robot"
    assert {el.get("name") for el in root.findall("link")} == set(EXPECTED_LINKS)
    joints = {el.get("name"): el for el in root.findall("joint")}
    assert set(joints) == set(EXPECTED_JOINTS)
    lift_limit = joints[LIFT_JOINT].find("limit")
    assert float(lift_limit.get("lower")) == config.robot.lift.lower
    assert float(lift_limit.get("upper")) == config.robot.lift.upper
    fork_limit = joints[FORK_JOINT].find("limit")
    assert float(fork_limit.get("upper")) == config.robot.forks.upper
    assert joints["left_wheel_joint"].find("axis").get("xyz") == "0 1 0"
    total_mass = sum(float(el.find("inertial/mass").get("value")) for el in root.findall("link"))
    assert total_mass == pytest.approx(
        config.robot.chassis.mass + 2 * config.robot.wheels.mass + 2 * config.robot.casters.mass
        + config.robot.mast.mass + config.robot.lift.carriage_mass + config.robot.forks.mass
    )  # fmt: skip


def test_robot_loads_with_expected_joints(world, pb, config):
    c, body = world.client_id, world.robot_id
    info = {}
    for i in range(pb.getNumJoints(body, physicsClientId=c)):
        j = pb.getJointInfo(body, i, physicsClientId=c)
        info[j[1].decode()] = j
    assert set(info) == set(EXPECTED_JOINTS)
    assert info["left_wheel_joint"][2] == pb.JOINT_REVOLUTE
    assert info["right_wheel_joint"][2] == pb.JOINT_REVOLUTE
    assert info[LIFT_JOINT][2] == pb.JOINT_PRISMATIC
    assert info[FORK_JOINT][2] == pb.JOINT_PRISMATIC
    assert info["mast_joint"][2] == pb.JOINT_FIXED
    assert (info[LIFT_JOINT][8], info[LIFT_JOINT][9]) == pytest.approx((config.robot.lift.lower, config.robot.lift.upper))
    assert (info[FORK_JOINT][8], info[FORK_JOINT][9]) == pytest.approx((config.robot.forks.lower, config.robot.forks.upper))
    assert pb.getBodyInfo(body, physicsClientId=c)[0].decode() == "base_link"
    assert set(world.model.links) == set(EXPECTED_LINKS)
    # Masses PyBullet uses come from the generated URDF.
    assert pb.getDynamicsInfo(body, -1, physicsClientId=c)[0] == pytest.approx(config.robot.chassis.mass)
    assert pb.getDynamicsInfo(body, world.model.links["lift_carriage"], physicsClientId=c)[0] == pytest.approx(
        config.robot.lift.carriage_mass
    )


def test_drive_wheels_have_traction_and_casters_are_frictionless(world, pb, config):
    c, body, links = world.client_id, world.robot_id, world.model.links
    assert pb.getDynamicsInfo(body, links["left_wheel"], physicsClientId=c)[1] == pytest.approx(
        config.robot.wheels.lateral_friction
    )
    assert pb.getDynamicsInfo(body, links["caster_front"], physicsClientId=c)[1] == pytest.approx(0.0)


def test_validation_rejects_model_with_renamed_joint(world, pb, description, tmp_path):
    from simulation.world import RobotModelError, index_robot_model

    broken = dataclasses.replace(
        description,
        joints=tuple(
            dataclasses.replace(j, name="lift_joint_v2") if j.name == LIFT_JOINT else j for j in description.joints
        ),
    )
    path = write_urdf(broken, tmp_path)
    body = pb.loadURDF(str(path), basePosition=(0, 0, 3), physicsClientId=world.client_id)
    with pytest.raises(RobotModelError, match="lift_joint"):
        index_robot_model(world.client_id, body, description)


def test_validation_rejects_model_with_wrong_limits(world, pb, description, tmp_path):
    from simulation.world import RobotModelError, index_robot_model

    broken = dataclasses.replace(
        description,
        joints=tuple(dataclasses.replace(j, upper=0.1) if j.name == FORK_JOINT else j for j in description.joints),
    )
    body = pb.loadURDF(str(write_urdf(broken, tmp_path)), basePosition=(0, 0, 3), physicsClientId=world.client_id)
    with pytest.raises(RobotModelError, match="limits"):
        index_robot_model(world.client_id, body, description)
