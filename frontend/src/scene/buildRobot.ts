import * as THREE from 'three';
import type { RobotJoint, RobotModel, RobotShape, RobotState } from '../api/types';
import { rpyEuler } from './three-utils';

/**
 * Builds the robot from the backend's robot description — the same links,
 * primitives and joint frames the URDF loaded into PyBullet was generated
 * from. Nothing here animates the robot on its own: ``applyRobotState`` only
 * copies the pose and joint positions reported by the physics engine.
 */
export interface RobotRig {
  root: THREE.Group;
  joints: Map<string, { joint: RobotJoint; motion: THREE.Group }>;
}

const METALLIC = new Set(['mast', 'fork', 'hub']);

function shapeObject(shape: RobotShape, material: THREE.Material): THREE.Object3D {
  const holder = new THREE.Group();
  holder.name = shape.name;
  holder.position.set(...shape.xyz);
  holder.setRotationFromEuler(rpyEuler(shape.rpy));
  let geometry: THREE.BufferGeometry;
  if (shape.kind === 'box') {
    geometry = new THREE.BoxGeometry(shape.size[0], shape.size[1], shape.size[2]);
  } else if (shape.kind === 'cylinder') {
    geometry = new THREE.CylinderGeometry(shape.size[0], shape.size[0], shape.size[1], 40);
  } else {
    geometry = new THREE.SphereGeometry(shape.size[0], 24, 16);
  }
  const mesh = new THREE.Mesh(geometry, material);
  // URDF cylinders run along local Z; Three.js cylinders along local Y.
  if (shape.kind === 'cylinder') mesh.rotation.x = Math.PI / 2;
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  holder.add(mesh);
  return holder;
}

export function buildRobot(model: RobotModel): RobotRig {
  const materials = new Map<string, THREE.Material>();
  for (const [name, [r, g, b, a]] of Object.entries(model.materials)) {
    const emissive = name === 'beacon' ? new THREE.Color(r, g, b).multiplyScalar(0.6) : new THREE.Color(0, 0, 0);
    materials.set(
      name,
      new THREE.MeshStandardMaterial({
        color: new THREE.Color(r, g, b),
        opacity: a,
        transparent: a < 1,
        metalness: METALLIC.has(name) ? 0.65 : 0.1,
        roughness: METALLIC.has(name) ? 0.35 : 0.6,
        emissive,
      }),
    );
  }
  const fallback = new THREE.MeshStandardMaterial({ color: 0xff00ff });

  const links = new Map<string, THREE.Group>();
  for (const link of model.links) {
    const group = new THREE.Group();
    group.name = link.name;
    for (const shape of link.shapes) group.add(shapeObject(shape, materials.get(shape.material) ?? fallback));
    links.set(link.name, group);
  }

  const joints = new Map<string, { joint: RobotJoint; motion: THREE.Group }>();
  for (const joint of model.joints) {
    const parent = links.get(joint.parent);
    const child = links.get(joint.child);
    if (!parent || !child) throw new Error(`robot model joint ${joint.name} references a missing link`);
    const frame = new THREE.Group(); // fixed joint origin in the parent link
    frame.name = `${joint.name}:origin`;
    frame.position.set(...joint.xyz);
    frame.setRotationFromEuler(rpyEuler(joint.rpy));
    const motion = new THREE.Group(); // joint displacement applied here
    motion.name = `${joint.name}:motion`;
    frame.add(motion);
    motion.add(child);
    parent.add(frame);
    joints.set(joint.name, { joint, motion });
  }

  const root = links.get(model.base_link);
  if (!root) throw new Error(`robot model has no base link ${model.base_link}`);
  root.visible = false; // shown once real telemetry arrives
  return { root, joints };
}

function setJoint(rig: RobotRig, name: string, q: number): void {
  const entry = rig.joints.get(name);
  if (!entry) return;
  const [ax, ay, az] = entry.joint.axis;
  if (entry.joint.type === 'prismatic') {
    entry.motion.position.set(ax * q, ay * q, az * q);
  } else if (entry.joint.type === 'continuous') {
    entry.motion.quaternion.setFromAxisAngle(new THREE.Vector3(ax, ay, az), q);
  }
}

/** Copy the physics-reported pose and joint positions onto the rig. */
export function applyRobotState(rig: RobotRig, state: RobotState): void {
  const [x, y, z] = state.pose.position;
  const [qx, qy, qz, qw] = state.pose.orientation;
  rig.root.position.set(x, y, z);
  rig.root.quaternion.set(qx, qy, qz, qw);
  setJoint(rig, 'left_wheel_joint', state.wheels.left.position);
  setJoint(rig, 'right_wheel_joint', state.wheels.right.position);
  setJoint(rig, 'lift_joint', state.lift.position);
  setJoint(rig, 'fork_joint', state.forks.position);
  rig.root.visible = true;
}
