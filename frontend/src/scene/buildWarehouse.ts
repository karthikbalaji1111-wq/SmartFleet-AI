import * as THREE from 'three';
import type { StaticBox, WarehouseConfigResponse, ZoneConfig } from '../api/types';
import { labelSprite, textTexture } from './three-utils';

/**
 * Builds the warehouse in the world frame (Z up). Every solid object is one of
 * the backend's ``static_geometry`` boxes — exactly the PyBullet collision
 * bodies — so what you see is what the robot can collide with. Floor
 * markings (grid, zones, start arrow) are paint only.
 */

const ZONE_COLORS: Record<ZoneConfig['kind'], string> = {
  loading: '#f5b324',
  delivery: '#3fb950',
  home: '#39c5cf',
};

function boxMaterials() {
  return {
    wall: new THREE.MeshStandardMaterial({
      color: '#c9d1d9',
      roughness: 0.9,
      transparent: true,
      opacity: 0.22,
      depthWrite: false,
      side: THREE.DoubleSide,
    }),
    rack_post: new THREE.MeshStandardMaterial({ color: '#2f6fd6', roughness: 0.45, metalness: 0.5 }),
    shelf: [
      new THREE.MeshStandardMaterial({ color: '#f07c1b', roughness: 0.55, metalness: 0.3 }),
      new THREE.MeshStandardMaterial({ color: '#d9661a', roughness: 0.55, metalness: 0.3 }),
    ],
    station: new THREE.MeshStandardMaterial({ color: '#59636e', roughness: 0.7, metalness: 0.2 }),
  };
}

function boxMesh(box: StaticBox, material: THREE.Material): THREE.Mesh {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...box.size), material);
  mesh.position.set(...box.center);
  mesh.name = box.id;
  mesh.userData = { kind: box.kind, group: box.group };
  mesh.castShadow = box.kind !== 'wall';
  mesh.receiveShadow = true;
  return mesh;
}

function floorGrid(sx: number, sy: number, spacing: number): THREE.Group {
  const minor: number[] = [];
  const major: number[] = [];
  const z = 0.003;
  for (let x = -sx / 2; x <= sx / 2 + 1e-6; x += spacing) {
    (Math.abs(Math.round(x) % 5) === 0 ? major : minor).push(x, -sy / 2, z, x, sy / 2, z);
  }
  for (let y = -sy / 2; y <= sy / 2 + 1e-6; y += spacing) {
    (Math.abs(Math.round(y) % 5) === 0 ? major : minor).push(-sx / 2, y, z, sx / 2, y, z);
  }
  const group = new THREE.Group();
  for (const [positions, opacity] of [
    [minor, 0.18],
    [major, 0.38],
  ] as const) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    group.add(
      new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: '#e6edf3', transparent: true, opacity })),
    );
  }
  return group;
}

function zoneMarking(zone: ZoneConfig): THREE.Group {
  const group = new THREE.Group();
  group.name = `zone:${zone.id}`;
  const [cx, cy] = zone.center;
  const [w, h] = zone.size;
  const color = ZONE_COLORS[zone.kind];

  const fill = new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.16, depthWrite: false }),
  );
  fill.position.set(cx, cy, 0.004);
  fill.receiveShadow = true;
  group.add(fill);

  const border = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(cx - w / 2, cy - h / 2, 0.006),
      new THREE.Vector3(cx + w / 2, cy - h / 2, 0.006),
      new THREE.Vector3(cx + w / 2, cy + h / 2, 0.006),
      new THREE.Vector3(cx - w / 2, cy + h / 2, 0.006),
    ]),
    new THREE.LineBasicMaterial({ color }),
  );
  group.add(border);

  // Painted floor label along the zone's southern edge.
  const { texture, aspect } = textTexture(zone.label, { color, fontPx: 72, weight: 800 });
  const labelH = Math.min(0.42, h * 0.12);
  const labelW = Math.min(labelH * aspect, w * 0.92);
  const label = new THREE.Mesh(
    new THREE.PlaneGeometry(labelW, labelW / aspect),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false }),
  );
  label.position.set(cx, cy - h / 2 + labelH * 0.9, 0.007);
  group.add(label);
  return group;
}

function startArrow(x: number, y: number, yaw: number): THREE.Mesh {
  const shape = new THREE.Shape();
  shape.moveTo(0.45, 0);
  shape.lineTo(-0.25, 0.3);
  shape.lineTo(-0.1, 0);
  shape.lineTo(-0.25, -0.3);
  shape.closePath();
  const arrow = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    new THREE.MeshBasicMaterial({ color: '#39c5cf', transparent: true, opacity: 0.55, depthWrite: false }),
  );
  arrow.position.set(x, y, 0.008);
  arrow.rotation.z = yaw;
  arrow.name = 'start-pose-marker';
  return arrow;
}

export function buildWarehouse(data: WarehouseConfigResponse): THREE.Group {
  const { warehouse, robot } = data.config;
  const { size_x: sx, size_y: sy, grid_spacing } = warehouse.floor;
  const root = new THREE.Group();
  root.name = 'warehouse';

  const outside = new THREE.Mesh(
    new THREE.PlaneGeometry(sx + 40, sy + 40),
    new THREE.MeshStandardMaterial({ color: '#161b22', roughness: 1 }),
  );
  outside.position.z = -0.01;
  outside.receiveShadow = true;
  root.add(outside);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(sx, sy),
    new THREE.MeshStandardMaterial({ color: '#6b727c', roughness: 0.92, metalness: 0.02 }),
  );
  floor.receiveShadow = true;
  floor.name = 'floor';
  root.add(floor);
  root.add(floorGrid(sx, sy, grid_spacing));
  for (const zone of warehouse.zones) root.add(zoneMarking(zone));
  root.add(startArrow(robot.start_pose.x, robot.start_pose.y, robot.start_pose.yaw));

  const mats = boxMaterials();
  const rackBounds = new Map<string, THREE.Box3>();
  for (const box of data.static_geometry) {
    const material =
      box.kind === 'wall'
        ? mats.wall
        : box.kind === 'rack_post'
          ? mats.rack_post
          : box.kind === 'rack_shelf'
            ? mats.shelf[(box.level ?? 0) % 2]
            : mats.station;
    const mesh = boxMesh(box, material);
    root.add(mesh);
    if (box.kind === 'rack_post' || box.kind === 'rack_shelf') {
      const bounds = rackBounds.get(box.group) ?? new THREE.Box3();
      bounds.expandByObject(mesh);
      rackBounds.set(box.group, bounds);
    }
    if (box.kind === 'station') {
      const label = labelSprite(box.id.toUpperCase(), 0.22, '#f5b324');
      label.position.set(box.center[0], box.center[1], box.size[2] + 0.75);
      root.add(label);
    }
  }
  for (const [rackId, bounds] of rackBounds) {
    const center = bounds.getCenter(new THREE.Vector3());
    const label = labelSprite(`RACK ${rackId}`, 0.32);
    label.position.set(center.x, center.y, bounds.max.z + 0.35);
    root.add(label);
  }
  return root;
}

/** The sample tote; its pose always comes from physics telemetry. */
export function buildContainer(size: [number, number, number]): THREE.Group {
  const group = new THREE.Group();
  group.name = 'container';
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(...size),
    new THREE.MeshStandardMaterial({ color: '#3d7bd9', roughness: 0.55 }),
  );
  body.castShadow = true;
  body.receiveShadow = true;
  group.add(body);
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(body.geometry),
    new THREE.LineBasicMaterial({ color: '#0d2a57' }),
  );
  group.add(edges);
  group.visible = false;
  return group;
}

/** Floor ring that makes the robot easy to spot from far away. */
export function buildRobotMarker(radius: number): THREE.Mesh {
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(radius, radius + 0.07, 56),
    new THREE.MeshBasicMaterial({ color: '#f5b324', transparent: true, opacity: 0.7, depthWrite: false }),
  );
  ring.name = 'robot-marker';
  ring.visible = false;
  return ring;
}
