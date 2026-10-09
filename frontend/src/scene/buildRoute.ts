import * as THREE from 'three';
import type { OccupancyGridData, RouteModel } from '../api/types';
import { labelSprite } from './three-utils';

/**
 * Route overlay in the world frame (Z up), built only from the coordinates the
 * backend planner returned. Nothing here moves the robot or fakes progress:
 * ``setProgress`` just re-colours segments using the waypoint index reported
 * in telemetry.
 */
export interface RouteOverlay {
  group: THREE.Group;
  setProgress: (activeIndex: number | null, status: string) => void;
}

const COLORS = {
  ahead: new THREE.Color('#58a6ff'),
  done: new THREE.Color('#3a4a60'),
  active: new THREE.Color('#f5b324'),
  raw: '#9aa7b4',
  goal: '#39c5cf',
  failed: new THREE.Color('#f85149'),
};

function segmentMesh(a: { x: number; y: number }, b: { x: number; y: number }, width: number, z: number) {
  const length = Math.hypot(b.x - a.x, b.y - a.y);
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(Math.max(length, 1e-3), width, 0.01),
    new THREE.MeshBasicMaterial({ color: COLORS.ahead, transparent: true, opacity: 0.85, depthWrite: false }),
  );
  mesh.position.set((a.x + b.x) / 2, (a.y + b.y) / 2, z);
  mesh.rotation.z = Math.atan2(b.y - a.y, b.x - a.x);
  return mesh;
}

export function buildRouteOverlay(route: RouteModel): RouteOverlay {
  const group = new THREE.Group();
  group.name = 'route-overlay';
  const wps = route.waypoints;

  // Raw A* cell route (fine dashed line) under the simplified route ribbon.
  const raw = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(route.path.map(([x, y]) => new THREE.Vector3(x, y, 0.012))),
    new THREE.LineDashedMaterial({ color: COLORS.raw, dashSize: 0.12, gapSize: 0.08, transparent: true, opacity: 0.8 }),
  );
  raw.computeLineDistances();
  raw.name = 'astar-raw-path';
  group.add(raw);

  const segments: THREE.Mesh[] = [];
  for (let i = 0; i + 1 < wps.length; i++) {
    const seg = segmentMesh(wps[i], wps[i + 1], 0.2, 0.02);
    seg.name = `route-segment-${i}`;
    segments.push(seg);
    group.add(seg);
  }

  // Intermediate waypoint markers, numbered in travel order.
  const markers: THREE.Mesh[] = [];
  for (let i = 1; i + 1 < wps.length; i++) {
    const marker = new THREE.Mesh(
      new THREE.CylinderGeometry(0.2, 0.2, 0.02, 24),
      new THREE.MeshBasicMaterial({ color: COLORS.ahead }),
    );
    marker.rotation.x = Math.PI / 2;
    marker.position.set(wps[i].x, wps[i].y, 0.03);
    marker.name = `waypoint-${i}`;
    markers.push(marker);
    group.add(marker);
    const label = labelSprite(String(i), 0.32, '#e6edf3');
    label.position.set(wps[i].x, wps[i].y, 0.45);
    group.add(label);
  }

  // Destination flag (snapped position), heading arrow if the goal has a yaw.
  const goal = route.destination;
  const flag = new THREE.Group();
  flag.name = 'destination-marker';
  flag.position.set(goal.x, goal.y, 0);
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.3, 0.45, 40),
    new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0.9, depthWrite: false }),
  );
  ring.position.z = 0.025;
  flag.add(ring);
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.02, 0.02, 1.2, 12),
    new THREE.MeshStandardMaterial({ color: '#c9d1d9' }),
  );
  pole.rotation.x = Math.PI / 2;
  pole.position.z = 0.6;
  flag.add(pole);
  const banner = new THREE.Mesh(
    new THREE.BoxGeometry(0.32, 0.02, 0.2),
    new THREE.MeshStandardMaterial({ color: COLORS.goal, emissive: new THREE.Color(COLORS.goal).multiplyScalar(0.4) }),
  );
  banner.position.set(0.17, 0, 1.08);
  flag.add(banner);
  if (goal.yaw !== null) {
    const shape = new THREE.Shape();
    shape.moveTo(0.55, 0);
    shape.lineTo(0.3, 0.12);
    shape.lineTo(0.3, -0.12);
    shape.closePath();
    const arrow = new THREE.Mesh(
      new THREE.ShapeGeometry(shape),
      new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0.9, depthWrite: false }),
    );
    arrow.rotation.z = goal.yaw;
    arrow.position.z = 0.026;
    flag.add(arrow);
  }
  const name = labelSprite(goal.label, 0.34, '#39c5cf');
  name.position.set(0, 0, 1.45);
  flag.add(name);
  group.add(flag);

  // A floor pick that was snapped: show where the operator clicked.
  if (goal.snapped) {
    const req = new THREE.Vector3(goal.requested_x, goal.requested_y, 0.03);
    const link = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([req, new THREE.Vector3(goal.x, goal.y, 0.03)]),
      new THREE.LineDashedMaterial({ color: '#f85149', dashSize: 0.06, gapSize: 0.05 }),
    );
    link.computeLineDistances();
    group.add(link);
    const cross = new THREE.Mesh(
      new THREE.RingGeometry(0.06, 0.1, 4),
      new THREE.MeshBasicMaterial({ color: '#f85149', depthWrite: false }),
    );
    cross.position.copy(req);
    cross.rotation.z = Math.PI / 4;
    group.add(cross);
  }

  const setProgress = (activeIndex: number | null, status: string) => {
    const failed = status === 'failed' || status === 'cancelled';
    const arrived = status === 'arrived';
    segments.forEach((seg, i) => {
      const mat = seg.material as THREE.MeshBasicMaterial;
      // Segment i leads to waypoint i + 1.
      const done = arrived || (activeIndex !== null && i + 1 < activeIndex);
      const active = !arrived && activeIndex !== null && i + 1 === activeIndex;
      mat.color.copy(failed ? COLORS.failed : done ? COLORS.done : active ? COLORS.active : COLORS.ahead);
      mat.opacity = done ? 0.45 : 0.9;
    });
    markers.forEach((m, k) => {
      const index = k + 1;
      const mat = m.material as THREE.MeshBasicMaterial;
      const passed = arrived || (activeIndex !== null && index < activeIndex);
      mat.color.copy(passed ? COLORS.done : activeIndex === index ? COLORS.active : COLORS.ahead);
    });
  };
  setProgress(null, 'planned');
  return { group, setProgress };
}

/** Planner occupancy grid as a floor texture: amber = clearance zone, red = obstacle footprint. */
export function buildGridOverlay(grid: OccupancyGridData): THREE.Mesh {
  const cells = Uint8Array.from(atob(grid.cells), (c) => c.charCodeAt(0));
  const rgba = new Uint8Array(grid.width * grid.height * 4);
  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    if (v === 1) rgba.set([245, 179, 36, 80], i * 4);
    else if (v === 2) rgba.set([248, 81, 73, 140], i * 4);
  }
  const texture = new THREE.DataTexture(rgba, grid.width, grid.height, THREE.RGBAFormat);
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  const w = grid.width * grid.resolution;
  const h = grid.height * grid.resolution;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false }),
  );
  // Row 0 of the texture is the grid's southern row, matching the plane's v = 0 edge.
  mesh.position.set(grid.origin[0] + w / 2, grid.origin[1] + h / 2, 0.008);
  mesh.name = 'occupancy-grid';
  return mesh;
}
