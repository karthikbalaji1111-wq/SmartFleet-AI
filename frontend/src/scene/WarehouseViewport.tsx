import { useEffect, useRef, useState, type RefObject } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { OccupancyGridData, RouteModel, SimulationSnapshot, WarehouseConfigResponse } from '../api/types';
import { applyRobotState, buildRobot } from './buildRobot';
import { buildGridOverlay, buildRouteOverlay, type RouteOverlay } from './buildRoute';
import { buildContainer, buildRobotMarker, buildWarehouse } from './buildWarehouse';
import { disposeObject } from './three-utils';

interface Props {
  data: WarehouseConfigResponse | null;
  snapshotRef: RefObject<SimulationSnapshot | null>;
  hasTelemetry: boolean;
  route: RouteModel | null;
  grid: OccupancyGridData | null;
  showGrid: boolean;
  pickMode: boolean;
  onPick: (x: number, y: number) => void;
}

interface ViewApi {
  overview: () => void;
  top: () => void;
}

/** World (Z up) -> Three.js (Y up): (x, y, z) -> (x, z, -y). */
const toThree = (x: number, y: number, z: number) => new THREE.Vector3(x, z, -y);

export function WarehouseViewport({ data, snapshotRef, hasTelemetry, route, grid, showGrid, pickMode, onPick }: Props) {
  const mountRef = useRef<HTMLDivElement>(null);
  const viewApi = useRef<ViewApi | null>(null);
  const followRef = useRef(false);
  const [follow, setFollow] = useState(false);
  const worldRef = useRef<THREE.Group | null>(null);
  const overlayRef = useRef<RouteOverlay | null>(null);
  const pickRef = useRef({ pickMode, onPick });
  pickRef.current = { pickMode, onPick };

  useEffect(() => {
    followRef.current = follow;
  }, [follow]);

  useEffect(() => {
    const mount = mountRef.current;
    if (!data || !mount) return;
    const { size_x: sx, size_y: sy } = data.config.warehouse.floor;

    // ---- renderer / scene / camera ----
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    mount.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#0d1117');
    scene.fog = new THREE.Fog('#0d1117', 45, 90);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 200);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.screenSpacePanning = false; // pan across the floor plane
    controls.minDistance = 1.5;
    controls.maxDistance = 60;
    controls.maxPolarAngle = Math.PI * 0.495;

    const overview = () => {
      controls.target.copy(toThree(1, 0, 0));
      camera.position.copy(toThree(13, -15, 13));
      controls.update();
    };
    const top = () => {
      // Slightly tilted: a camera exactly above its target is degenerate for OrbitControls.
      controls.target.copy(toThree(0, 0, 0));
      camera.position.copy(toThree(0, -0.1, Math.max(sx, sy) * 1.25));
      controls.update();
    };
    viewApi.current = { overview, top };
    overview();

    // ---- lighting ----
    scene.add(new THREE.HemisphereLight('#dfe8f5', '#2a2f36', 1.1));
    const sun = new THREE.DirectionalLight('#ffffff', 2.2);
    sun.position.copy(toThree(-8, -10, 18));
    sun.castShadow = true;
    sun.shadow.mapSize.set(4096, 4096);
    const reach = Math.max(sx, sy) * 0.65;
    Object.assign(sun.shadow.camera, { left: -reach, right: reach, top: reach, bottom: -reach, near: 1, far: 60 });
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.02;
    scene.add(sun);

    // ---- content (built in the Z-up world frame) ----
    const world = new THREE.Group();
    world.rotation.x = -Math.PI / 2;
    scene.add(world);
    world.add(buildWarehouse(data));

    const rig = buildRobot(data.robot_model);
    world.add(rig.root);
    const containerGroups: Record<string, THREE.Group> = {};
    for (const c of data.config.warehouse.containers) {
      const g = buildContainer(c.size);
      world.add(g);
      containerGroups[c.id] = g;
    }
    const marker = buildRobotMarker(0.62);

    world.add(marker);
    worldRef.current = world;

    // ---- floor picking (destination selection): a click, not an orbit drag ----
    const floor = world.getObjectByName('floor');
    const raycaster = new THREE.Raycaster();
    let down: { x: number; y: number; t: number } | null = null;
    const onPointerDown = (e: PointerEvent) => {
      down = { x: e.clientX, y: e.clientY, t: performance.now() };
    };
    const onPointerUp = (e: PointerEvent) => {
      const start = down;
      down = null;
      if (!start || !pickRef.current.pickMode || !floor || e.button !== 0) return;
      if (Math.hypot(e.clientX - start.x, e.clientY - start.y) > 6 || performance.now() - start.t > 600) return;
      const rect = renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      raycaster.setFromCamera(ndc, camera);
      const hit = raycaster.intersectObject(floor, false)[0];
      if (hit) pickRef.current.onPick(hit.point.x, -hit.point.z); // Three.js (Y up) -> world (Z up)
    };
    renderer.domElement.addEventListener('pointerdown', onPointerDown);
    renderer.domElement.addEventListener('pointerup', onPointerUp);

    // ---- sizing ----
    const resize = (w: number, h: number) => {
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver((entries) => {
      if (!entries.length) return;
      const { width, height } = entries[0].contentRect;
      resize(width, height);
    });
    observer.observe(mount);
    resize(mount.clientWidth, mount.clientHeight);

    // ---- render loop: copy the latest physics state, never invent motion ----
    let frame = 0;
    const lastRobot = new THREE.Vector3();
    let haveLast = false;
    const tick = () => {
      frame = requestAnimationFrame(tick);
      const state = snapshotRef.current?.world;
      if (state) {
        applyRobotState(rig, state.robot);
        const [rx, ry] = state.robot.pose.position;
        marker.position.set(rx, ry, 0.009);
        marker.visible = true;
        for (const cState of state.containers) {
          const g = containerGroups[cState.id];
          if (g) {
            const [cx, cy, cz] = cState.position;
            const [qx, qy, qz, qw] = cState.orientation;
            g.position.set(cx, cy, cz);
            g.quaternion.set(qx, qy, qz, qw);
            g.visible = true;
          }
        }

        const robotThree = toThree(rx, ry, 0.4);
        if (followRef.current && haveLast) {
          const delta = robotThree.clone().sub(lastRobot);
          camera.position.add(delta);
          controls.target.add(delta);
        }
        if (followRef.current && !haveLast) {
          const offset = camera.position.clone().sub(controls.target);
          controls.target.copy(robotThree);
          camera.position.copy(robotThree).add(offset.setLength(Math.min(offset.length(), 9)));
        }
        lastRobot.copy(robotThree);
        haveLast = followRef.current;
        if (overlayRef.current) {
          overlayRef.current.group.visible = true;
          overlayRef.current.setProgress(state.navigation.waypoint_index, state.navigation.status);
        }
      } else {
        rig.root.visible = false;
        marker.visible = false;
        Object.values(containerGroups).forEach((g) => (g.visible = false));

        haveLast = false;
        if (overlayRef.current) overlayRef.current.group.visible = false;
      }
      controls.update();
      renderer.render(scene, camera);
    };
    tick();

    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      renderer.domElement.removeEventListener('pointerdown', onPointerDown);
      renderer.domElement.removeEventListener('pointerup', onPointerUp);
      controls.dispose();
      disposeObject(scene);
      renderer.dispose();
      renderer.domElement.remove();
      viewApi.current = null;
      worldRef.current = null;
      overlayRef.current = null;
    };
  }, [data, snapshotRef]);

  // Route overlay: rebuilt only when the backend reports a new route.
  useEffect(() => {
    const world = worldRef.current;
    if (!world || !route) return;
    const overlay = buildRouteOverlay(route);
    world.add(overlay.group);
    overlayRef.current = overlay;
    return () => {
      world.remove(overlay.group);
      disposeObject(overlay.group);
      if (overlayRef.current === overlay) overlayRef.current = null;
    };
  }, [route, data]);

  // Planner occupancy grid overlay (static map), toggled from the navigation panel.
  useEffect(() => {
    const world = worldRef.current;
    if (!world || !grid || !showGrid) return;
    const mesh = buildGridOverlay(grid);
    world.add(mesh);
    return () => {
      world.remove(mesh);
      disposeObject(mesh);
    };
  }, [grid, showGrid, data]);

  return (
    <div className={`viewport ${pickMode ? 'viewport--pick' : ''}`}>
      <div ref={mountRef} className="viewport-canvas" />
      <div className="viewport-toolbar">
        <button type="button" className="btn btn-ghost" onClick={() => viewApi.current?.overview()} disabled={!data}>
          Overview
        </button>
        <button type="button" className="btn btn-ghost" onClick={() => viewApi.current?.top()} disabled={!data}>
          Top
        </button>
        <button
          type="button"
          className={`btn btn-ghost ${follow ? 'is-active' : ''}`}
          aria-pressed={follow}
          onClick={() => setFollow((f) => !f)}
          disabled={!data}
        >
          Follow robot
        </button>
      </div>
      <div className="viewport-legend">
        <span>
          <i style={{ background: '#2f6fd6' }} /> Rack upright
        </span>
        <span>
          <i style={{ background: '#f07c1b' }} /> Shelf level
        </span>
        <span>
          <i style={{ background: '#f5b324' }} /> Loading
        </span>
        <span>
          <i style={{ background: '#3fb950' }} /> Delivery
        </span>
        <span>
          <i style={{ background: '#39c5cf' }} /> Robot home
        </span>
        <span>
          <i style={{ background: '#3d7bd9' }} /> Container
        </span>
        {route && (
          <span>
            <i style={{ background: '#58a6ff' }} /> A* route
          </span>
        )}
      </div>
      <div className={`viewport-hint ${pickMode ? 'viewport-hint--pick' : ''}`}>
        {pickMode
          ? 'Click the floor to set a destination · drag still orbits'
          : 'Drag: orbit · Right-drag: pan · Wheel: zoom'}
      </div>
      {!data && <div className="viewport-overlay">Waiting for backend — loading warehouse configuration…</div>}
      {data && !hasTelemetry && (
        <div className="viewport-overlay viewport-overlay--soft">
          No live telemetry — robot hidden until the backend reports its physics state.
        </div>
      )}
    </div>
  );
}
