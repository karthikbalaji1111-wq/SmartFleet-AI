import { useEffect, useRef, useState, type RefObject } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { SimulationSnapshot, WarehouseConfigResponse } from '../api/types';
import { applyRobotState, buildRobot } from './buildRobot';
import { buildContainer, buildRobotMarker, buildWarehouse } from './buildWarehouse';
import { disposeObject } from './three-utils';

interface Props {
  data: WarehouseConfigResponse | null;
  snapshotRef: RefObject<SimulationSnapshot | null>;
  hasTelemetry: boolean;
}

interface ViewApi {
  overview: () => void;
  top: () => void;
}

/** World (Z up) -> Three.js (Y up): (x, y, z) -> (x, z, -y). */
const toThree = (x: number, y: number, z: number) => new THREE.Vector3(x, z, -y);

export function WarehouseViewport({ data, snapshotRef, hasTelemetry }: Props) {
  const mountRef = useRef<HTMLDivElement>(null);
  const viewApi = useRef<ViewApi | null>(null);
  const followRef = useRef(false);
  const [follow, setFollow] = useState(false);

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
      controls.target.copy(toThree(0, 0, 0));
      camera.position.copy(toThree(0, -0.01, Math.max(sx, sy) * 1.25));
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
    const container = buildContainer(data.config.warehouse.container.size);
    world.add(container);
    const marker = buildRobotMarker(0.62);
    world.add(marker);

    // ---- sizing ----
    const resize = () => {
      const { clientWidth: w, clientHeight: h } = mount;
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(mount);
    resize();

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
        const [cx, cy, cz] = state.container.position;
        const [qx, qy, qz, qw] = state.container.orientation;
        container.position.set(cx, cy, cz);
        container.quaternion.set(qx, qy, qz, qw);
        container.visible = true;

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
      } else {
        rig.root.visible = false;
        marker.visible = false;
        container.visible = false;
        haveLast = false;
      }
      controls.update();
      renderer.render(scene, camera);
    };
    tick();

    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      controls.dispose();
      disposeObject(scene);
      renderer.dispose();
      renderer.domElement.remove();
      viewApi.current = null;
    };
  }, [data, snapshotRef]);

  return (
    <div className="viewport">
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
      </div>
      <div className="viewport-hint">Drag: orbit · Right-drag: pan · Wheel: zoom</div>
      {!data && <div className="viewport-overlay">Waiting for backend — loading warehouse configuration…</div>}
      {data && !hasTelemetry && (
        <div className="viewport-overlay viewport-overlay--soft">
          No live telemetry — robot hidden until the backend reports its physics state.
        </div>
      )}
    </div>
  );
}
