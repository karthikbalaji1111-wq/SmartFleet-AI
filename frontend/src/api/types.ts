// TypeScript mirrors of the backend's Pydantic models (backend/app/schemas.py,
// simulation/state.py, simulation/config.py). Coordinates are in the world
// frame: metres, Z up, yaw CCW from +X.

export type Vec2 = [number, number];
export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // x, y, z, w

// ---------------------------------------------------------------- config ---
export interface ZoneConfig {
  id: string;
  kind: 'loading' | 'delivery' | 'home';
  label: string;
  center: Vec2;
  size: Vec2;
}

export interface RackConfig {
  id: string;
  type: string;
  center: Vec2;
  axis: 'x' | 'y';
}

export interface RackType {
  length: number;
  depth: number;
  height: number;
  bays: number;
  post_size: number;
  shelf_thickness: number;
  levels: number[];
}

export interface MotionLimits {
  max_linear_velocity: number;
  max_angular_velocity: number;
  max_linear_acceleration: number;
  max_angular_acceleration: number;
  max_linear_deceleration: number;
  max_angular_deceleration: number;
  min_command_duration: number;
  max_command_duration: number;
  default_command_duration: number;
}

export interface ActuatorConfig {
  lower: number;
  upper: number;
  default_position: number;
  position_tolerance: number;
  max_jog_step: number;
  max_velocity: number;
  max_force: number;
}

export type MechanismName = 'lift' | 'forks';

export interface LiftPreset {
  id: string;
  label: string;
  position: number;
  surface_height: number;
}

export interface AppConfig {
  frame: { up_axis: 'z'; description: string };
  warehouse: {
    name: string;
    floor: { size_x: number; size_y: number; grid_spacing: number };
    walls: { height: number; thickness: number };
    rack_types: Record<string, RackType>;
    racks: RackConfig[];
    zones: ZoneConfig[];
    stations: { id: string; zone: string; center: Vec2; size: Vec3 }[];
    container: { id: string; size: Vec3; mass: number; station: string };
  };
  robot: {
    id: string;
    name: string;
    start_pose: { x: number; y: number; yaw: number };
    wheels: { radius: number; track_width: number };
    lift: ActuatorConfig;
    forks: ActuatorConfig;
    limits: MotionLimits;
  };
  simulation: { physics_hz: number; telemetry_hz: number };
}

export interface StaticBox {
  id: string;
  kind: 'wall' | 'rack_post' | 'rack_shelf' | 'station';
  group: string;
  center: Vec3;
  size: Vec3;
  level: number | null;
}

export interface RobotShape {
  name: string;
  kind: 'box' | 'cylinder' | 'sphere';
  size: number[];
  xyz: Vec3;
  rpy: Vec3;
  material: string;
  collision: boolean;
}

export interface RobotLink {
  name: string;
  mass: number;
  com: Vec3;
  shapes: RobotShape[];
}

export interface RobotJoint {
  name: string;
  type: 'fixed' | 'continuous' | 'prismatic';
  parent: string;
  child: string;
  xyz: Vec3;
  rpy: Vec3;
  axis: Vec3;
  lower: number | null;
  upper: number | null;
}

export interface RobotModel {
  name: string;
  base_link: string;
  links: RobotLink[];
  joints: RobotJoint[];
  materials: Record<string, [number, number, number, number]>;
}

export interface WarehouseConfigResponse {
  config: AppConfig;
  static_geometry: StaticBox[];
  robot_model: RobotModel;
  container_initial_position: Vec3;
  lift_presets: LiftPreset[];
}

// ------------------------------------------------------------- telemetry ---
export interface JointTelemetry {
  position: number;
  velocity: number;
  applied_effort: number;
}

/** Lift or fork prismatic joint, measured by PyBullet. */
export interface MechanismTelemetry {
  position: number;
  velocity: number;
  target: number;
  error: number;
  at_target: boolean;
  state: 'holding' | 'moving' | 'blocked';
  fault: 'stalled' | 'overload' | 'tilt' | null;
  applied_force: number;
  lower: number;
  upper: number;
  default: number;
  max_velocity: number;
}

export interface RobotState {
  id: string;
  pose: {
    position: Vec3;
    orientation: Quat;
    heading: number;
    heading_deg: number;
    roll: number;
    pitch: number;
  };
  velocity: {
    forward: number;
    lateral: number;
    yaw_rate: number;
    linear_world: Vec3;
    angular_world: Vec3;
  };
  wheels: {
    left: JointTelemetry;
    right: JointTelemetry;
    target_left: number;
    target_right: number;
    brake_engaged: boolean;
  };
  lift: MechanismTelemetry;
  forks: MechanismTelemetry;
  fork_surface_height: number;
  command: {
    active: boolean;
    target_linear: number;
    target_angular: number;
    linear: number;
    angular: number;
    remaining: number;
  };
}

export interface BodyState {
  id: string;
  position: Vec3;
  orientation: Quat;
}

export interface WorldState {
  sim_time: number;
  step: number;
  robot: RobotState;
  container: BodyState;
}

export type SimulationStatus = 'stopped' | 'running' | 'paused';

export interface SimulationSnapshot {
  status: SimulationStatus;
  ready: boolean;
  physics_hz: number;
  timestep: number;
  world: WorldState | null;
  last_event_id: number;
}

export interface ServerEvent {
  id: number;
  timestamp: string;
  sim_time: number | null;
  level: 'info' | 'warning' | 'error';
  source: string;
  message: string;
}

export interface TelemetryMessage {
  type: 'telemetry';
  snapshot: SimulationSnapshot;
  events: ServerEvent[];
}

export interface ControlResponse {
  message: string;
  snapshot: SimulationSnapshot;
}

export interface MechanismCommandResponse {
  accepted: boolean;
  mechanism: MechanismName;
  target: number;
  previous_target: number;
  position: number;
  clamped: boolean;
  message: string;
}

export interface ReadyResponse {
  ready: boolean;
  pybullet_available: boolean;
  world_initialized: boolean;
  detail: string | null;
}
