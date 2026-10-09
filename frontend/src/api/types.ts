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
    containers: { id: string; size: Vec3; mass: number; location: string }[];
    slots: { id: string; rack_id: string; bay: number; level: number; center: Vec3 }[];
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
  navigation: {
    grid: { resolution: number; clearance_margin: number; goal_snap_radius: number };
    controller: { goal_tolerance: number; heading_tolerance: number; cruise_speed: number };
    travel: { max_lift: number; max_fork_extension: number };
    destinations: DestinationConfig[];
  };
}

export interface DestinationConfig {
  id: string;
  label: string;
  kind: 'home' | 'aisle' | 'corridor' | 'staging';
  x: number;
  y: number;
  yaw: number | null;
}

// ------------------------------------------------------------ navigation ---
export type NavStatus = 'idle' | 'planning' | 'planned' | 'navigating' | 'paused' | 'arrived' | 'cancelled' | 'failed';

export interface PlannerMetrics {
  status: string;
  message: string;
  length: number;
  raw_length: number;
  expanded: number;
  time_ms: number;
  waypoint_count: number;
  raw_point_count: number;
  start_snapped: boolean;
}

export interface NavDestination {
  id: string | null;
  label: string;
  x: number;
  y: number;
  yaw: number | null;
  requested_x: number;
  requested_y: number;
  snapped: boolean;
  snap_distance: number;
}

export interface RouteModel {
  version: number;
  destination: NavDestination;
  start: Vec2;
  waypoints: { x: number; y: number; heading: number }[];
  path: Vec2[];
  planner: PlannerMetrics;
}

export interface NavigationTelemetry {
  status: NavStatus;
  destination: NavDestination | null;
  route_version: number;
  waypoint_index: number | null;
  waypoint_count: number;
  phase: string | null;
  distance_to_goal: number | null;
  remaining_distance: number | null;
  route_length: number | null;
  progress: number | null;
  heading_error: number | null;
  cross_track_error: number | null;
  command_linear: number;
  command_angular: number;
  elapsed: number;
  planner: PlannerMetrics | null;
  reason: string | null;
  interlock: string[];
}

export interface OccupancyGridData {
  width: number;
  height: number;
  resolution: number;
  origin: Vec2;
  inflation_radius: number;
  robot_radius: number;
  encoding: 'base64-u8-row-major';
  cells: string;
}

export interface NavigationResponse {
  message: string;
  navigation: NavigationTelemetry;
  route: RouteModel | null;
}

export type PlanRequest = { destination_id: string } | { x: number; y: number; snap?: boolean };

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
  container_initial_positions: Record<string, Vec3>;
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
  containers: BodyState[];
  navigation: NavigationTelemetry;
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
  route_update: { version: number; route: RouteModel | null } | null;
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
