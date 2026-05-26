// ===========================================================================
// Shared Type Definitions
// Used by both the World Server and Robot Agents.
// ===========================================================================

// ---------------------------------------------------------------------------
// Geometry Primitives
// ---------------------------------------------------------------------------

export interface Vec2 {
  x: number;
  y: number;
}

export interface BoundingBox {
  min: Vec2;
  max: Vec2;
}

// ---------------------------------------------------------------------------
// Pose
// ---------------------------------------------------------------------------

export interface Pose2D {
  x: number;
  y: number;
  theta: number; // radians, heading direction
}

// ---------------------------------------------------------------------------
// Sensor Data (produced by World Server, consumed by Robot)
// ---------------------------------------------------------------------------

export interface LidarScan {
  /** Ray angles in radians, relative to robot heading. Fixed array per config. */
  angles: number[];
  /** Distance per ray in meters. Values >= max_range mean no return. */
  ranges: number[];
  /** Maximum sensor range in meters */
  max_range: number;
  /** Timestamp in ms since simulation start */
  timestamp: number;
}

export interface OdometryReading {
  /** Change in x position in robot's local frame (meters) */
  delta_x: number;
  /** Change in y position in robot's local frame (meters) */
  delta_y: number;
  /** Change in heading (radians, positive = counterclockwise) */
  delta_theta: number;
  /** Timestamp in ms since simulation start */
  timestamp: number;
}

// ---------------------------------------------------------------------------
// Motor Commands (produced by Robot, consumed by World Server)
// ---------------------------------------------------------------------------

export interface MotorCommand {
  /** Forward velocity in m/s (positive = forward, negative = reverse) */
  linear_velocity: number;
  /** Rotational velocity in rad/s (positive = counterclockwise) */
  angular_velocity: number;
  /** Timestamp in ms since simulation start */
  timestamp: number;
}

// ---------------------------------------------------------------------------
// Sensor Configuration (defines LiDAR and Odometry characteristics)
// ---------------------------------------------------------------------------

export interface LidarConfig {
  /** Number of rays per scan */
  ray_count: number;
  /** Angular range in radians (centered on robot heading). E.g. 2*PI for 360 */
  angular_range: number;
  /** Maximum range in meters */
  max_range: number;
  /** Noise standard deviation in meters (range measurement noise) */
  range_noise_stddev: number;
  /** Probability that a single ray returns a spurious reading [0,1] */
  spurious_rate: number;
}

export interface OdometryConfig {
  /** Noise as fraction of linear distance traveled (e.g. 0.02 = 2% error) */
  linear_noise_fraction: number;
  /** Noise as fraction of angular change (e.g. 0.05 = 5% error) */
  angular_noise_fraction: number;
  /** Base linear noise floor in meters (even when stationary) */
  linear_noise_floor: number;
  /** Base angular noise floor in radians */
  angular_noise_floor: number;
}

export interface RobotPhysicsConfig {
  /** Maximum forward speed in m/s */
  max_linear_speed: number;
  /** Maximum rotational speed in rad/s */
  max_angular_speed: number;
  /** Robot radius in meters (for collision detection) */
  radius: number;
  /** Wheel base width in meters (for differential drive model) */
  wheel_base: number;
}

// ---------------------------------------------------------------------------
// Occupancy Grid
// ---------------------------------------------------------------------------

export interface OccupancyGridConfig {
  /** Cell size in meters */
  resolution: number;
  /** Grid width in cells */
  width: number;
  /** Grid height in cells */
  height: number;
  /** World x-coordinate of cell [0,0] center */
  origin_x: number;
  /** World y-coordinate of cell [0,0] center */
  origin_y: number;
}

export interface OccupancyGrid {
  config: OccupancyGridConfig;
  /** Log-odds values. Positive = occupied, negative = free, zero = unknown. */
  cells: Float32Array;
}

// Log-odds constants for occupancy grid updates
export const LOG_ODDS_OCCUPIED = 0.847;  // ln(0.7 / 0.3)
export const LOG_ODDS_FREE = -0.405;     // ln(0.4 / 0.6) — conservative free
export const LOG_ODDS_MAX = 5.0;
export const LOG_ODDS_MIN = -5.0;

// ---------------------------------------------------------------------------
// SLAM Particle
// ---------------------------------------------------------------------------

export interface SlamParticle {
  pose: Pose2D;
  weight: number;
}

// ---------------------------------------------------------------------------
// Feature Detection
// ---------------------------------------------------------------------------

export enum FeatureType {
  Junction = "junction",
  Doorway = "doorway",
  DeadEnd = "dead_end",
  Corner = "corner",
  RoomEntry = "room_entry",
}

export interface DetectedFeature {
  type: FeatureType;
  /** Position in robot's SLAM coordinate frame */
  position: Vec2;
  /** Confidence 0-1 */
  confidence: number;
  /** Heading direction at detection point (radians) */
  heading: number;
  /** Angles of corridors leaving this feature (radians, in SLAM frame) */
  exit_directions: number[];
  /** Radius of the feature region for local map extraction (meters) */
  local_radius: number;
}

// ---------------------------------------------------------------------------
// Architecture-Aligned Enums
// ---------------------------------------------------------------------------

export enum StabilityClass {
  Static = "static",
  SemiStatic = "semi_static",
  Dynamic = "dynamic",
}

export enum ConfidenceTier {
  Dead = "dead",
  Candidate = "candidate",
  Provisional = "provisional",
  Committed = "committed",
  Anchored = "anchored",
}

export enum EdgeType {
  Door = "door",
  Opening = "opening",
  CorridorContinuation = "corridor_continuation",
  Stairway = "stairway",
  Ramp = "ramp",
  Conveyor = "conveyor",
  VisualOnly = "visual_only",
  Unknown = "unknown",
}

export enum EdgeStatus {
  Open = "open",
  Blocked = "blocked",
  Restricted = "restricted",
  Unknown = "unknown",
}

export enum SourceType {
  Manual = "manual",
  Sensor = "sensor",
  PlanImport = "plan_import",
  Peer = "peer",
}

// ---------------------------------------------------------------------------
// Confidence Tier Boundaries
// ---------------------------------------------------------------------------

export function confidenceTierFromScore(score: number): ConfidenceTier {
  if (score < 0.10) return ConfidenceTier.Dead;
  if (score < 0.35) return ConfidenceTier.Candidate;
  if (score < 0.65) return ConfidenceTier.Provisional;
  if (score < 0.85) return ConfidenceTier.Committed;
  return ConfidenceTier.Anchored;
}

// ---------------------------------------------------------------------------
// Observation Record (the fundamental data unit for evidence)
// ---------------------------------------------------------------------------

export interface ObservationRecord {
  id: string;
  source_id: string;
  source_type: SourceType;
  timestamp: number;
  confidence: number;
  confidence_fields: Record<string, number>;
  target_type: "node" | "edge" | "attribute";
  target_id: string | null;
  payload: ObservationPayload;
  sensor_metadata: SensorMetadata;
  supersedes: string[];
}

export interface ObservationPayload {
  position?: Vec2;
  geometry?: { area: number; perimeter: number };
  features?: { type: string; signature: string; position_relative: Vec2 }[];
  edge_distance?: number;
  bearing_change?: number;
  node_type?: FeatureType;
  edge_type?: EdgeType;
  traversability?: Traversability;
}

export interface SensorMetadata {
  sensors_used: string[];
  fusion_confidence: number;
  observation_count?: number;
  source_diversity?: number;
  confidence_hash?: string;
}

// ---------------------------------------------------------------------------
// Edge Traversability
// ---------------------------------------------------------------------------

export interface Traversability {
  clearance_width: number;
  clearance_height: number;
  surface_type: "concrete" | "metal" | "grating" | "unknown";
  slope: number;
  status: EdgeStatus;
  restrictions: string[];
}

// ---------------------------------------------------------------------------
// Network Graph (shared via Gun.js between robots)
// ---------------------------------------------------------------------------

export interface NetworkNode {
  id: string;
  type: FeatureType;
  /** Position in the discovering robot's SLAM coordinate frame */
  position: Vec2;
  /** Confidence 0-1 based on observation quality */
  confidence: number;
  /** Number of times this node has been observed/confirmed */
  observation_count: number;
  /** Number of corridors leaving this node */
  exit_count: number;
  /** Content hash of the local occupancy map (for merge matching) */
  local_map_hash: string;
  /** Robot ID that first discovered this node */
  discovered_by: string;
  /** Timestamp of discovery */
  discovered_at: number;

  // --- Exploration state (new) ---
  /** Whether a robot has arrived at this node and completed a 360 scan */
  explored: boolean;
  /** Which robots have explored this node */
  explored_by: string[];
  /** Timestamp of last exploration scan */
  last_explored_at: number;

  // --- Architecture-aligned fields (new) ---
  stability_class: StabilityClass;
  tier: ConfidenceTier;
  observation_log: string[];
  last_observed: number;
  version: number;
}

export interface NetworkEdge {
  id: string;
  source_node: string;
  target_node: string;
  /** Distance along corridor in meters (Euclidean for untraveled, measured for traveled) */
  distance: number;
  /** Total heading change when traversing source -> target (radians) */
  bearing_change: number;
  /** Whether the edge can be traversed in both directions */
  bidirectional: boolean;
  /** Confidence 0-1 */
  confidence: number;
  /** Content hash of the corridor occupancy map */
  corridor_map_hash: string;
  /** Robot ID that first discovered this edge */
  discovered_by: string;
  /** Timestamp of discovery */
  discovered_at: number;

  // --- Exploration state (new) ---
  /** Whether a robot has physically traversed this edge */
  traveled: boolean;
  /** Which robots have traveled this edge */
  traveled_by: string[];

  // --- Architecture-aligned fields (new) ---
  edge_type: EdgeType;
  traversability: Traversability;
  stability_class: StabilityClass;
  tier: ConfidenceTier;
  observation_log: string[];
  last_observed: number;
  version: number;
}

// ---------------------------------------------------------------------------
// Default constructors for new fields
// ---------------------------------------------------------------------------

export function defaultTraversability(): Traversability {
  return {
    clearance_width: Infinity,
    clearance_height: Infinity,
    surface_type: "unknown",
    slope: 0,
    status: EdgeStatus.Unknown,
    restrictions: [],
  };
}

export function defaultNodeExtensions(): Pick<NetworkNode,
  "explored" | "explored_by" | "last_explored_at" |
  "stability_class" | "tier" | "observation_log" | "last_observed" | "version"
> {
  return {
    explored: false,
    explored_by: [],
    last_explored_at: 0,
    stability_class: StabilityClass.Static,
    tier: ConfidenceTier.Candidate,
    observation_log: [],
    last_observed: 0,
    version: 1,
  };
}

export function defaultEdgeExtensions(): Pick<NetworkEdge,
  "traveled" | "traveled_by" |
  "edge_type" | "traversability" | "stability_class" | "tier" |
  "observation_log" | "last_observed" | "version"
> {
  return {
    traveled: false,
    traveled_by: [],
    edge_type: EdgeType.Unknown,
    traversability: defaultTraversability(),
    stability_class: StabilityClass.Static,
    tier: ConfidenceTier.Candidate,
    observation_log: [],
    last_observed: 0,
    version: 1,
  };
}

// ---------------------------------------------------------------------------
// Visualization Types (World Server → Browser)
// ---------------------------------------------------------------------------

export interface VizRobot {
  id: string;
  /** Primary display position: actual ground truth position */
  x: number;
  y: number;
  heading: number;
  /** Where the robot THINKS it is (SLAM pose mapped to world coords) */
  believedX: number;
  believedY: number;
  believedHeading: number;
  /** Where the robot ACTUALLY is (ground truth) — same as x/y */
  actualX: number;
  actualY: number;
  actualHeading: number;
  /** Cumulative movement error: sum of per-tick SLAM vs actual movement differences */
  slamDrift: number;
  /** Instantaneous position error: distance between believed and actual position right now */
  positionError: number;
  status: string;
  trail: Vec2[];
  cellsExplored: number;
}

export interface VizNode {
  id: string;
  x: number;
  y: number;
  type: string;
  tier: string;
  edgeCount: number;
  isFrontier: boolean;
  explored?: boolean;
}

export interface VizEdge {
  id: string;
  sourceId: string;
  targetId: string;
  tier: string;
  distance: number;
  status: string;
  traveled?: boolean;
}

export interface VizMetrics {
  coverage: number;
  totalTraversable: number;
  totalExplored: number;
  nodeCount: number;
  edgeCount: number;
  tickCount: number;
}

// ---------------------------------------------------------------------------
// Map Cell Types (ground truth world)
// ---------------------------------------------------------------------------

export enum CellType {
  Wall = "#",
  Floor = ".",
  Doorway = "D",
  Obstacle = "O",
  Pillar = "P",
  RoughTerrain = "~",
  RobotStart = "R",
}

export interface RobotStartPosition {
  id: string;
  x: number;
  y: number;
}

export interface GroundTruthMap {
  name: string;
  width: number;
  height: number;
  cells: CellType[][];
  robot_starts: RobotStartPosition[];
  difficulty: number;
  description: string;
}

// ---------------------------------------------------------------------------
// Simulation Configuration
// ---------------------------------------------------------------------------

export interface SimulationConfig {
  /** Simulation tick interval in ms */
  tick_interval_ms: number;
  /** LiDAR sensor configuration */
  lidar: LidarConfig;
  /** Odometry noise configuration */
  odometry: OdometryConfig;
  /** Robot physical properties */
  robot_physics: RobotPhysicsConfig;
  /** SLAM occupancy grid resolution in meters per cell */
  slam_resolution: number;
  /** SLAM occupancy grid size in meters (square, centered on robot start) */
  slam_grid_size: number;
  /** Number of SLAM particles */
  slam_particle_count: number;
  /** Feature detection: minimum distance between nodes (meters). Lower = more nodes in tight spaces. */
  min_node_separation: number;
  /** Feature detection: run every N ticks */
  feature_detection_interval: number;
  /** Loop closure: scan match threshold (0-1) */
  loop_closure_threshold: number;
  /** Loop closure: check every N ticks */
  loop_closure_interval: number;
}

export const DEFAULT_SIMULATION_CONFIG: SimulationConfig = {
  tick_interval_ms: 100,
  lidar: {
    ray_count: 360,
    angular_range: 2 * Math.PI,
    max_range: 8.0,
    range_noise_stddev: 0.01,
    spurious_rate: 0.002,
  },
  odometry: {
    linear_noise_fraction: 0.01,
    angular_noise_fraction: 0.015,
    linear_noise_floor: 0.0005,
    angular_noise_floor: 0.0005,
  },
  robot_physics: {
    max_linear_speed: 3.0,
    max_angular_speed: 3.0,
    radius: 0.25,
    wheel_base: 0.3,
  },
  slam_resolution: 0.1,
  slam_grid_size: 60.0,
  slam_particle_count: 30,
  min_node_separation: 2.5,
  feature_detection_interval: 5,
  loop_closure_threshold: 0.7,
  loop_closure_interval: 20,
};
