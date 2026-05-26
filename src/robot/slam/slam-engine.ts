// ===========================================================================
// SLAM Engine
// Orchestrates the full SLAM pipeline: particle filter + occupancy grid +
// scan matching. Provides the robot's pose estimate and the built map.
// ===========================================================================

import type {
  LidarScan,
  OdometryReading,
  OccupancyGrid,
  Pose2D,
  Vec2,
  SimulationConfig,
} from "../../shared/types";
import { SlamParticleFilter, DEFAULT_PARTICLE_FILTER_CONFIG } from "./particle-filter";
import type { SlamResult, ParticleFilterConfig } from "./particle-filter";
import { gridStats, getCellAtWorld, worldToGrid, gridToWorld, gridInBounds } from "./occupancy-grid";
import { TypedEventEmitter } from "../../shared/events";

// ---------------------------------------------------------------------------
// SLAM Engine Events
// ---------------------------------------------------------------------------

export interface SlamEngineEvents {
  /** Emitted after every SLAM step with the result */
  "step": SlamResult;
  /** Emitted when the grid has grown significantly (for external monitoring) */
  "grid_update": { explored_cells: number; total_cells: number };
}

// ---------------------------------------------------------------------------
// SLAM Engine
// ---------------------------------------------------------------------------

export class SlamEngine {
  readonly events: TypedEventEmitter<SlamEngineEvents>;
  private particleFilter: SlamParticleFilter;
  private stepCount: number = 0;
  private totalExploredCells: number = 0;
  private lastPose: Pose2D = { x: 0, y: 0, theta: 0 };

  constructor(config?: Partial<ParticleFilterConfig>) {
    this.events = new TypedEventEmitter();
    const pfConfig: ParticleFilterConfig = {
      ...DEFAULT_PARTICLE_FILTER_CONFIG,
      ...config,
    };
    this.particleFilter = new SlamParticleFilter(pfConfig);
  }

  /**
   * Initialize the SLAM engine. Robot starts at the SLAM origin.
   * (The robot doesn't know its real-world position.)
   */
  initialize(): void {
    this.particleFilter.initialize({ x: 0, y: 0, theta: 0 });
    this.stepCount = 0;
    this.totalExploredCells = 0;
    this.lastPose = { x: 0, y: 0, theta: 0 };
  }

  /** Whether the engine has been initialized */
  get isInitialized(): boolean {
    return this.particleFilter.isInitialized;
  }

  /** Current best pose estimate */
  get pose(): Pose2D {
    return this.particleFilter.bestPose;
  }

  /** Current occupancy grid */
  get grid(): OccupancyGrid {
    return this.particleFilter.occupancyGrid;
  }

  /** Total SLAM steps processed */
  get steps(): number {
    return this.stepCount;
  }

  /**
   * Process one tick of sensor data through the SLAM pipeline.
   * This is the main entry point called every tick by the robot agent.
   */
  processSensorData(odometry: OdometryReading, lidar: LidarScan): SlamResult {
    if (!this.isInitialized) {
      this.initialize();
    }

    this.stepCount++;
    this.lastPose = { ...this.particleFilter.bestPose };

    const result = this.particleFilter.processStep(odometry, lidar);

    // Track exploration progress
    const stats = gridStats(result.grid);
    const explored = stats.free + stats.occupied;
    if (explored > this.totalExploredCells + 10) {
      this.totalExploredCells = explored;
      this.events.emit("grid_update", {
        explored_cells: explored,
        total_cells: stats.total,
      });
    }

    this.events.emit("step", result);
    return result;
  }

  /**
   * Get the number of cells the robot considers "explored" (known free or occupied).
   */
  getExploredCellCount(): number {
    const stats = gridStats(this.grid);
    return stats.free + stats.occupied;
  }

  /**
   * Check if a world position (in SLAM frame) is free in the occupancy grid.
   */
  isPositionFree(x: number, y: number): boolean {
    const logOdds = getCellAtWorld(this.grid, x, y);
    return logOdds < -0.5;
  }

  /**
   * Check if a world position (in SLAM frame) is occupied.
   */
  isPositionOccupied(x: number, y: number): boolean {
    const logOdds = getCellAtWorld(this.grid, x, y);
    return logOdds > 0.5;
  }

  /**
   * Check if a world position (in SLAM frame) is unknown.
   */
  isPositionUnknown(x: number, y: number): boolean {
    const logOdds = getCellAtWorld(this.grid, x, y);
    return Math.abs(logOdds) <= 0.5;
  }

  /**
   * Get the distance the robot has moved since last step (for treadmill tracking).
   */
  getLastMovementDistance(): number {
    const dx = this.pose.x - this.lastPose.x;
    const dy = this.pose.y - this.lastPose.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
   * Get the heading change since last step.
   */
  getLastHeadingChange(): number {
    let diff = this.pose.theta - this.lastPose.theta;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff <= -Math.PI) diff += 2 * Math.PI;
    return diff;
  }

  /**
   * Run scan matching at a node arrival. Only call this when the robot
   * has stopped at a node and new topology is available.
   */
  refinePoseAtNode(scan: LidarScan): { dx: number; dy: number; score: number } {
    return this.particleFilter.refinePoseAtNode(scan);
  }

  /**
   * Cast a ray through the SLAM grid from the current pose.
   * Returns the distance to the first occupied cell, or maxRange.
   * Used by feature detection to analyze the local environment.
   */
  castRayInGrid(angle: number, maxRange: number): number {
    const { grid } = this;
    const { config } = grid;
    const pose = this.pose;

    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    const stepSize = config.resolution;
    const maxSteps = Math.ceil(maxRange / stepSize);

    for (let i = 1; i <= maxSteps; i++) {
      const dist = i * stepSize;
      const wx = pose.x + dx * dist;
      const wy = pose.y + dy * dist;

      const { gx, gy } = worldToGrid(wx, wy, config);
      if (!gridInBounds(gx, gy, config)) return dist;

      const logOdds = grid.cells[gy * config.width + gx];
      if (logOdds > 0.5) return dist; // hit occupied cell
    }

    return maxRange;
  }

  /**
   * A* pathfinding on the occupancy grid.
   * Returns a list of world-coordinate waypoints from start to goal,
   * or null if no path exists. Only traverses free cells.
   */
  findGridPath(
    fromX: number, fromY: number,
    toX: number, toY: number,
  ): Vec2[] | null {
    const { config, cells } = this.grid;
    const start = worldToGrid(fromX, fromY, config);
    const goal = worldToGrid(toX, toY, config);

    if (!gridInBounds(start.gx, start.gy, config)) return null;
    if (!gridInBounds(goal.gx, goal.gy, config)) return null;

    const key = (x: number, y: number) => y * config.width + x;
    const startKey = key(start.gx, start.gy);
    const goalKey = key(goal.gx, goal.gy);

    const gScore = new Map<number, number>();
    const fScore = new Map<number, number>();
    const cameFrom = new Map<number, number>();
    const open = new Set<number>();
    const closed = new Set<number>();

    gScore.set(startKey, 0);
    const heuristic = (x: number, y: number) =>
      Math.abs(x - goal.gx) + Math.abs(y - goal.gy);
    fScore.set(startKey, heuristic(start.gx, start.gy));
    open.add(startKey);

    const dirs = [
      { dx: 1, dy: 0, cost: 1 },
      { dx: -1, dy: 0, cost: 1 },
      { dx: 0, dy: 1, cost: 1 },
      { dx: 0, dy: -1, cost: 1 },
      { dx: 1, dy: 1, cost: 1.414 },
      { dx: -1, dy: 1, cost: 1.414 },
      { dx: 1, dy: -1, cost: 1.414 },
      { dx: -1, dy: -1, cost: 1.414 },
    ];

    let iterations = 0;
    const maxIterations = 50000;

    while (open.size > 0 && iterations < maxIterations) {
      iterations++;

      // Find lowest fScore in open set
      let currentKey = -1;
      let lowestF = Infinity;
      for (const k of open) {
        const f = fScore.get(k) ?? Infinity;
        if (f < lowestF) { lowestF = f; currentKey = k; }
      }

      if (currentKey === goalKey) {
        // Reconstruct path
        const path: Vec2[] = [];
        let ck = goalKey;
        while (ck !== startKey) {
          const cy = Math.floor(ck / config.width);
          const cx = ck % config.width;
          path.unshift(gridToWorld(cx, cy, config));
          const prev = cameFrom.get(ck);
          if (prev === undefined) break;
          ck = prev;
        }
        // Simplify: skip every other waypoint for smoother movement
        const simplified: Vec2[] = [];
        for (let i = 0; i < path.length; i += 3) {
          simplified.push(path[i]);
        }
        if (simplified.length === 0 || simplified[simplified.length - 1] !== path[path.length - 1]) {
          simplified.push(path[path.length - 1]);
        }
        return simplified;
      }

      open.delete(currentKey);
      closed.add(currentKey);

      const curY = Math.floor(currentKey / config.width);
      const curX = currentKey % config.width;

      for (const dir of dirs) {
        const nx = curX + dir.dx;
        const ny = curY + dir.dy;
        if (!gridInBounds(nx, ny, config)) continue;

        const nk = key(nx, ny);
        if (closed.has(nk)) continue;

        // Cell must be KNOWN FREE (log-odds < -0.3). Unknown and occupied are blocked.
        const cellVal = cells[nk];
        if (cellVal > -0.3) continue;

        const tentativeG = (gScore.get(currentKey) ?? Infinity) + dir.cost;
        if (tentativeG < (gScore.get(nk) ?? Infinity)) {
          cameFrom.set(nk, currentKey);
          gScore.set(nk, tentativeG);
          fScore.set(nk, tentativeG + heuristic(nx, ny));
          open.add(nk);
        }
      }
    }

    return null; // no path found
  }

  /**
   * Cast a ray from a specific position (not just current pose).
   * Used by feature detection.
   */
  castRayFrom(startX: number, startY: number, angle: number, maxRange: number): number {
    const { grid } = this;
    const { config } = grid;

    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    const stepSize = config.resolution;
    const maxSteps = Math.ceil(maxRange / stepSize);

    for (let i = 1; i <= maxSteps; i++) {
      const dist = i * stepSize;
      const wx = startX + dx * dist;
      const wy = startY + dy * dist;

      const { gx, gy } = worldToGrid(wx, wy, config);
      if (!gridInBounds(gx, gy, config)) return dist;

      const logOdds = grid.cells[gy * config.width + gx];
      if (logOdds > 0.5) return dist;
    }

    return maxRange;
  }
}
