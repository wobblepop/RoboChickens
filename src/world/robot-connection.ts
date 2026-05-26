// ===========================================================================
// Robot Connection
// Manages the data stream for a single robot.
// The server sends sensor data and receives motor commands.
// In this in-process simulation, the "connection" is a direct function call
// (no actual network transport — robots run in the same process).
// ===========================================================================

import type {
  LidarScan,
  OdometryReading,
  MotorCommand,
  Pose2D,
  Vec2,
  SimulationConfig,
} from "../shared/types";
import { RobotAgent } from "../robot/robot-agent";
import { LidarSimulator } from "./lidar-simulator";
import { OdometrySimulator } from "./odometry-simulator";
import { PhysicsEngine, PhysicalRobotState } from "./physics-engine";
import { GroundTruthWorld } from "./ground-truth-map";

// ---------------------------------------------------------------------------
// Robot Connection
// ---------------------------------------------------------------------------

export class RobotConnection {
  readonly id: string;
  readonly agent: RobotAgent;
  readonly physicalState: PhysicalRobotState;

  private readonly lidarSim: LidarSimulator;
  private readonly odomSim: OdometrySimulator;
  private readonly physics: PhysicsEngine;
  private readonly world: GroundTruthWorld;

  /** Robot's starting world position (for SLAM→world coordinate transform) */
  private readonly startWorldX: number;
  private readonly startWorldY: number;

  /** Last motor command received from the robot */
  private lastCommand: MotorCommand = { linear_velocity: 0, angular_velocity: 0, timestamp: 0 };
  /** Explored cells tracked by world (for visualization accuracy) */
  private exploredWorldCells: Set<string> = new Set();
  private newlyExploredThisTick: string[] = [];

  /** Accumulated SLAM drift: total position error growth over time.
   *  Measured as distance between where robot thinks it moved vs where it actually moved,
   *  accumulated each tick. At t=0 this is 0 by definition. */
  private cumulativeDrift: number = 0;
  private prevSlamPose: Pose2D = { x: 0, y: 0, theta: 0 };
  private prevActualPose: Pose2D | null = null;

  constructor(
    id: string,
    world: GroundTruthWorld,
    physics: PhysicsEngine,
    lidarSim: LidarSimulator,
    odomSim: OdometrySimulator,
    startX: number,
    startY: number,
    config: SimulationConfig,
  ) {
    this.id = id;
    this.world = world;
    this.physics = physics;
    this.lidarSim = lidarSim;
    this.odomSim = odomSim;
    this.startWorldX = startX + 0.5;
    this.startWorldY = startY + 0.5;

    // Create the physical robot in the world
    this.physicalState = physics.createRobot(id, startX, startY, 0);

    // Create the independent robot agent (it knows NOTHING about the world)
    this.agent = new RobotAgent(id, config);
  }

  /**
   * Run one tick of the robot's lifecycle:
   * 1. Apply last motor command (physics)
   * 2. Generate sensor data from new position
   * 3. Send sensor data to robot agent
   * 4. Receive motor command for next tick
   */
  tick(timestamp: number): void {
    // Step 1: Apply the robot's last motor command to move it in the world
    this.physics.applyMotorCommand(this.physicalState, this.lastCommand);

    // Step 2: Generate sensor data from the robot's TRUE position
    const actualMovement = this.physics.getActualLocalMovement(this.physicalState);
    const odometry = this.odomSim.reading(
      actualMovement.dx,
      actualMovement.dy,
      actualMovement.dtheta,
      timestamp,
    );
    const lidar = this.lidarSim.scan(this.world, this.physicalState.pose, timestamp);

    // Step 3: Feed sensor data to the robot agent and get motor command
    // THE ROBOT ONLY SEES: lidar scan + odometry reading
    // It has NO access to physicalState, world, or anything else
    this.lastCommand = this.agent.processTick(lidar, odometry);

    // Step 4: Track SLAM drift — compare per-tick movement in SLAM vs reality
    const currSlamPose = this.agent.pose;
    const currActualPose = this.physicalState.pose;
    if (this.prevActualPose) {
      const slamDx = currSlamPose.x - this.prevSlamPose.x;
      const slamDy = currSlamPose.y - this.prevSlamPose.y;
      const actualDx = currActualPose.x - this.prevActualPose.x;
      const actualDy = currActualPose.y - this.prevActualPose.y;
      const movementErrorX = slamDx - actualDx;
      const movementErrorY = slamDy - actualDy;
      this.cumulativeDrift += Math.sqrt(movementErrorX * movementErrorX + movementErrorY * movementErrorY);
    }
    this.prevSlamPose = { ...currSlamPose };
    this.prevActualPose = { ...currActualPose };

    // Step 5: Track explored cells for visualization
    this.trackExploredCells();
  }

  /**
   * Get the robot's actual position (for visualization only — robot doesn't see this).
   */
  getActualPose(): Pose2D {
    return this.physicalState.pose;
  }

  /**
   * Get newly explored world cells this tick (for visualization delta updates).
   */
  getNewlyExploredCells(): string[] {
    return this.newlyExploredThisTick;
  }

  /**
   * Get total explored cell count.
   */
  getExploredCellCount(): number {
    return this.exploredWorldCells.size;
  }

  /**
   * Cumulative SLAM drift: sum of per-tick movement errors.
   * 0 at start; grows when SLAM movement disagrees with actual movement.
   */
  getCumulativeDrift(): number {
    return this.cumulativeDrift;
  }

  /**
   * Instantaneous position error: distance between where robot thinks
   * it is and where it actually is right now.
   */
  getPositionError(): number {
    const slam = this.agent.pose;
    const worldFromSlam = this.slamToWorld(slam.x, slam.y);
    const actual = this.physicalState.pose;
    const dx = worldFromSlam.x - actual.x;
    const dy = worldFromSlam.y - actual.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
   * Transform a position from the robot's SLAM frame to world coordinates.
   * SLAM frame origin (0,0) = robot's actual starting world position.
   */
  slamToWorld(slamX: number, slamY: number): Vec2 {
    return {
      x: slamX + this.startWorldX,
      y: slamY + this.startWorldY,
    };
  }

  /**
   * Transform a world coordinate to the robot's SLAM frame.
   */
  worldToSlam(worldX: number, worldY: number): Vec2 {
    return {
      x: worldX - this.startWorldX,
      y: worldY - this.startWorldY,
    };
  }

  /** Reset */
  reset(startX: number, startY: number): void {
    const pose: Pose2D = { x: startX + 0.5, y: startY + 0.5, theta: 0 };
    this.physicalState.pose = { ...pose };
    this.physicalState.prev_pose = { ...pose };
    this.physicalState.collided = false;
    this.lastCommand = { linear_velocity: 0, angular_velocity: 0, timestamp: 0 };
    this.exploredWorldCells.clear();
    this.newlyExploredThisTick = [];
    this.cumulativeDrift = 0;
    this.prevSlamPose = { x: 0, y: 0, theta: 0 };
    this.prevActualPose = null;
    this.agent.reset();
  }

  // -------------------------------------------------------------------------
  // Private
  // -------------------------------------------------------------------------

  /**
   * Track which world cells the robot can currently see (for visualization).
   * Uses the robot's actual position to determine visible cells.
   */
  private trackExploredCells(): void {
    this.newlyExploredThisTick = [];
    const pose = this.physicalState.pose;
    const sensorRange = 8; // matches LiDAR max range

    // Check cells within sensor range
    const cx = Math.floor(pose.x);
    const cy = Math.floor(pose.y);
    for (let dy = -sensorRange; dy <= sensorRange; dy++) {
      for (let dx = -sensorRange; dx <= sensorRange; dx++) {
        const gx = cx + dx;
        const gy = cy + dy;
        if (!this.world.inBounds(gx, gy)) continue;
        if (!this.world.isTraversable(gx, gy)) continue;

        const distSq = dx * dx + dy * dy;
        if (distSq > sensorRange * sensorRange) continue;

        // Check line of sight from robot to cell
        if (!this.world.hasLineOfSight(cx, cy, gx, gy)) continue;

        const key = `${gx},${gy}`;
        if (!this.exploredWorldCells.has(key)) {
          this.exploredWorldCells.add(key);
          this.newlyExploredThisTick.push(key);
        }
      }
    }
  }
}
