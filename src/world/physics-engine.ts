// ===========================================================================
// Physics Engine
// Applies motor commands to robots, handles collision detection against
// the ground-truth world. Only the server runs this — robots never see it.
// ===========================================================================

import type { MotorCommand, Pose2D, RobotPhysicsConfig } from "../shared/types";
import { normalizeAngle, clamp } from "../shared/math";
import { GroundTruthWorld } from "./ground-truth-map";

// ---------------------------------------------------------------------------
// Robot Physical State (server-side only)
// ---------------------------------------------------------------------------

export interface PhysicalRobotState {
  id: string;
  pose: Pose2D;
  /** Last tick's pose (for computing actual movement) */
  prev_pose: Pose2D;
  /** Whether the robot collided this tick */
  collided: boolean;
}

// ---------------------------------------------------------------------------
// Physics Engine
// ---------------------------------------------------------------------------

export class PhysicsEngine {
  private readonly world: GroundTruthWorld;
  private readonly config: RobotPhysicsConfig;
  private readonly dt: number; // seconds per tick

  constructor(world: GroundTruthWorld, config: RobotPhysicsConfig, tickIntervalMs: number) {
    this.world = world;
    this.config = config;
    this.dt = tickIntervalMs / 1000;
  }

  /**
   * Apply a motor command to a robot's physical state.
   * Uses differential drive kinematics with collision detection.
   * Returns the updated state (mutates in place for efficiency).
   */
  applyMotorCommand(robot: PhysicalRobotState, command: MotorCommand): PhysicalRobotState {
    // Save previous pose
    robot.prev_pose = { ...robot.pose };
    robot.collided = false;

    // Clamp commanded velocities to physical limits
    const linearVel = clamp(
      command.linear_velocity,
      -this.config.max_linear_speed,
      this.config.max_linear_speed,
    );
    const angularVel = clamp(
      command.angular_velocity,
      -this.config.max_angular_speed,
      this.config.max_angular_speed,
    );

    // Compute movement in world frame using mid-point integration
    // (more accurate than Euler for curved paths)
    const halfDtheta = angularVel * this.dt * 0.5;
    const midTheta = robot.pose.theta + halfDtheta;

    const dx = linearVel * this.dt * Math.cos(midTheta);
    const dy = linearVel * this.dt * Math.sin(midTheta);
    const dtheta = angularVel * this.dt;

    // Candidate new position
    const newX = robot.pose.x + dx;
    const newY = robot.pose.y + dy;
    const newTheta = normalizeAngle(robot.pose.theta + dtheta);

    // Collision detection: check if the robot's circle at the new position
    // overlaps any wall
    if (this.world.isCircleTraversable(newX, newY, this.config.radius)) {
      // No collision — accept movement
      robot.pose.x = newX;
      robot.pose.y = newY;
      robot.pose.theta = newTheta;
    } else {
      // Collision — try sliding along each axis independently
      robot.collided = true;

      // Try each axis independently (not else-if — both can slide)
      if (this.world.isCircleTraversable(newX, robot.pose.y, this.config.radius)) {
        robot.pose.x = newX;
      }
      if (this.world.isCircleTraversable(robot.pose.x, newY, this.config.radius)) {
        robot.pose.y = newY;
      }

      // Always allow rotation (turning in place doesn't cause collision)
      robot.pose.theta = newTheta;
    }

    return robot;
  }

  /**
   * Compute the actual movement in the robot's LOCAL frame between
   * prev_pose and current pose. This is what the odometry simulator uses.
   */
  getActualLocalMovement(robot: PhysicalRobotState): { dx: number; dy: number; dtheta: number } {
    const worldDx = robot.pose.x - robot.prev_pose.x;
    const worldDy = robot.pose.y - robot.prev_pose.y;
    const dtheta = normalizeAngle(robot.pose.theta - robot.prev_pose.theta);

    // Transform world-frame displacement into the robot's previous local frame
    const c = Math.cos(-robot.prev_pose.theta);
    const s = Math.sin(-robot.prev_pose.theta);
    const localDx = worldDx * c - worldDy * s;
    const localDy = worldDx * s + worldDy * c;

    return { dx: localDx, dy: localDy, dtheta };
  }

  /**
   * Initialize a robot at a world position.
   */
  createRobot(id: string, worldX: number, worldY: number, heading: number): PhysicalRobotState {
    const pose: Pose2D = { x: worldX + 0.5, y: worldY + 0.5, theta: heading };
    return {
      id,
      pose: { ...pose },
      prev_pose: { ...pose },
      collided: false,
    };
  }
}
