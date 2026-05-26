import type { MotorCommand, Pose2D, Vec2, LidarScan } from "../../shared/types";
import { normalizeAngle } from "../../shared/math";

export interface MotorControllerConfig {
  max_linear_speed: number;
  max_angular_speed: number;
  ahead_clear_threshold: number;
  robot_radius: number;
}

export const DEFAULT_MOTOR_CONFIG: MotorControllerConfig = {
  max_linear_speed: 2.5,
  max_angular_speed: 8.0,
  ahead_clear_threshold: 0.8,
  robot_radius: 0.25,
};

export class MotorController {
  private readonly config: MotorControllerConfig;

  constructor(config: MotorControllerConfig = DEFAULT_MOTOR_CONFIG) {
    this.config = config;
  }

  driveToward(target: Vec2, pose: Pose2D, scan: LidarScan, timestamp: number): MotorCommand {
    const dx = target.x - pose.x;
    const dy = target.y - pose.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const targetAngleRel = normalizeAngle(Math.atan2(dy, dx) - pose.theta);
    const safetyMargin = this.config.robot_radius + 0.15;

    // The robot has LIDAR — it should NEVER drive into walls.
    // Check clearance in the direction we want to go BEFORE moving.

    // Emergency: something very close — find the closest wall and drive AWAY from it
    const nearestAll = this.minRangeInCone(scan, 0, Math.PI);
    if (nearestAll < safetyMargin) {
      // Find the angle of the closest wall
      let closestAngle = 0;
      let closestRange = Infinity;
      for (let i = 0; i < scan.ranges.length; i++) {
        if (scan.ranges[i] < closestRange) {
          closestRange = scan.ranges[i];
          closestAngle = scan.angles[i];
        }
      }
      // Drive directly away from the wall (opposite direction)
      const awayAngle = normalizeAngle(closestAngle + Math.PI);
      return {
        linear_velocity: 0.3,
        angular_velocity: awayAngle * 3.0,
        timestamp,
      };
    }

    // Check clearance toward target
    const targetClearance = this.minRangeInCone(scan, targetAngleRel, 10 * Math.PI / 180);
    const forwardClearance = this.minRangeInCone(scan, 0, 15 * Math.PI / 180);

    let steerAngle: number;
    if (targetClearance > 1.0) {
      // Target direction clear — go directly
      steerAngle = targetAngleRel;
    } else {
      // Target blocked — find the best open direction
      const bestDir = this.findBestOpenDirection(scan, targetAngleRel);
      steerAngle = bestDir.angle;
    }

    // Speed proportional to clearance ahead — never drive fast toward walls
    const aheadClear = this.minRangeInCone(scan, steerAngle, 15 * Math.PI / 180);
    const speedFraction = Math.min(1.0, Math.max(0.1, (aheadClear - safetyMargin) / 2.0));
    const cmd = this.steerToward(steerAngle, dist, scan, timestamp);
    cmd.linear_velocity *= speedFraction;

    // Side wall avoidance — push away from close walls
    const leftMin = this.minRangeInCone(scan, Math.PI / 2, 15 * Math.PI / 180);
    const rightMin = this.minRangeInCone(scan, -Math.PI / 2, 15 * Math.PI / 180);
    const sideMin = Math.min(leftMin, rightMin);
    if (sideMin < 0.8 && Math.abs(cmd.linear_velocity) > 0.05) {
      const urgency = 1.0 - (sideMin / 0.8);
      cmd.angular_velocity += (leftMin - rightMin) * (2.0 + urgency * 4.0);
      if (sideMin < safetyMargin) {
        cmd.linear_velocity *= 0.2;
      }
    }

    return cmd;
  }

  stop(timestamp: number): MotorCommand {
    return { linear_velocity: 0, angular_velocity: 0, timestamp };
  }

  reset(): void {}

  private steerToward(angleRel: number, dist: number, scan: LidarScan, timestamp: number): MotorCommand {
    const absAngle = Math.abs(angleRel);

    // Speed cap uses average range (responsive but not crippled by single-ray clips)
    // Safety stop (in driveToward) uses min range for genuine danger
    const fwdAvg = this.avgRangeInCone(scan, 0, 15 * Math.PI / 180);
    const tgtAvg = this.avgRangeInCone(scan, angleRel, 15 * Math.PI / 180);
    const effectiveRange = Math.max(fwdAvg, tgtAvg);

    const safetyMargin = this.config.robot_radius + 0.1;
    const maxSafeSpeed = Math.max(0, (effectiveRange - safetyMargin) * 2.5);
    const speedCap = Math.min(this.config.max_linear_speed, maxSafeSpeed);

    if (absAngle > 0.8) {
      const turnRate = Math.min(absAngle * 3.0, this.config.max_angular_speed * 0.6);
      return {
        linear_velocity: 0,
        angular_velocity: Math.sign(angleRel) * turnRate,
        timestamp,
      };
    }

    if (absAngle > 0.2) {
      const angularSpeed = angleRel * 4.0;
      return {
        linear_velocity: Math.min(speedCap * 0.4, dist),
        angular_velocity: angularSpeed,
        timestamp,
      };
    }

    const linearSpeed = Math.min(speedCap, dist * 2);
    const angularSpeed = angleRel * 3.0;
    return {
      linear_velocity: linearSpeed,
      angular_velocity: angularSpeed,
      timestamp,
    };
  }

  private findBestOpenDirection(
    scan: LidarScan,
    targetAngleRel: number,
  ): { angle: number; range: number } {
    const candidates: { angle: number; range: number; angleDiff: number }[] = [];
    const sampleCount = 36;

    for (let i = 0; i < sampleCount; i++) {
      const angle = -Math.PI + (i + 0.5) * (2 * Math.PI / sampleCount);
      const range = this.minRangeInCone(scan, angle, 8 * Math.PI / 180);

      if (range > this.config.ahead_clear_threshold) {
        const angleDiff = Math.abs(normalizeAngle(angle - targetAngleRel));
        candidates.push({ angle, range, angleDiff });
      }
    }

    if (candidates.length === 0) {
      let maxRange = 0;
      let maxAngle = 0;
      for (let i = 0; i < scan.ranges.length; i += 5) {
        if (scan.ranges[i] > maxRange) {
          maxRange = scan.ranges[i];
          maxAngle = scan.angles[i];
        }
      }
      return { angle: maxAngle, range: maxRange };
    }

    let best = candidates[0];
    for (const c of candidates) {
      if (c.angleDiff < best.angleDiff) {
        best = c;
      }
    }

    return { angle: best.angle, range: best.range };
  }

  /**
   * Minimum range within a cone — for safety-critical decisions.
   * One close wall ray dominates (conservative).
   */
  private minRangeInCone(scan: LidarScan, center: number, halfWidth: number): number {
    let min = scan.max_range;
    let found = false;
    for (let i = 0; i < scan.angles.length; i++) {
      if (Math.abs(normalizeAngle(scan.angles[i] - center)) <= halfWidth) {
        if (scan.ranges[i] < min) {
          min = scan.ranges[i];
        }
        found = true;
      }
    }
    return found ? min : scan.max_range;
  }

  /**
   * Average range within a cone — for general direction assessment.
   */
  private avgRangeInCone(scan: LidarScan, center: number, halfWidth: number): number {
    let sum = 0;
    let count = 0;
    for (let i = 0; i < scan.angles.length; i++) {
      if (Math.abs(normalizeAngle(scan.angles[i] - center)) <= halfWidth) {
        sum += scan.ranges[i];
        count++;
      }
    }
    if (count === 0) return scan.max_range;
    return sum / count;
  }
}
