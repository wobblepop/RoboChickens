// ===========================================================================
// LiDAR Simulator
// Raycasts from the robot's actual position in the ground-truth world to
// produce a simulated LiDAR scan with configurable noise.
// The robot never sees this module — it only receives the output scan.
// ===========================================================================

import type { LidarScan, LidarConfig, Pose2D } from "../shared/types";
import { gaussianRandom } from "../shared/math";
import { GroundTruthWorld } from "./ground-truth-map";

export class LidarSimulator {
  private readonly config: LidarConfig;
  private readonly angles: number[];

  constructor(config: LidarConfig) {
    this.config = config;

    // Pre-compute ray angles (evenly distributed across angular range)
    this.angles = new Array(config.ray_count);
    const startAngle = -config.angular_range / 2;
    const step = config.angular_range / config.ray_count;
    for (let i = 0; i < config.ray_count; i++) {
      this.angles[i] = startAngle + (i + 0.5) * step;
    }
  }

  /**
   * Generate a LiDAR scan from the robot's actual position in the world.
   * This is the "real sensor" — it sees the true world and adds noise.
   */
  scan(world: GroundTruthWorld, actualPose: Pose2D, timestamp: number): LidarScan {
    const ranges = new Array<number>(this.config.ray_count);

    for (let i = 0; i < this.config.ray_count; i++) {
      // Absolute angle in world frame
      const worldAngle = actualPose.theta + this.angles[i];

      // Cast ray in the ground-truth world
      let range = world.castRay(
        actualPose.x, actualPose.y,
        worldAngle,
        this.config.max_range,
      );

      // Apply noise
      if (Math.random() < this.config.spurious_rate) {
        // Spurious reading: random distance between 0 and max_range
        range = Math.random() * this.config.max_range;
      } else if (range < this.config.max_range) {
        // Valid hit: add Gaussian noise to the range measurement
        range += gaussianRandom(0, this.config.range_noise_stddev);
        // Clamp to valid range
        if (range < 0.05) range = 0.05;
        if (range > this.config.max_range) range = this.config.max_range;
      }
      // If range == max_range (no hit), leave as-is (no noise on "no return")

      ranges[i] = range;
    }

    return {
      angles: this.angles.slice(), // copy to prevent mutation
      ranges,
      max_range: this.config.max_range,
      timestamp,
    };
  }
}
