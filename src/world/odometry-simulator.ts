// ===========================================================================
// Odometry Simulator
// Given the robot's actual movement (computed by the physics engine),
// produces an odometry reading with configurable noise.
// The robot never sees the true movement — only this noisy version.
// ===========================================================================

import type { OdometryReading, OdometryConfig } from "../shared/types";
import { gaussianRandom } from "../shared/math";

export class OdometrySimulator {
  private readonly config: OdometryConfig;

  constructor(config: OdometryConfig) {
    this.config = config;
  }

  /**
   * Given the actual movement of the robot this tick (in the robot's local frame),
   * produce a noisy odometry reading.
   *
   * @param actualDx - True x displacement in robot frame (meters)
   * @param actualDy - True y displacement in robot frame (meters)
   * @param actualDtheta - True heading change (radians)
   * @param timestamp - Current simulation time
   */
  reading(
    actualDx: number,
    actualDy: number,
    actualDtheta: number,
    timestamp: number,
  ): OdometryReading {
    const linearDist = Math.sqrt(actualDx * actualDx + actualDy * actualDy);
    const angularDist = Math.abs(actualDtheta);

    // Noise scales with distance moved
    const linearNoise = Math.max(
      this.config.linear_noise_floor,
      linearDist * this.config.linear_noise_fraction,
    );
    const angularNoise = Math.max(
      this.config.angular_noise_floor,
      angularDist * this.config.angular_noise_fraction,
    );

    // Cross-contamination: linear movement creates angular error and vice versa
    const crossLinearFromAngular = angularDist * this.config.linear_noise_fraction * 0.3;
    const crossAngularFromLinear = linearDist * this.config.angular_noise_fraction * 0.3;

    const totalLinearNoise = linearNoise + crossLinearFromAngular;
    const totalAngularNoise = angularNoise + crossAngularFromLinear;

    // Add Gaussian noise
    const noisyDx = actualDx + gaussianRandom(0, totalLinearNoise);
    const noisyDy = actualDy + gaussianRandom(0, totalLinearNoise * 0.5); // lateral noise is lower
    const noisyDtheta = actualDtheta + gaussianRandom(0, totalAngularNoise);

    return {
      delta_x: noisyDx,
      delta_y: noisyDy,
      delta_theta: noisyDtheta,
      timestamp,
    };
  }
}
