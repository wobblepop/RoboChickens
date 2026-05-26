// ===========================================================================
// SLAM Particle Filter (GMapping-style)
// Maintains a set of particles, each with a pose and weight.
// Uses a single shared occupancy grid updated by the best particle's pose.
// Scan matching refines the best particle before grid update.
// ===========================================================================

import type { LidarScan, OdometryReading, OccupancyGrid, Pose2D, SlamParticle } from "../../shared/types";
import { gaussianRandom, normalizeAngle } from "../../shared/math";
import { scanMatchScore, correlativeScanMatch, DEFAULT_SCAN_MATCH_CONFIG } from "./scan-matching";
import { updateGridFromScan } from "./inverse-sensor-model";
import { createGridCentered } from "./occupancy-grid";

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

export interface ParticleFilterConfig {
  /** Number of particles */
  particle_count: number;
  /** Odometry noise: linear fraction (how much linear distance adds noise) */
  odom_linear_noise: number;
  /** Odometry noise: angular fraction */
  odom_angular_noise: number;
  /** Minimum noise floor for linear (meters) */
  odom_linear_floor: number;
  /** Minimum noise floor for angular (radians) */
  odom_angular_floor: number;
  /** Effective sample size ratio for resampling trigger (0-1) */
  resample_threshold: number;
  /** Whether to use scan matching for pose refinement */
  use_scan_matching: boolean;
  /** Grid resolution in meters */
  grid_resolution: number;
  /** Grid size in meters (square) */
  grid_size: number;
}

export const DEFAULT_PARTICLE_FILTER_CONFIG: ParticleFilterConfig = {
  particle_count: 30,
  odom_linear_noise: 0.002,
  odom_angular_noise: 0.005,
  odom_linear_floor: 0.0001,
  odom_angular_floor: 0.0001,
  resample_threshold: 0.5,
  use_scan_matching: true,
  grid_resolution: 0.1,
  grid_size: 80.0,
};

// ---------------------------------------------------------------------------
// SLAM Particle Filter
// ---------------------------------------------------------------------------

export interface SlamResult {
  /** Best estimate of robot pose */
  pose: Pose2D;
  /** The occupancy grid (updated with latest scan) */
  grid: OccupancyGrid;
  /** Confidence based on particle spread (0-1) */
  confidence: number;
  /** Whether resampling occurred this step */
  resampled: boolean;
}

export class SlamParticleFilter {
  private particles: SlamParticle[];
  private grid: OccupancyGrid;
  private bestIdx: number = 0;
  private readonly config: ParticleFilterConfig;
  private initialized: boolean = false;
  private stepCount: number = 0;

  constructor(config: ParticleFilterConfig = DEFAULT_PARTICLE_FILTER_CONFIG) {
    this.config = config;
    this.particles = [];
    this.grid = createGridCentered(config.grid_resolution, config.grid_size, 0, 0);
  }

  /**
   * Initialize the particle filter at a known starting pose.
   * All particles start at this pose (no uncertainty at start).
   */
  initialize(startPose: Pose2D): void {
    this.particles = [];
    for (let i = 0; i < this.config.particle_count; i++) {
      this.particles.push({
        pose: { ...startPose },
        weight: 1.0 / this.config.particle_count,
      });
    }
    this.bestIdx = 0;
    this.grid = createGridCentered(
      this.config.grid_resolution,
      this.config.grid_size,
      startPose.x,
      startPose.y,
    );
    this.initialized = true;
    this.stepCount = 0;
  }

  /** Whether the filter has been initialized */
  get isInitialized(): boolean {
    return this.initialized;
  }

  /** Get the current best pose estimate */
  get bestPose(): Pose2D {
    if (this.particles.length === 0) return { x: 0, y: 0, theta: 0 };
    return this.particles[this.bestIdx].pose;
  }

  /** Get the current occupancy grid */
  get occupancyGrid(): OccupancyGrid {
    return this.grid;
  }

  /**
   * Process one step of the SLAM algorithm.
   * 1. Apply odometry (motion model) to all particles
   * 2. Scan matching on best particle (refine pose)
   * 3. Weight particles by scan-to-map agreement
   * 4. Resample if effective count too low
   * 5. Update grid using best particle's pose
   */
  processStep(odometry: OdometryReading, scan: LidarScan): SlamResult {
    if (!this.initialized) {
      this.initialize({ x: 0, y: 0, theta: 0 });
    }

    this.stepCount++;
    let resampled = false;

    // Step 1: Motion model — apply odometry with noise to all particles
    const linearDist = Math.sqrt(odometry.delta_x ** 2 + odometry.delta_y ** 2);
    const angularDist = Math.abs(odometry.delta_theta);

    for (let i = 0; i < this.particles.length; i++) {
      const p = this.particles[i];
      const linearNoise = Math.max(
        this.config.odom_linear_floor,
        linearDist * this.config.odom_linear_noise,
      );
      const angularNoise = Math.max(
        this.config.odom_angular_floor,
        angularDist * this.config.odom_angular_noise,
      );

      // Add noise to odometry
      const noisyDx = odometry.delta_x + gaussianRandom(0, linearNoise);
      const noisyDy = odometry.delta_y + gaussianRandom(0, linearNoise * 0.5);
      const noisyDtheta = odometry.delta_theta + gaussianRandom(0, angularNoise);

      // Apply in robot's local frame (rotate by particle's heading)
      const c = Math.cos(p.pose.theta);
      const s = Math.sin(p.pose.theta);
      p.pose.x += noisyDx * c - noisyDy * s;
      p.pose.y += noisyDx * s + noisyDy * c;
      p.pose.theta = normalizeAngle(p.pose.theta + noisyDtheta);
    }

    // Scan matching is DISABLED during normal travel.
    // It runs only at node arrivals (triggered externally via refinePoseAtNode)
    // to prevent tunnel-axis drift in straight corridors.
    //
    // Particle weighting by scan-to-map agreement still runs — but only to
    // distinguish particles, not to snap the best particle to a new position.
    if (this.stepCount > 10) {
      for (let i = 0; i < this.particles.length; i++) {
        const score = scanMatchScore(this.grid, scan, this.particles[i].pose);
        this.particles[i].weight *= Math.exp(score * 0.3);
      }
      this.normalizeWeights();
    }

    // Find best particle
    this.bestIdx = 0;
    for (let i = 1; i < this.particles.length; i++) {
      if (this.particles[i].weight > this.particles[this.bestIdx].weight) {
        this.bestIdx = i;
      }
    }

    // Step 4: Resample if effective count too low
    const nEff = this.effectiveSampleSize();
    if (nEff < this.config.particle_count * this.config.resample_threshold) {
      this.resample();
      resampled = true;
    }

    // Step 5: Update grid using best particle's pose
    updateGridFromScan(this.grid, scan, this.particles[this.bestIdx].pose);

    // Compute confidence from particle spread
    const confidence = this.computeConfidence();

    return {
      pose: { ...this.particles[this.bestIdx].pose },
      grid: this.grid,
      confidence,
      resampled,
    };
  }

  /**
   * Run scan matching ONCE at a node arrival. This is the only time
   * correlative scan matching runs — never during straight-line travel.
   *
   * Minimizes: α * topology_mismatch + β * displacement²
   *
   * The topology_mismatch is the negative scan-match score (how well the
   * current scan fits the grid). The displacement is the squared distance
   * from the odometry-predicted pose. Both terms are normalized to [0, 1]
   * so neither dominates:
   *   - topology score normalized by dividing by max possible score (~5.0)
   *   - displacement normalized by dividing by search window² (~0.25)
   *
   * Returns the correction applied (dx, dy) and the match quality.
   */
  refinePoseAtNode(scan: LidarScan): { dx: number; dy: number; score: number } {
    if (this.particles.length === 0) return { dx: 0, dy: 0, score: 0 };

    const best = this.particles[this.bestIdx];
    const priorPose = { ...best.pose };

    const matchResult = correlativeScanMatch(this.grid, scan, priorPose, {
      ...DEFAULT_SCAN_MATCH_CONFIG,
      // Tighter window at nodes — we trust odometry for coarse position
      coarse_window_xy: 0.3,
      coarse_window_theta: 5 * Math.PI / 180,
      // Strong odometry prior to prevent wild jumps
      odometry_prior_weight: 8.0,
    });

    const dx = matchResult.pose.x - priorPose.x;
    const dy = matchResult.pose.y - priorPose.y;
    const displacement = Math.sqrt(dx * dx + dy * dy);

    // Only apply if the match is better than the prior AND displacement is small
    if (matchResult.score > 0 && displacement < 0.5) {
      best.pose = matchResult.pose;
      return { dx, dy, score: matchResult.score };
    }

    return { dx: 0, dy: 0, score: matchResult.score };
  }

  // -------------------------------------------------------------------------
  // Internal Methods
  // -------------------------------------------------------------------------

  private normalizeWeights(): void {
    let sum = 0;
    for (const p of this.particles) sum += p.weight;
    if (sum < 1e-30) {
      // All weights collapsed — reset to uniform
      const uniform = 1.0 / this.particles.length;
      for (const p of this.particles) p.weight = uniform;
      return;
    }
    for (const p of this.particles) p.weight /= sum;
  }

  private effectiveSampleSize(): number {
    let sumSq = 0;
    for (const p of this.particles) sumSq += p.weight * p.weight;
    if (sumSq < 1e-30) return this.particles.length;
    return 1.0 / sumSq;
  }

  /**
   * Low-variance (systematic) resampling.
   * More statistically efficient than multinomial resampling.
   */
  private resample(): void {
    const n = this.particles.length;
    const newParticles: SlamParticle[] = [];

    // Build cumulative weight array
    const cumulative = new Array<number>(n);
    cumulative[0] = this.particles[0].weight;
    for (let i = 1; i < n; i++) {
      cumulative[i] = cumulative[i - 1] + this.particles[i].weight;
    }

    // Systematic resampling with single random offset
    const step = 1.0 / n;
    let r = Math.random() * step;
    let j = 0;

    for (let i = 0; i < n; i++) {
      while (j < n - 1 && r > cumulative[j]) j++;
      newParticles.push({
        pose: { ...this.particles[j].pose },
        weight: step,
      });
      // Jitter only when noise is configured (prevents particle depletion)
      if (this.config.odom_linear_noise > 0) {
        newParticles[i].pose.x += gaussianRandom(0, 0.002);
        newParticles[i].pose.y += gaussianRandom(0, 0.002);
      }
      if (this.config.odom_angular_noise > 0) {
        newParticles[i].pose.theta = normalizeAngle(
          newParticles[i].pose.theta + gaussianRandom(0, 0.001),
        );
      }
      r += step;
    }

    this.particles = newParticles;
    this.bestIdx = 0;
  }

  /**
   * Confidence based on particle spread.
   * Tight cluster = high confidence, dispersed = low.
   */
  private computeConfidence(): number {
    // Weighted mean position
    let mx = 0, my = 0;
    for (const p of this.particles) {
      mx += p.pose.x * p.weight;
      my += p.pose.y * p.weight;
    }

    // Weighted variance
    let variance = 0;
    for (const p of this.particles) {
      const dx = p.pose.x - mx;
      const dy = p.pose.y - my;
      variance += (dx * dx + dy * dy) * p.weight;
    }

    const spread = Math.sqrt(variance);
    // Map spread to confidence: 0 spread = 1.0, large spread = 0.0
    return Math.exp(-spread * 2.0);
  }
}
