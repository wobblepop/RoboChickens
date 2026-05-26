// ===========================================================================
// Scan Matching
// Multi-resolution correlative scan matcher.
// Scores a LiDAR scan at candidate poses against the occupancy grid,
// finding the pose that best aligns the scan with the existing map.
// ===========================================================================

import type { LidarScan, OccupancyGrid, Pose2D } from "../../shared/types";
import { gridInBounds } from "./occupancy-grid";
import { normalizeAngle } from "../../shared/math";

// ---------------------------------------------------------------------------
// Scan Match Score
// ---------------------------------------------------------------------------

/**
 * Score how well a LiDAR scan matches the occupancy grid at a candidate pose.
 * Higher score = better match.
 *
 * For each ray that hit something (range < max_range):
 *   - Compute where the hit point would be if the robot were at candidatePose
 *   - Look up the grid cell at that point
 *   - Add the log-odds value to the score (positive = occupied = good match)
 *
 * Returns the average score per valid ray. Range: approximately [-5, 5].
 */
export function scanMatchScore(
  grid: OccupancyGrid,
  scan: LidarScan,
  candidatePose: Pose2D,
): number {
  const { config, cells } = grid;
  let score = 0;
  let validRays = 0;

  for (let i = 0; i < scan.ranges.length; i++) {
    const range = scan.ranges[i];
    if (range >= scan.max_range) continue; // no-return rays don't contribute

    const angle = candidatePose.theta + scan.angles[i];
    const hitX = candidatePose.x + range * Math.cos(angle);
    const hitY = candidatePose.y + range * Math.sin(angle);

    const gx = Math.floor((hitX - config.origin_x) / config.resolution);
    const gy = Math.floor((hitY - config.origin_y) / config.resolution);

    if (!gridInBounds(gx, gy, config)) continue;

    score += cells[gy * config.width + gx];
    validRays++;
  }

  if (validRays === 0) return 0;
  return score / validRays;
}

// ---------------------------------------------------------------------------
// Multi-Resolution Correlative Scan Matching
// ---------------------------------------------------------------------------

export interface ScanMatchConfig {
  /** Coarse search: translation window in meters (±) */
  coarse_window_xy: number;
  /** Coarse search: rotation window in radians (±) */
  coarse_window_theta: number;
  /** Coarse search: translation step multiplier (× resolution) */
  coarse_step_multiplier: number;
  /** Coarse search: angular step in radians */
  coarse_angular_step: number;
  /** Fine search: translation step multiplier (× resolution) */
  fine_step_multiplier: number;
  /** Fine search: angular step in radians */
  fine_angular_step: number;
  /** Odometry prior weight: penalizes candidates far from the prior pose.
   *  Higher = trust odometry more, prevents tunnel degeneration. */
  odometry_prior_weight: number;
}

export const DEFAULT_SCAN_MATCH_CONFIG: ScanMatchConfig = {
  coarse_window_xy: 0.5,
  coarse_window_theta: 10 * Math.PI / 180,
  coarse_step_multiplier: 4,
  coarse_angular_step: 2 * Math.PI / 180,
  fine_step_multiplier: 1,
  fine_angular_step: 0.5 * Math.PI / 180,
  odometry_prior_weight: 20.0,
};

/**
 * Multi-resolution correlative scan matching.
 *
 * Phase 1 (coarse): Search a wide window with large steps.
 * Phase 2 (fine): Search a narrow window around the best coarse result.
 *
 * Returns the pose that maximizes the scan-to-map match score.
 */
export function correlativeScanMatch(
  grid: OccupancyGrid,
  scan: LidarScan,
  priorPose: Pose2D,
  config: ScanMatchConfig = DEFAULT_SCAN_MATCH_CONFIG,
): { pose: Pose2D; score: number } {
  const resolution = grid.config.resolution;
  const lambda = config.odometry_prior_weight;

  // Normalize both terms to comparable scales:
  //   - Scan score range: roughly [-5, +5] (log-odds averaged over rays)
  //   - Displacement: (dx² + dy²) for 0.5m window = 0.25m²
  //   - dtheta: ~0.17 rad (10°) squared = 0.03
  //
  // Scale displacement to match scan score magnitude:
  //   max_displacement² = window² = coarse_window_xy² ≈ 0.25
  //   max_scan_score ≈ 5.0
  //   normalization = max_scan_score / max_displacement² = 20
  //
  // Combined objective: minimize (topology_mismatch² + displacement²)
  // where both are on [0, 1] scale.
  const windowSq = Math.max(0.01, config.coarse_window_xy * config.coarse_window_xy);
  const displacementScale = 5.0 / windowSq;

  function priorPenalty(candidate: Pose2D): number {
    const dx = candidate.x - priorPose.x;
    const dy = candidate.y - priorPose.y;
    const dtheta = normalizeAngle(candidate.theta - priorPose.theta);
    return lambda * displacementScale * (dx * dx + dy * dy) + lambda * 10.0 * dtheta * dtheta;
  }

  // Phase 1: Coarse search
  const coarseStep = resolution * config.coarse_step_multiplier;
  let bestPose = priorPose;
  let bestScore = scanMatchScore(grid, scan, priorPose);

  const xSteps = Math.ceil(config.coarse_window_xy / coarseStep);
  const ySteps = Math.ceil(config.coarse_window_xy / coarseStep);
  const thetaSteps = Math.ceil(config.coarse_window_theta / config.coarse_angular_step);

  for (let xi = -xSteps; xi <= xSteps; xi++) {
    const cx = priorPose.x + xi * coarseStep;
    for (let yi = -ySteps; yi <= ySteps; yi++) {
      const cy = priorPose.y + yi * coarseStep;
      for (let ti = -thetaSteps; ti <= thetaSteps; ti++) {
        const ctheta = priorPose.theta + ti * config.coarse_angular_step;
        const candidate: Pose2D = { x: cx, y: cy, theta: ctheta };
        const score = scanMatchScore(grid, scan, candidate) - priorPenalty(candidate);
        if (score > bestScore) {
          bestScore = score;
          bestPose = candidate;
        }
      }
    }
  }

  // Phase 2: Fine search around best coarse result
  const fineStep = resolution * config.fine_step_multiplier;
  const fineWindowXY = coarseStep * 2;
  const fineWindowTheta = config.coarse_angular_step * 2;

  const fineXSteps = Math.ceil(fineWindowXY / fineStep);
  const fineYSteps = Math.ceil(fineWindowXY / fineStep);
  const fineThetaSteps = Math.ceil(fineWindowTheta / config.fine_angular_step);

  const coarseBest = { ...bestPose };

  for (let xi = -fineXSteps; xi <= fineXSteps; xi++) {
    const cx = coarseBest.x + xi * fineStep;
    for (let yi = -fineYSteps; yi <= fineYSteps; yi++) {
      const cy = coarseBest.y + yi * fineStep;
      for (let ti = -fineThetaSteps; ti <= fineThetaSteps; ti++) {
        const ctheta = coarseBest.theta + ti * config.fine_angular_step;
        const candidate: Pose2D = { x: cx, y: cy, theta: ctheta };
        const score = scanMatchScore(grid, scan, candidate) - priorPenalty(candidate);
        if (score > bestScore) {
          bestScore = score;
          bestPose = candidate;
        }
      }
    }
  }

  return { pose: bestPose, score: bestScore };
}

/**
 * Lightweight scan match for loop closure: compare a scan against
 * a stored grid region. Uses the same scoring but with a wider search window.
 */
export function loopClosureScanMatch(
  storedGrid: OccupancyGrid,
  currentScan: LidarScan,
  initialGuess: Pose2D,
): { pose: Pose2D; score: number } {
  const loopConfig: ScanMatchConfig = {
    coarse_window_xy: 1.5,
    coarse_window_theta: 20 * Math.PI / 180,
    coarse_step_multiplier: 3,
    coarse_angular_step: 3 * Math.PI / 180,
    fine_step_multiplier: 1,
    odometry_prior_weight: 0.5,
    fine_angular_step: 0.5 * Math.PI / 180,
  };
  return correlativeScanMatch(storedGrid, currentScan, initialGuess, loopConfig);
}
