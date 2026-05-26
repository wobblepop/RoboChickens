// ===========================================================================
// Inverse Sensor Model
// Given a LiDAR scan and a pose, updates the occupancy grid by raycasting
// each beam: cells the beam passes through become free, the endpoint (if hit)
// becomes occupied.
// ===========================================================================

import type { LidarScan, OccupancyGrid, Pose2D } from "../../shared/types";
import { LOG_ODDS_OCCUPIED, LOG_ODDS_FREE, LOG_ODDS_MAX, LOG_ODDS_MIN } from "../../shared/types";
import { clamp } from "../../shared/math";
import { worldToGrid, gridInBounds } from "./occupancy-grid";

// ---------------------------------------------------------------------------
// Bresenham Ray (inlined for performance — this runs thousands of times/tick)
// ---------------------------------------------------------------------------

/**
 * Update the occupancy grid from a single LiDAR scan at the given pose.
 * For each ray:
 *   - Cells the ray passes through: log-odds += L_FREE
 *   - Cell at the endpoint (if hit): log-odds += L_OCC
 */
export function updateGridFromScan(
  grid: OccupancyGrid,
  scan: LidarScan,
  pose: Pose2D,
): void {
  const { config } = grid;
  const { cells } = grid;
  const { width } = config;

  for (let i = 0; i < scan.ranges.length; i++) {
    const range = scan.ranges[i];
    const rayAngle = pose.theta + scan.angles[i];
    const hitWall = range < scan.max_range;

    // Compute start cell
    const sx = Math.floor((pose.x - config.origin_x) / config.resolution);
    const sy = Math.floor((pose.y - config.origin_y) / config.resolution);

    // Compute end cell
    const endWorldX = pose.x + range * Math.cos(rayAngle);
    const endWorldY = pose.y + range * Math.sin(rayAngle);
    const ex = Math.floor((endWorldX - config.origin_x) / config.resolution);
    const ey = Math.floor((endWorldY - config.origin_y) / config.resolution);

    // Bresenham from (sx,sy) to (ex,ey)
    let cx = sx;
    let cy = sy;
    let dx = Math.abs(ex - cx);
    let dy = Math.abs(ey - cy);
    const stepX = cx < ex ? 1 : -1;
    const stepY = cy < ey ? 1 : -1;
    let err = dx - dy;

    const maxSteps = dx + dy + 1;
    let step = 0;

    while (step < maxSteps) {
      const isEndpoint = (cx === ex && cy === ey);

      if (cx >= 0 && cx < config.width && cy >= 0 && cy < config.height) {
        const idx = cy * width + cx;
        if (isEndpoint && hitWall) {
          // Endpoint hit: mark as occupied, plus neighbors to fill gaps
          cells[idx] = clampLogOdds(cells[idx] + LOG_ODDS_OCCUPIED);
          // Thicken: also mark 4-connected neighbors as lightly occupied
          const halfOcc = LOG_ODDS_OCCUPIED * 0.5;
          for (const [nx, ny] of [[cx-1,cy],[cx+1,cy],[cx,cy-1],[cx,cy+1]]) {
            if (nx >= 0 && nx < config.width && ny >= 0 && ny < config.height) {
              const ni = ny * width + nx;
              if (cells[ni] < 0.3) { // only if not already free
                cells[ni] = clampLogOdds(cells[ni] + halfOcc);
              }
            }
          }
        } else if (!isEndpoint) {
          // Ray passes through: mark as free
          cells[idx] = clampLogOdds(cells[idx] + LOG_ODDS_FREE);
        }
      }

      if (isEndpoint) break;

      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        cx += stepX;
      }
      if (e2 < dx) {
        err += dx;
        cy += stepY;
      }
      step++;
    }
  }
}

function clampLogOdds(value: number): number {
  if (value > LOG_ODDS_MAX) return LOG_ODDS_MAX;
  if (value < LOG_ODDS_MIN) return LOG_ODDS_MIN;
  return value;
}

/**
 * Count how many new cells were observed (transitioned from unknown to known)
 * after a grid update. Useful for tracking exploration progress.
 *
 * Call this by comparing the grid before and after updateGridFromScan.
 * Alternatively, track cell state changes within the update itself.
 * For simplicity, this version checks a snapshot.
 */
export function countNewlyObservedCells(
  gridBefore: Float32Array,
  gridAfter: Float32Array,
  unknownThreshold: number = 0.3,
): number {
  let count = 0;
  for (let i = 0; i < gridBefore.length; i++) {
    const wasBefore = Math.abs(gridBefore[i]) > unknownThreshold;
    const isNow = Math.abs(gridAfter[i]) > unknownThreshold;
    if (!wasBefore && isNow) count++;
  }
  return count;
}

/**
 * Get the set of grid cells that are newly observed (for visualization).
 * Returns "x,y" formatted strings in world grid coordinates.
 */
export function getNewlyExploredCells(
  grid: OccupancyGrid,
  previousSnapshot: Float32Array | null,
  unknownThreshold: number = 0.3,
): string[] {
  const result: string[] = [];
  const { config } = grid;

  for (let gy = 0; gy < config.height; gy++) {
    for (let gx = 0; gx < config.width; gx++) {
      const idx = gy * config.width + gx;
      const current = Math.abs(grid.cells[idx]) > unknownThreshold;
      const previous = previousSnapshot ? Math.abs(previousSnapshot[idx]) > unknownThreshold : false;
      if (current && !previous) {
        // Convert grid cell to world grid coordinates (1 cell = 1 meter in viz)
        const worldX = Math.floor(config.origin_x + gx * config.resolution);
        const worldY = Math.floor(config.origin_y + gy * config.resolution);
        result.push(`${worldX},${worldY}`);
      }
    }
  }

  return result;
}
