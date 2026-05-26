// ===========================================================================
// Occupancy Grid
// Dense Float32Array-based log-odds occupancy grid.
// Provides creation, cell access, coordinate conversion, and region extraction.
// ===========================================================================

import type { OccupancyGrid, OccupancyGridConfig, Vec2 } from "../../shared/types";
import { LOG_ODDS_MAX, LOG_ODDS_MIN } from "../../shared/types";
import { clamp } from "../../shared/math";

// ---------------------------------------------------------------------------
// Grid Creation
// ---------------------------------------------------------------------------

/**
 * Create an occupancy grid initialized to zero (unknown) everywhere.
 * Origin is the world position of cell [0,0]'s center.
 */
export function createGrid(
  resolution: number,
  widthMeters: number,
  heightMeters: number,
  originX: number,
  originY: number,
): OccupancyGrid {
  const width = Math.ceil(widthMeters / resolution);
  const height = Math.ceil(heightMeters / resolution);
  const config: OccupancyGridConfig = {
    resolution,
    width,
    height,
    origin_x: originX,
    origin_y: originY,
  };
  return {
    config,
    cells: new Float32Array(width * height),
  };
}

/**
 * Create a grid centered on a given world position.
 */
export function createGridCentered(
  resolution: number,
  sizeMeters: number,
  centerX: number,
  centerY: number,
): OccupancyGrid {
  const originX = centerX - sizeMeters / 2;
  const originY = centerY - sizeMeters / 2;
  return createGrid(resolution, sizeMeters, sizeMeters, originX, originY);
}

// ---------------------------------------------------------------------------
// Coordinate Conversion
// ---------------------------------------------------------------------------

/** Convert world coordinates to grid cell indices. Returns floored integers. */
export function worldToGrid(worldX: number, worldY: number, config: OccupancyGridConfig): { gx: number; gy: number } {
  return {
    gx: Math.floor((worldX - config.origin_x) / config.resolution),
    gy: Math.floor((worldY - config.origin_y) / config.resolution),
  };
}

/** Convert grid cell indices to world coordinates (cell center). */
export function gridToWorld(gx: number, gy: number, config: OccupancyGridConfig): Vec2 {
  return {
    x: config.origin_x + (gx + 0.5) * config.resolution,
    y: config.origin_y + (gy + 0.5) * config.resolution,
  };
}

/** Check if grid coordinates are within bounds. */
export function gridInBounds(gx: number, gy: number, config: OccupancyGridConfig): boolean {
  return gx >= 0 && gx < config.width && gy >= 0 && gy < config.height;
}

// ---------------------------------------------------------------------------
// Cell Access
// ---------------------------------------------------------------------------

/** Get the log-odds value at a grid cell. Returns 0 (unknown) if out of bounds. */
export function getCell(grid: OccupancyGrid, gx: number, gy: number): number {
  if (!gridInBounds(gx, gy, grid.config)) return 0;
  return grid.cells[gy * grid.config.width + gx];
}

/** Get the log-odds value at a world position. */
export function getCellAtWorld(grid: OccupancyGrid, worldX: number, worldY: number): number {
  const { gx, gy } = worldToGrid(worldX, worldY, grid.config);
  return getCell(grid, gx, gy);
}

/** Set the log-odds value at a grid cell. Clamps to [LOG_ODDS_MIN, LOG_ODDS_MAX]. */
export function setCell(grid: OccupancyGrid, gx: number, gy: number, value: number): void {
  if (!gridInBounds(gx, gy, grid.config)) return;
  grid.cells[gy * grid.config.width + gx] = clamp(value, LOG_ODDS_MIN, LOG_ODDS_MAX);
}

/** Add to the log-odds value at a grid cell. Clamps result. */
export function addToCell(grid: OccupancyGrid, gx: number, gy: number, delta: number): void {
  if (!gridInBounds(gx, gy, grid.config)) return;
  const idx = gy * grid.config.width + gx;
  grid.cells[idx] = clamp(grid.cells[idx] + delta, LOG_ODDS_MIN, LOG_ODDS_MAX);
}

// ---------------------------------------------------------------------------
// Probability Conversion
// ---------------------------------------------------------------------------

/** Convert log-odds to probability [0, 1]. */
export function logOddsToProbability(logOdds: number): number {
  return 1.0 - 1.0 / (1.0 + Math.exp(logOdds));
}

/** Convert probability [0, 1] to log-odds. */
export function probabilityToLogOdds(p: number): number {
  if (p <= 0) return LOG_ODDS_MIN;
  if (p >= 1) return LOG_ODDS_MAX;
  return Math.log(p / (1 - p));
}

// ---------------------------------------------------------------------------
// Occupancy Queries
// ---------------------------------------------------------------------------

/** Check if a cell is occupied (log-odds above threshold). */
export function isCellOccupied(grid: OccupancyGrid, gx: number, gy: number, threshold: number = 0.5): boolean {
  return getCell(grid, gx, gy) > threshold;
}

/** Check if a cell is free (log-odds below negative threshold). */
export function isCellFree(grid: OccupancyGrid, gx: number, gy: number, threshold: number = -0.5): boolean {
  return getCell(grid, gx, gy) < threshold;
}

/** Check if a cell is unknown (close to zero). */
export function isCellUnknown(grid: OccupancyGrid, gx: number, gy: number, tolerance: number = 0.3): boolean {
  const val = getCell(grid, gx, gy);
  return val > -tolerance && val < tolerance;
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/** Count cells by category. */
export function gridStats(grid: OccupancyGrid): { occupied: number; free: number; unknown: number; total: number } {
  let occupied = 0;
  let free = 0;
  let unknown = 0;
  const total = grid.config.width * grid.config.height;
  for (let i = 0; i < total; i++) {
    const v = grid.cells[i];
    if (v > 0.5) occupied++;
    else if (v < -0.5) free++;
    else unknown++;
  }
  return { occupied, free, unknown, total };
}

// ---------------------------------------------------------------------------
// Region Extraction
// ---------------------------------------------------------------------------

/**
 * Extract a sub-region of the grid around a world position.
 * Returns a new smaller grid covering the specified radius.
 */
export function extractRegion(
  grid: OccupancyGrid,
  centerX: number,
  centerY: number,
  radiusMeters: number,
): OccupancyGrid {
  const { config } = grid;
  const cellRadius = Math.ceil(radiusMeters / config.resolution);
  const { gx: cx, gy: cy } = worldToGrid(centerX, centerY, config);

  const regionWidth = cellRadius * 2 + 1;
  const regionHeight = cellRadius * 2 + 1;
  const regionOriginX = config.origin_x + (cx - cellRadius) * config.resolution;
  const regionOriginY = config.origin_y + (cy - cellRadius) * config.resolution;

  const regionConfig: OccupancyGridConfig = {
    resolution: config.resolution,
    width: regionWidth,
    height: regionHeight,
    origin_x: regionOriginX,
    origin_y: regionOriginY,
  };

  const regionCells = new Float32Array(regionWidth * regionHeight);

  for (let ry = 0; ry < regionHeight; ry++) {
    for (let rx = 0; rx < regionWidth; rx++) {
      const srcX = cx - cellRadius + rx;
      const srcY = cy - cellRadius + ry;
      if (gridInBounds(srcX, srcY, config)) {
        regionCells[ry * regionWidth + rx] = grid.cells[srcY * config.width + srcX];
      }
      // Out-of-bounds cells stay at 0 (unknown)
    }
  }

  return { config: regionConfig, cells: regionCells };
}

/**
 * Convert grid cells to the visualization format (array of {x, y, state, confidence}).
 * Only includes cells that have been observed (not unknown).
 */
export function gridToVizCells(
  grid: OccupancyGrid,
): { x: number; y: number; state: string; confidence: number }[] {
  const result: { x: number; y: number; state: string; confidence: number }[] = [];
  const { config } = grid;

  for (let gy = 0; gy < config.height; gy++) {
    for (let gx = 0; gx < config.width; gx++) {
      const logOdds = grid.cells[gy * config.width + gx];
      if (Math.abs(logOdds) < 0.3) continue; // skip unknown cells

      const world = gridToWorld(gx, gy, config);
      let state: string;
      if (logOdds > 0.5) state = "wall";
      else state = "floor";

      const confidence = Math.min(1.0, Math.abs(logOdds) / LOG_ODDS_MAX);
      result.push({ x: world.x, y: world.y, state, confidence });
    }
  }

  return result;
}
