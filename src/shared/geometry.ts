// ===========================================================================
// Geometry Utilities
// Bresenham line algorithm, ray-segment intersection, line-of-sight.
// ===========================================================================

import type { Vec2 } from "./types";

// ---------------------------------------------------------------------------
// Bresenham Line Algorithm
// ---------------------------------------------------------------------------

export interface GridCell {
  x: number;
  y: number;
}

/**
 * Bresenham's line algorithm. Returns all grid cells between (x0,y0) and (x1,y1)
 * inclusive of both endpoints. Works for any octant.
 */
export function bresenhamLine(x0: number, y0: number, x1: number, y1: number): GridCell[] {
  const cells: GridCell[] = [];
  let cx = Math.floor(x0);
  let cy = Math.floor(y0);
  const ex = Math.floor(x1);
  const ey = Math.floor(y1);

  let dx = Math.abs(ex - cx);
  let dy = Math.abs(ey - cy);
  const sx = cx < ex ? 1 : -1;
  const sy = cy < ey ? 1 : -1;
  let err = dx - dy;

  const maxSteps = dx + dy + 1;
  for (let i = 0; i < maxSteps; i++) {
    cells.push({ x: cx, y: cy });
    if (cx === ex && cy === ey) break;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      cx += sx;
    }
    if (e2 < dx) {
      err += dx;
      cy += sy;
    }
  }

  return cells;
}

/**
 * Bresenham ray from start to end. Unlike bresenhamLine, this takes
 * floating-point world coordinates and a grid resolution, converting to
 * grid cells internally. Returns cells in traversal order.
 */
export function bresenhamRay(
  startX: number, startY: number,
  endX: number, endY: number,
  originX: number, originY: number,
  resolution: number,
): GridCell[] {
  const gx0 = Math.floor((startX - originX) / resolution);
  const gy0 = Math.floor((startY - originY) / resolution);
  const gx1 = Math.floor((endX - originX) / resolution);
  const gy1 = Math.floor((endY - originY) / resolution);
  return bresenhamLine(gx0, gy0, gx1, gy1);
}

// ---------------------------------------------------------------------------
// Ray-Segment Intersection
// ---------------------------------------------------------------------------

export interface RayHit {
  distance: number;
  point: Vec2;
}

/**
 * Test ray intersection with a line segment.
 * Ray starts at origin, goes in direction (cos(angle), sin(angle)).
 * Segment is from p0 to p1.
 * Returns the hit distance and point, or null if no intersection.
 */
export function raySegmentIntersection(
  rayOrigin: Vec2,
  rayAngle: number,
  p0: Vec2,
  p1: Vec2,
): RayHit | null {
  const dx = Math.cos(rayAngle);
  const dy = Math.sin(rayAngle);

  const segDx = p1.x - p0.x;
  const segDy = p1.y - p0.y;

  const denom = dx * segDy - dy * segDx;
  if (Math.abs(denom) < 1e-10) return null; // parallel

  const ox = p0.x - rayOrigin.x;
  const oy = p0.y - rayOrigin.y;

  const t = (ox * segDy - oy * segDx) / denom;  // distance along ray
  const u = (ox * dy - oy * dx) / denom;         // parameter along segment

  if (t < 0) return null;     // behind ray origin
  if (u < 0 || u > 1) return null; // outside segment

  return {
    distance: t,
    point: { x: rayOrigin.x + dx * t, y: rayOrigin.y + dy * t },
  };
}

/**
 * Cast a ray through a grid, returning the distance to the first occupied cell.
 * The isOccupied callback checks whether a grid cell is blocked.
 * Returns max_range if no hit found within range.
 */
export function gridRaycast(
  startX: number, startY: number,
  angle: number,
  maxRange: number,
  originX: number, originY: number,
  resolution: number,
  gridWidth: number,
  gridHeight: number,
  isOccupied: (gx: number, gy: number) => boolean,
): number {
  const endX = startX + maxRange * Math.cos(angle);
  const endY = startY + maxRange * Math.sin(angle);

  const cells = bresenhamRay(startX, startY, endX, endY, originX, originY, resolution);

  for (let i = 1; i < cells.length; i++) {
    const cell = cells[i];
    if (cell.x < 0 || cell.x >= gridWidth || cell.y < 0 || cell.y >= gridHeight) {
      // Out of bounds — treat as max range
      const prevCell = cells[i - 1];
      const wx = originX + (prevCell.x + 0.5) * resolution;
      const wy = originY + (prevCell.y + 0.5) * resolution;
      const dx = wx - startX;
      const dy = wy - startY;
      return Math.sqrt(dx * dx + dy * dy);
    }
    if (isOccupied(cell.x, cell.y)) {
      // Hit an occupied cell — return distance to cell center
      const wx = originX + (cell.x + 0.5) * resolution;
      const wy = originY + (cell.y + 0.5) * resolution;
      const dx = wx - startX;
      const dy = wy - startY;
      return Math.sqrt(dx * dx + dy * dy);
    }
  }

  return maxRange;
}

// ---------------------------------------------------------------------------
// Line-of-Sight (grid-based)
// ---------------------------------------------------------------------------

/**
 * Check if there is line-of-sight between two points in a grid.
 * Returns true if no occupied cells are crossed (exclusive of endpoints).
 */
export function hasLineOfSight(
  x0: number, y0: number,
  x1: number, y1: number,
  originX: number, originY: number,
  resolution: number,
  gridWidth: number,
  gridHeight: number,
  isOccupied: (gx: number, gy: number) => boolean,
): boolean {
  const cells = bresenhamRay(x0, y0, x1, y1, originX, originY, resolution);
  // Skip first and last cell (the endpoints)
  for (let i = 1; i < cells.length - 1; i++) {
    const cell = cells[i];
    if (cell.x < 0 || cell.x >= gridWidth || cell.y < 0 || cell.y >= gridHeight) {
      return false;
    }
    if (isOccupied(cell.x, cell.y)) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Distance from point to line segment
// ---------------------------------------------------------------------------

export function pointToSegmentDistanceSq(point: Vec2, segA: Vec2, segB: Vec2): number {
  const dx = segB.x - segA.x;
  const dy = segB.y - segA.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq < 1e-12) {
    // Degenerate segment (point)
    const px = point.x - segA.x;
    const py = point.y - segA.y;
    return px * px + py * py;
  }
  let t = ((point.x - segA.x) * dx + (point.y - segA.y) * dy) / lenSq;
  if (t < 0) t = 0;
  else if (t > 1) t = 1;
  const projX = segA.x + t * dx;
  const projY = segA.y + t * dy;
  const px = point.x - projX;
  const py = point.y - projY;
  return px * px + py * py;
}
