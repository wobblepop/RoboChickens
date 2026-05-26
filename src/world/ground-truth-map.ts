// ===========================================================================
// Ground Truth Map
// Loads and stores the warehouse map. Provides cell access and traversability
// checks. This is the "real world" that only the server knows about.
// ===========================================================================

import * as fs from "fs";
import * as path from "path";
import { CellType, GroundTruthMap, RobotStartPosition } from "../shared/types";

// ---------------------------------------------------------------------------
// Map Loading
// ---------------------------------------------------------------------------

/**
 * Parse a warehouse map file into a GroundTruthMap.
 * File format:
 *   # Name: Map Name
 *   # Difficulty: N
 *   # Description: ...
 *   Then lines of cell characters: #.DOPR~
 *   R or digits 1-9 mark robot start positions.
 */
export function loadMap(filePath: string): GroundTruthMap {
  const content = fs.readFileSync(filePath, "utf-8");
  const lines = content.split(/\r?\n/);

  let name = path.basename(filePath, ".txt");
  let difficulty = 1;
  let description = "";
  const gridLines: string[] = [];

  for (const line of lines) {
    if (line.startsWith("# Name:")) {
      name = line.slice(7).trim();
    } else if (line.startsWith("# Difficulty:")) {
      const parts = line.slice(13).trim().split(/\s/);
      difficulty = parseInt(parts[0], 10) || 1;
    } else if (line.startsWith("# Description:")) {
      description = line.slice(14).trim();
    } else if (line.startsWith("#") && !line.startsWith("##")) {
      // Other comment or grid line starting with #
      if (line.length > 1 && !line.startsWith("# ")) {
        gridLines.push(line);
      }
    } else if (line.length > 0) {
      gridLines.push(line);
    }
  }

  if (gridLines.length === 0) {
    throw new Error(`Map file ${filePath} contains no grid data`);
  }

  const height = gridLines.length;
  const width = Math.max(...gridLines.map((l) => l.length));
  const cells: CellType[][] = [];
  const robotStarts: RobotStartPosition[] = [];

  for (let y = 0; y < height; y++) {
    const row: CellType[] = [];
    const line = gridLines[y];
    for (let x = 0; x < width; x++) {
      const ch = x < line.length ? line[x] : "#";
      let cellType: CellType;
      switch (ch) {
        case "#": cellType = CellType.Wall; break;
        case ".": cellType = CellType.Floor; break;
        case "D": cellType = CellType.Doorway; break;
        case "O": cellType = CellType.Obstacle; break;
        case "P": cellType = CellType.Pillar; break;
        case "~": cellType = CellType.RoughTerrain; break;
        case "R":
          cellType = CellType.RobotStart;
          robotStarts.push({ id: `robot-${robotStarts.length}`, x, y });
          break;
        default:
          // Digits 1-9 are also robot starts
          if (ch >= "1" && ch <= "9") {
            cellType = CellType.RobotStart;
            robotStarts.push({ id: `robot-${ch}`, x, y });
          } else {
            cellType = CellType.Wall;
          }
      }
      row.push(cellType);
    }
    cells.push(row);
  }

  return { name, width, height, cells, robot_starts: robotStarts, difficulty, description };
}

// ---------------------------------------------------------------------------
// Ground Truth World
// ---------------------------------------------------------------------------

export class GroundTruthWorld {
  readonly map: GroundTruthMap;
  private traversableCount: number = -1;

  constructor(map: GroundTruthMap) {
    this.map = map;
  }

  get width(): number { return this.map.width; }
  get height(): number { return this.map.height; }

  inBounds(x: number, y: number): boolean {
    return x >= 0 && x < this.map.width && y >= 0 && y < this.map.height;
  }

  getCell(x: number, y: number): CellType {
    if (!this.inBounds(x, y)) return CellType.Wall;
    return this.map.cells[y][x];
  }

  isTraversable(x: number, y: number): boolean {
    const cell = this.getCell(x, y);
    return cell === CellType.Floor ||
           cell === CellType.Doorway ||
           cell === CellType.RoughTerrain ||
           cell === CellType.RobotStart;
  }

  isOccupied(x: number, y: number): boolean {
    return !this.isTraversable(x, y);
  }

  /**
   * Check if a world-coordinate position (in meters, where 1 cell = 1 meter)
   * is traversable. Uses the cell grid directly.
   */
  isPositionTraversable(worldX: number, worldY: number): boolean {
    const gx = Math.floor(worldX);
    const gy = Math.floor(worldY);
    return this.isTraversable(gx, gy);
  }

  /**
   * Check if a circle at world position with given radius collides with walls.
   * Tests the 4 cardinal + 4 diagonal points on the circle perimeter.
   */
  isCircleTraversable(worldX: number, worldY: number, radius: number): boolean {
    // Check center
    if (!this.isPositionTraversable(worldX, worldY)) return false;

    // Check 8 perimeter points
    for (let i = 0; i < 8; i++) {
      const angle = (i / 8) * 2 * Math.PI;
      const px = worldX + radius * Math.cos(angle);
      const py = worldY + radius * Math.sin(angle);
      if (!this.isPositionTraversable(px, py)) return false;
    }
    return true;
  }

  /** Total number of traversable cells in the map. Cached after first call. */
  getTraversableCount(): number {
    if (this.traversableCount >= 0) return this.traversableCount;
    let count = 0;
    for (let y = 0; y < this.map.height; y++) {
      for (let x = 0; x < this.map.width; x++) {
        if (this.isTraversable(x, y)) count++;
      }
    }
    this.traversableCount = count;
    return count;
  }

  /**
   * Bresenham line-of-sight check between two grid cells.
   * Returns true if no wall/obstacle cells are crossed (exclusive of endpoints).
   */
  hasLineOfSight(x0: number, y0: number, x1: number, y1: number): boolean {
    let cx = x0;
    let cy = y0;
    let dx = Math.abs(x1 - cx);
    let dy = Math.abs(y1 - cy);
    const sx = cx < x1 ? 1 : -1;
    const sy = cy < y1 ? 1 : -1;
    let err = dx - dy;
    const maxSteps = dx + dy + 1;

    for (let i = 0; i < maxSteps; i++) {
      // Skip start and end
      if ((cx !== x0 || cy !== y0) && (cx !== x1 || cy !== y1)) {
        if (this.isOccupied(cx, cy)) return false;
      }
      if (cx === x1 && cy === y1) break;
      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
    return true;
  }

  /**
   * Cast a ray from a world-coordinate position at an angle.
   * Returns the distance to the first wall hit (in meters), or maxRange.
   * Each cell is 1 meter.
   */
  castRay(worldX: number, worldY: number, angle: number, maxRange: number): number {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);

    let gx = Math.floor(worldX);
    let gy = Math.floor(worldY);

    const stepX = dx >= 0 ? 1 : -1;
    const stepY = dy >= 0 ? 1 : -1;

    const tDeltaX = Math.abs(dx) > 1e-10 ? Math.abs(1.0 / dx) : 1e10;
    const tDeltaY = Math.abs(dy) > 1e-10 ? Math.abs(1.0 / dy) : 1e10;

    let tMaxX: number;
    let tMaxY: number;
    if (dx >= 0) {
      tMaxX = ((gx + 1) - worldX) / dx;
    } else {
      tMaxX = (gx - worldX) / dx;
    }
    if (dy >= 0) {
      tMaxY = ((gy + 1) - worldY) / dy;
    } else {
      tMaxY = (gy - worldY) / dy;
    }

    const maxSteps = Math.ceil(maxRange * 2) + 2;
    for (let i = 0; i < maxSteps; i++) {
      // The distance to the NEXT cell boundary crossing
      const crossDist = Math.min(tMaxX, tMaxY);
      if (crossDist > maxRange) return maxRange;

      // Step into the next cell
      if (tMaxX < tMaxY) {
        gx += stepX;
        tMaxX += tDeltaX;
      } else {
        gy += stepY;
        tMaxY += tDeltaY;
      }

      // If the new cell is a wall or out of bounds, the ray hit at crossDist
      if (!this.inBounds(gx, gy) || this.isOccupied(gx, gy)) {
        return crossDist;
      }
    }
    return maxRange;
  }

  /** Get the grid as a 2D string array for visualization */
  getGridStrings(): string[][] {
    const grid: string[][] = [];
    for (let y = 0; y < this.map.height; y++) {
      const row: string[] = [];
      for (let x = 0; x < this.map.width; x++) {
        row.push(this.map.cells[y][x] as string);
      }
      grid.push(row);
    }
    return grid;
  }
}

// ---------------------------------------------------------------------------
// Map Listing
// ---------------------------------------------------------------------------

export function listMaps(mapsDir: string): string[] {
  if (!fs.existsSync(mapsDir)) return [];
  return fs.readdirSync(mapsDir)
    .filter((f) => f.startsWith("map-") && f.endsWith(".txt"))
    .sort();
}
