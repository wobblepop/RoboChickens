// ===========================================================================
// Node Map Manager
// Manages local area maps for each detected node.
// A node's local map is the composite of SLAM grid data around the node,
// plus additional scans taken while the robot is near the node.
// ===========================================================================

import type { Vec2, OccupancyGrid, DetectedFeature, LidarScan } from "../../shared/types";
import { extractRegion, gridToVizCells } from "../slam/occupancy-grid";

// ---------------------------------------------------------------------------
// Node Map Data
// ---------------------------------------------------------------------------

export interface NodeLocalMap {
  node_id: string;
  /** Position of the node in SLAM frame */
  position: Vec2;
  /** Radius of the local map region (meters) */
  radius: number;
  /** Viz-format cells extracted from the SLAM grid */
  cells: { x: number; y: number; state: string; confidence: number }[];
  /** Reference LiDAR scans taken at this node (for loop closure matching) */
  reference_scans: LidarScan[];
  /** Content hash of the local map (for network dedup/matching) */
  content_hash: string;
  /** Edge IDs that connect to this node */
  connected_edges: string[];
  /** Number of times the robot has visited this node */
  visit_count: number;
}

// ---------------------------------------------------------------------------
// Node Map Manager
// ---------------------------------------------------------------------------

export class NodeMapManager {
  private nodeMaps: Map<string, NodeLocalMap> = new Map();

  /**
   * Create a local map for a newly detected node.
   * Extracts the grid region around the feature position.
   */
  createNodeMap(
    nodeId: string,
    feature: DetectedFeature,
    slamGrid: OccupancyGrid,
    referenceScan: LidarScan | null,
  ): NodeLocalMap {
    const region = extractRegion(
      slamGrid,
      feature.position.x,
      feature.position.y,
      feature.local_radius,
    );
    const cells = gridToVizCells(region);
    const hash = this.computeHash(cells);

    const nodeMap: NodeLocalMap = {
      node_id: nodeId,
      position: { ...feature.position },
      radius: feature.local_radius,
      cells,
      reference_scans: referenceScan ? [referenceScan] : [],
      content_hash: hash,
      connected_edges: [],
      visit_count: 1,
    };

    this.nodeMaps.set(nodeId, nodeMap);
    return nodeMap;
  }

  /**
   * Update a node's local map with fresh data (robot revisiting).
   */
  updateNodeMap(
    nodeId: string,
    slamGrid: OccupancyGrid,
    referenceScan: LidarScan | null,
  ): void {
    const existing = this.nodeMaps.get(nodeId);
    if (!existing) return;

    // Re-extract grid region
    const region = extractRegion(
      slamGrid,
      existing.position.x,
      existing.position.y,
      existing.radius,
    );
    existing.cells = gridToVizCells(region);
    existing.content_hash = this.computeHash(existing.cells);
    existing.visit_count++;

    // Add reference scan (keep max 5 for memory)
    if (referenceScan) {
      existing.reference_scans.push(referenceScan);
      if (existing.reference_scans.length > 5) {
        existing.reference_scans.shift();
      }
    }
  }

  /**
   * Register an edge as connected to a node.
   */
  addEdgeToNode(nodeId: string, edgeId: string): void {
    const nodeMap = this.nodeMaps.get(nodeId);
    if (nodeMap && !nodeMap.connected_edges.includes(edgeId)) {
      nodeMap.connected_edges.push(edgeId);
    }
  }

  /**
   * Get a node's local map.
   */
  getNodeMap(nodeId: string): NodeLocalMap | null {
    return this.nodeMaps.get(nodeId) ?? null;
  }

  /**
   * Get all node maps.
   */
  getAllNodeMaps(): NodeLocalMap[] {
    return Array.from(this.nodeMaps.values());
  }

  /**
   * Get reference scans for a node (used for loop closure matching).
   */
  getReferenceScan(nodeId: string): LidarScan | null {
    const nodeMap = this.nodeMaps.get(nodeId);
    if (!nodeMap || nodeMap.reference_scans.length === 0) return null;
    // Return the most recent scan
    return nodeMap.reference_scans[nodeMap.reference_scans.length - 1];
  }

  /**
   * Get the content hash for a node's local map (for network publishing).
   */
  getContentHash(nodeId: string): string {
    return this.nodeMaps.get(nodeId)?.content_hash ?? "";
  }

  /** Reset all state */
  reset(): void {
    this.nodeMaps.clear();
  }

  // -------------------------------------------------------------------------
  // Private
  // -------------------------------------------------------------------------

  /**
   * Simple content hash based on cell states.
   * Not cryptographic — just for dedup/matching.
   */
  private computeHash(
    cells: { x: number; y: number; state: string; confidence: number }[],
  ): string {
    if (cells.length === 0) return "empty";
    // Hash based on cell count, state distribution, and rough spatial extent
    let wallCount = 0;
    let floorCount = 0;
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    for (const cell of cells) {
      if (cell.state === "wall") wallCount++;
      else floorCount++;
      if (cell.x < minX) minX = cell.x;
      if (cell.x > maxX) maxX = cell.x;
      if (cell.y < minY) minY = cell.y;
      if (cell.y > maxY) maxY = cell.y;
    }

    const extentX = Math.round((maxX - minX) * 10);
    const extentY = Math.round((maxY - minY) * 10);
    return `${cells.length}:${wallCount}:${floorCount}:${extentX}x${extentY}`;
  }
}
