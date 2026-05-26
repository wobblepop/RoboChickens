// ===========================================================================
// Edge Map Manager
// Manages the "treadmill" for the current edge being traversed.
// As the robot moves between two nodes, this accumulates the corridor data:
// distance traveled, bearing changes, and tracks which grid cells belong
// to this edge's corridor map.
// ===========================================================================

import type { Vec2, Pose2D, OccupancyGrid } from "../../shared/types";
import { generateId } from "../../shared/uuid";
import { vec2Distance, normalizeAngle } from "../../shared/math";
import { extractRegion, gridToVizCells } from "../slam/occupancy-grid";

// ---------------------------------------------------------------------------
// Edge Treadmill State
// ---------------------------------------------------------------------------

export interface ActiveEdge {
  id: string;
  source_node_id: string;
  /** Set when a new feature is detected (edge is completed) */
  target_node_id: string | null;
  /** Starting pose when the edge began */
  start_pose: Pose2D;
  /** Accumulated distance along this edge (meters) */
  distance: number;
  /** Accumulated bearing change (radians) */
  bearing_change: number;
  /** Trail of poses along the edge (sampled every few steps) */
  trajectory: Pose2D[];
  /** Whether this edge has been completed */
  completed: boolean;
  /** Timestamp when edge traversal started */
  started_at: number;
  /** Timestamp when edge was completed */
  completed_at: number | null;
}

export interface CompletedEdgeData {
  id: string;
  source_node_id: string;
  target_node_id: string;
  distance: number;
  bearing_change: number;
  trajectory: Pose2D[];
  corridor_map_cells: { x: number; y: number; state: string; confidence: number }[];
}

// ---------------------------------------------------------------------------
// Edge Map Manager
// ---------------------------------------------------------------------------

export class EdgeMapManager {
  private activeEdge: ActiveEdge | null = null;
  private completedEdges: CompletedEdgeData[] = [];
  private readonly trajectorySampleDistance: number = 0.3; // sample every 30cm
  private lastSamplePose: Pose2D | null = null;

  /** Get the currently active edge (being traversed) */
  getActiveEdge(): ActiveEdge | null {
    return this.activeEdge;
  }

  /** Get all completed edges */
  getCompletedEdges(): CompletedEdgeData[] {
    return this.completedEdges;
  }

  /**
   * Start a new edge from a source node.
   * Called when the robot departs from a node.
   */
  startEdge(sourceNodeId: string, currentPose: Pose2D, timestamp: number): string {
    const id = generateId();
    this.activeEdge = {
      id,
      source_node_id: sourceNodeId,
      target_node_id: null,
      start_pose: { ...currentPose },
      distance: 0,
      bearing_change: 0,
      trajectory: [{ ...currentPose }],
      completed: false,
      started_at: timestamp,
      completed_at: null,
    };
    this.lastSamplePose = { ...currentPose };
    return id;
  }

  /**
   * Update the active edge with the robot's current movement.
   * Called every tick while traversing.
   */
  update(currentPose: Pose2D, previousPose: Pose2D): void {
    if (!this.activeEdge || this.activeEdge.completed) return;

    // Accumulate distance
    const dx = currentPose.x - previousPose.x;
    const dy = currentPose.y - previousPose.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    this.activeEdge.distance += dist;

    // Accumulate bearing change
    const dtheta = normalizeAngle(currentPose.theta - previousPose.theta);
    this.activeEdge.bearing_change += dtheta;

    // Sample trajectory at regular intervals
    if (this.lastSamplePose) {
      const sampleDist = vec2Distance(
        { x: currentPose.x, y: currentPose.y },
        { x: this.lastSamplePose.x, y: this.lastSamplePose.y },
      );
      if (sampleDist >= this.trajectorySampleDistance) {
        this.activeEdge.trajectory.push({ ...currentPose });
        this.lastSamplePose = { ...currentPose };
      }
    }
  }

  /**
   * Complete the active edge when a new node is detected at the target end.
   * Extracts the corridor map from the SLAM grid along the trajectory.
   */
  completeEdge(
    targetNodeId: string,
    slamGrid: OccupancyGrid,
    timestamp: number,
    graphEdgeId?: string,
  ): CompletedEdgeData | null {
    if (!this.activeEdge || this.activeEdge.completed) return null;

    this.activeEdge.target_node_id = targetNodeId;
    this.activeEdge.completed = true;
    this.activeEdge.completed_at = timestamp;

    // Extract corridor map: get the grid region along the trajectory
    const corridorCells = this.extractCorridorMap(slamGrid);

    const completed: CompletedEdgeData = {
      id: graphEdgeId ?? this.activeEdge.id,
      source_node_id: this.activeEdge.source_node_id,
      target_node_id: targetNodeId,
      distance: this.activeEdge.distance,
      bearing_change: this.activeEdge.bearing_change,
      trajectory: [...this.activeEdge.trajectory],
      corridor_map_cells: corridorCells,
    };

    this.completedEdges.push(completed);
    this.activeEdge = null;
    this.lastSamplePose = null;

    return completed;
  }

  /**
   * Cancel the active edge (e.g., robot got lost or returned to source).
   */
  cancelActiveEdge(): void {
    this.activeEdge = null;
    this.lastSamplePose = null;
  }

  /**
   * Get corridor map data for a completed edge (for visualization popup).
   */
  getCorridorMapForEdge(edgeId: string): CompletedEdgeData | null {
    return this.completedEdges.find((e) => e.id === edgeId) ?? null;
  }

  /** Reset all state */
  reset(): void {
    this.activeEdge = null;
    this.completedEdges = [];
    this.lastSamplePose = null;
  }

  // -------------------------------------------------------------------------
  // Private
  // -------------------------------------------------------------------------

  /**
   * Extract corridor cells from the SLAM grid along the trajectory.
   * Takes a 2m-wide strip centered on the trajectory path.
   */
  private extractCorridorMap(
    slamGrid: OccupancyGrid,
  ): { x: number; y: number; state: string; confidence: number }[] {
    if (!this.activeEdge) return [];
    const trajectory = this.activeEdge.trajectory;
    if (trajectory.length < 2) return [];

    // Use the midpoint of the trajectory as center, with radius = half the distance
    const midIdx = Math.floor(trajectory.length / 2);
    const midPose = trajectory[midIdx];
    const radius = Math.max(2.0, this.activeEdge.distance / 2 + 1.0);

    const region = extractRegion(slamGrid, midPose.x, midPose.y, radius);
    return gridToVizCells(region);
  }
}
