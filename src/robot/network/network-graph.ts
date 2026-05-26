// ===========================================================================
// Network Graph
// The robot's local copy of the topological graph (nodes + edges).
// Nodes represent places discovered by feature detection or received from
// the Gun.js network. Edges represent visibility-verified connections
// between nodes (traveled or untraveled).
//
// Key concepts for the exploration algorithm:
//   - explored node: a robot has physically arrived and completed a 360 scan
//   - unexplored node: detected but no robot has scanned from there yet
//   - traveled edge: a robot has physically traversed this connection
//   - untraveled edge: visibility-verified connection, not yet traversed
// ===========================================================================

import type {
  Vec2,
  NetworkNode,
  NetworkEdge,
  DetectedFeature,
} from "../../shared/types";
import {
  FeatureType,
  StabilityClass,
  ConfidenceTier,
  EdgeType,
  EdgeStatus,
  defaultNodeExtensions,
  defaultEdgeExtensions,
  defaultTraversability,
  confidenceTierFromScore,
} from "../../shared/types";
import { generateId } from "../../shared/uuid";
import { vec2Distance } from "../../shared/math";

// ---------------------------------------------------------------------------
// Network Graph
// ---------------------------------------------------------------------------

export class NetworkGraph {
  private nodes: Map<string, NetworkNode> = new Map();
  private edges: Map<string, NetworkEdge> = new Map();
  private adjacency: Map<string, Set<string>> = new Map();
  private readonly robotId: string;

  constructor(robotId: string) {
    this.robotId = robotId;
  }

  // =========================================================================
  // Node Operations
  // =========================================================================

  createNode(feature: DetectedFeature, localMapHash: string, timestamp: number): NetworkNode {
    const node: NetworkNode = {
      id: generateId(),
      type: feature.type,
      position: { ...feature.position },
      confidence: feature.confidence,
      observation_count: 1,
      exit_count: feature.exit_directions.length,
      local_map_hash: localMapHash,
      discovered_by: this.robotId,
      discovered_at: timestamp,
      ...defaultNodeExtensions(),
    };
    node.last_observed = timestamp;
    this.nodes.set(node.id, node);
    this.adjacency.set(node.id, new Set());
    return node;
  }

  createNodeAtPosition(
    position: Vec2,
    type: FeatureType,
    exitCount: number,
    timestamp: number,
  ): NetworkNode {
    const node: NetworkNode = {
      id: generateId(),
      type,
      position: { ...position },
      confidence: 0.8,
      observation_count: 1,
      exit_count: exitCount,
      local_map_hash: "",
      discovered_by: this.robotId,
      discovered_at: timestamp,
      ...defaultNodeExtensions(),
    };
    node.tier = ConfidenceTier.Provisional;
    node.last_observed = timestamp;
    this.nodes.set(node.id, node);
    this.adjacency.set(node.id, new Set());
    return node;
  }

  addExternalNode(node: NetworkNode): void {
    if (this.nodes.has(node.id)) return;
    this.nodes.set(node.id, { ...node });
    if (!this.adjacency.has(node.id)) {
      this.adjacency.set(node.id, new Set());
    }
  }

  removeEdge(edgeId: string): void {
    const edge = this.edges.get(edgeId);
    if (!edge) return;
    this.adjacency.get(edge.source_node)?.delete(edgeId);
    this.adjacency.get(edge.target_node)?.delete(edgeId);
    this.edges.delete(edgeId);
  }

  removeNode(nodeId: string): void {
    this.nodes.delete(nodeId);
    const edgeIds = this.adjacency.get(nodeId);
    if (edgeIds) {
      for (const eid of edgeIds) this.edges.delete(eid);
    }
    this.adjacency.delete(nodeId);
  }

  markNodeExplored(nodeId: string, robotId: string, timestamp: number): void {
    const node = this.nodes.get(nodeId);
    if (!node) return;
    node.explored = true;
    if (!node.explored_by.includes(robotId)) {
      node.explored_by.push(robotId);
    }
    node.last_explored_at = timestamp;
    node.observation_count++;
    node.last_observed = timestamp;
    node.version++;
  }

  strengthenNode(nodeId: string): void {
    const node = this.nodes.get(nodeId);
    if (!node) return;
    node.observation_count++;
    node.confidence = 1 - Math.exp(-0.25 * node.observation_count);
    node.tier = confidenceTierFromScore(node.confidence);
    node.version++;
  }

  getNode(nodeId: string): NetworkNode | null {
    return this.nodes.get(nodeId) ?? null;
  }

  getAllNodes(): NetworkNode[] {
    return Array.from(this.nodes.values());
  }

  // =========================================================================
  // Exploration-Specific Node Queries
  // =========================================================================

  getUnexploredNodes(): NetworkNode[] {
    const result: NetworkNode[] = [];
    for (const node of this.nodes.values()) {
      if (!node.explored) result.push(node);
    }
    return result;
  }

  getUnexploredAdjacentNodes(nodeId: string): NetworkNode[] {
    return this.getNeighbors(nodeId).filter(n => !n.explored);
  }

  getAdjacentNodeWithMostUnmappedEdges(nodeId: string): NetworkNode | null {
    const neighbors = this.getNeighbors(nodeId);
    let best: NetworkNode | null = null;
    let bestCount = -1;
    for (const neighbor of neighbors) {
      const untraveled = this.getUntraveledEdgesForNode(neighbor.id);
      if (untraveled.length > bestCount) {
        bestCount = untraveled.length;
        best = neighbor;
      }
    }
    return bestCount > 0 ? best : null;
  }

  // =========================================================================
  // Spatial Queries
  // =========================================================================

  findNearestNode(position: Vec2): NetworkNode | null {
    let nearest: NetworkNode | null = null;
    let minDist = Infinity;
    for (const node of this.nodes.values()) {
      const dist = vec2Distance(position, node.position);
      if (dist < minDist) {
        minDist = dist;
        nearest = node;
      }
    }
    return nearest;
  }

  findNodesInRadius(position: Vec2, radius: number): NetworkNode[] {
    const result: NetworkNode[] = [];
    for (const node of this.nodes.values()) {
      if (vec2Distance(position, node.position) <= radius) {
        result.push(node);
      }
    }
    return result;
  }

  findNearestUnexploredNode(position: Vec2, maxRadius: number = Infinity): NetworkNode | null {
    let nearest: NetworkNode | null = null;
    let minDist = Infinity;
    for (const node of this.nodes.values()) {
      if (node.explored) continue;
      const dist = vec2Distance(position, node.position);
      if (dist < minDist && dist <= maxRadius) {
        minDist = dist;
        nearest = node;
      }
    }
    return nearest;
  }

  // =========================================================================
  // Edge Operations
  // =========================================================================

  createEdge(
    sourceNodeId: string,
    targetNodeId: string,
    distance: number,
    bearingChange: number,
    timestamp: number,
    traveled: boolean = false,
  ): NetworkEdge | null {
    if (!this.nodes.has(sourceNodeId) || !this.nodes.has(targetNodeId)) return null;
    if (this.getEdgeBetween(sourceNodeId, targetNodeId)) return null;

    const edge: NetworkEdge = {
      id: generateId(),
      source_node: sourceNodeId,
      target_node: targetNodeId,
      distance,
      bearing_change: bearingChange,
      bidirectional: true,
      confidence: traveled ? 0.7 : 0.4,
      corridor_map_hash: "",
      discovered_by: this.robotId,
      discovered_at: timestamp,
      ...defaultEdgeExtensions(),
    };
    edge.traveled = traveled;
    edge.traveled_by = traveled ? [this.robotId] : [];
    edge.tier = traveled ? ConfidenceTier.Provisional : ConfidenceTier.Candidate;
    edge.last_observed = timestamp;

    this.edges.set(edge.id, edge);
    this.adjacency.get(sourceNodeId)?.add(edge.id);
    this.adjacency.get(targetNodeId)?.add(edge.id);
    return edge;
  }

  markEdgeTraveled(
    edgeId: string,
    robotId: string,
    measuredDistance: number | null,
    measuredBearingChange: number | null,
    timestamp: number,
  ): void {
    const edge = this.edges.get(edgeId);
    if (!edge) return;
    edge.traveled = true;
    if (!edge.traveled_by.includes(robotId)) {
      edge.traveled_by.push(robotId);
    }
    if (measuredDistance !== null) {
      edge.distance = measuredDistance;
    }
    if (measuredBearingChange !== null) {
      edge.bearing_change = measuredBearingChange;
    }
    edge.confidence = Math.min(1.0, edge.confidence + 0.3);
    edge.tier = confidenceTierFromScore(edge.confidence);
    edge.last_observed = timestamp;
    edge.version++;
  }

  addExternalEdge(edge: NetworkEdge): void {
    if (this.edges.has(edge.id)) return;
    this.edges.set(edge.id, { ...edge });
    this.adjacency.get(edge.source_node)?.add(edge.id);
    this.adjacency.get(edge.target_node)?.add(edge.id);
  }

  getEdge(edgeId: string): NetworkEdge | null {
    return this.edges.get(edgeId) ?? null;
  }

  getAllEdges(): NetworkEdge[] {
    return Array.from(this.edges.values());
  }

  getNodeEdges(nodeId: string): NetworkEdge[] {
    const edgeIds = this.adjacency.get(nodeId);
    if (!edgeIds) return [];
    const edges: NetworkEdge[] = [];
    for (const eid of edgeIds) {
      const edge = this.edges.get(eid);
      if (edge) edges.push(edge);
    }
    return edges;
  }

  getEdgeBetween(nodeA: string, nodeB: string): NetworkEdge | null {
    const edgesA = this.adjacency.get(nodeA);
    if (!edgesA) return null;
    for (const eid of edgesA) {
      const edge = this.edges.get(eid);
      if (!edge) continue;
      if ((edge.source_node === nodeA && edge.target_node === nodeB) ||
          (edge.source_node === nodeB && edge.target_node === nodeA)) {
        return edge;
      }
    }
    return null;
  }

  hasEdgeBetween(nodeA: string, nodeB: string): boolean {
    return this.getEdgeBetween(nodeA, nodeB) !== null;
  }

  getNeighbors(nodeId: string): NetworkNode[] {
    const edges = this.getNodeEdges(nodeId);
    const neighbors: NetworkNode[] = [];
    for (const edge of edges) {
      const otherId = edge.source_node === nodeId ? edge.target_node : edge.source_node;
      const other = this.nodes.get(otherId);
      if (other) neighbors.push(other);
    }
    return neighbors;
  }

  getOtherNode(edge: NetworkEdge, fromNodeId: string): NetworkNode | null {
    const otherId = edge.source_node === fromNodeId ? edge.target_node : edge.source_node;
    return this.nodes.get(otherId) ?? null;
  }

  // =========================================================================
  // Exploration-Specific Edge Queries
  // =========================================================================

  getUntraveledEdgesForNode(nodeId: string): NetworkEdge[] {
    return this.getNodeEdges(nodeId).filter(e => !e.traveled);
  }

  getTraveledEdgesForNode(nodeId: string): NetworkEdge[] {
    return this.getNodeEdges(nodeId).filter(e => e.traveled);
  }

  getAllUntraveledEdges(): NetworkEdge[] {
    const result: NetworkEdge[] = [];
    for (const edge of this.edges.values()) {
      if (!edge.traveled) result.push(edge);
    }
    return result;
  }

  /**
   * Choose the best edge to travel from a node using the exploration priority:
   *   1. Edges leading to unexplored adjacent nodes
   *   2. Untraveled edges (to already-explored nodes)
   *   3. Edges to adjacent nodes with the most unmapped edges
   *   4. Random edge
   */
  chooseExplorationEdge(nodeId: string): { edge: NetworkEdge; reason: string } | null {
    const edges = this.getNodeEdges(nodeId);
    if (edges.length === 0) return null;

    // Priority 0: BFS path toward any unexplored node in the graph
    const pathEdge = this.findFirstEdgeTowardUnexplored(nodeId);
    if (pathEdge) {
      return { edge: pathEdge, reason: "path_to_unexplored" };
    }

    // Priority 1: untraveled edges
    const untraveled = edges.filter(e => !e.traveled);
    if (untraveled.length > 0) {
      return { edge: this.pickNearestEdge(untraveled), reason: "untraveled_edge" };
    }

    // Priority 2: adjacent node with the most unmapped edges
    let bestEdge: NetworkEdge | null = null;
    let bestUnmappedCount = 0;
    for (const edge of edges) {
      const other = this.getOtherNode(edge, nodeId);
      if (!other) continue;
      const unmapped = this.getUntraveledEdgesForNode(other.id).length;
      if (unmapped > bestUnmappedCount) {
        bestUnmappedCount = unmapped;
        bestEdge = edge;
      }
    }
    if (bestEdge) {
      return { edge: bestEdge, reason: "most_unmapped_neighbor" };
    }

    // Priority 3: random edge
    const randomIdx = Math.floor(Math.random() * edges.length);
    return { edge: edges[randomIdx], reason: "random" };
  }

  /**
   * BFS from nodeId to find any unexplored node. Returns the first edge
   * on the shortest path from nodeId toward that unexplored node.
   */
  private findFirstEdgeTowardUnexplored(startId: string): NetworkEdge | null {
    const unexplored = this.getUnexploredNodes();
    if (unexplored.length === 0) return null;
    const unexploredIds = new Set(unexplored.map(n => n.id));

    // BFS tracking parent edges
    const visited = new Set<string>([startId]);
    const parentEdge = new Map<string, NetworkEdge>();
    const queue: string[] = [startId];

    while (queue.length > 0) {
      const current = queue.shift()!;
      if (current !== startId && unexploredIds.has(current)) {
        // Trace back to find the first edge from startId
        let node = current;
        while (parentEdge.has(node)) {
          const edge = parentEdge.get(node)!;
          const prev = edge.source_node === node ? edge.target_node : edge.source_node;
          if (prev === startId) return edge;
          node = prev;
        }
        return null;
      }

      const nodeEdges = this.getNodeEdges(current);
      for (const edge of nodeEdges) {
        const other = this.getOtherNode(edge, current);
        if (!other || visited.has(other.id)) continue;
        visited.add(other.id);
        parentEdge.set(other.id, edge);
        queue.push(other.id);
      }
    }

    return null;
  }

  // =========================================================================
  // Frontier Detection
  // =========================================================================

  getFrontierNodes(): NetworkNode[] {
    const frontiers: NetworkNode[] = [];
    for (const node of this.nodes.values()) {
      const connectedEdges = this.adjacency.get(node.id);
      const edgeCount = connectedEdges ? connectedEdges.size : 0;
      if (edgeCount < node.exit_count) {
        frontiers.push(node);
      }
    }
    return frontiers;
  }

  // =========================================================================
  // Graph Pathfinding (BFS shortest path through edges)
  // =========================================================================

  /**
   * Find the shortest path (by hop count) from one node to another.
   * Only traverses non-blocked edges. Returns an ordered list of
   * { nodeId, edgeId } hops, or null if no path exists.
   */
  findPath(fromId: string, toId: string): { nodeId: string; edgeId: string }[] | null {
    if (fromId === toId) return [];
    if (!this.nodes.has(fromId) || !this.nodes.has(toId)) return null;

    const visited = new Set<string>();
    const parent = new Map<string, { nodeId: string; edgeId: string }>();
    const queue: string[] = [fromId];
    visited.add(fromId);

    while (queue.length > 0) {
      const current = queue.shift()!;
      const edges = this.getNodeEdges(current);

      for (const edge of edges) {
        if (edge.traversability.status === EdgeStatus.Blocked) continue;

        const neighborId = edge.source_node === current ? edge.target_node : edge.source_node;
        if (visited.has(neighborId)) continue;

        visited.add(neighborId);
        parent.set(neighborId, { nodeId: current, edgeId: edge.id });

        if (neighborId === toId) {
          // Reconstruct path
          const path: { nodeId: string; edgeId: string }[] = [];
          let cur = toId;
          while (cur !== fromId) {
            const p = parent.get(cur)!;
            path.unshift({ nodeId: cur, edgeId: p.edgeId });
            cur = p.nodeId;
          }
          return path;
        }

        queue.push(neighborId);
      }
    }

    return null;
  }

  // =========================================================================
  // Graph Statistics
  // =========================================================================

  get nodeCount(): number { return this.nodes.size; }
  get edgeCount(): number { return this.edges.size; }

  get exploredNodeCount(): number {
    let count = 0;
    for (const node of this.nodes.values()) {
      if (node.explored) count++;
    }
    return count;
  }

  get traveledEdgeCount(): number {
    let count = 0;
    for (const edge of this.edges.values()) {
      if (edge.traveled) count++;
    }
    return count;
  }

  // =========================================================================
  // Trimming: Confidence Decay and Pruning
  // =========================================================================

  /**
   * Apply freshness decay to all entities based on their stability class.
   * Called periodically (e.g., every 50 ticks).
   *
   * Static features (walls, columns) don't decay.
   * Semi-static features (shelving) decay slowly: lambda=0.005/hour.
   * Dynamic features (doors, mobile objects) decay fast: lambda=0.05/hour.
   *
   * When an entity's confidence drops below the dead tier (0.10), it becomes
   * a candidate for pruning.
   */
  applyFreshnessDecay(currentTimestamp: number): void {
    // Sim-scaled decay: use minutes (60_000ms) not real hours (3_600_000ms).
    // At 100ms/tick, 600 ticks = 1 sim-minute. Semi-static decays slowly,
    // dynamic decays ~10x faster.
    for (const node of this.nodes.values()) {
      if (node.stability_class === StabilityClass.Static) continue;
      const minutesSinceObserved = Math.max(0, (currentTimestamp - node.last_observed) / 60_000);
      if (minutesSinceObserved <= 0) continue;

      const lambda = node.stability_class === StabilityClass.SemiStatic ? 0.01 : 0.1;
      const decayFactor = Math.exp(-lambda * minutesSinceObserved);
      node.confidence *= decayFactor;
      node.tier = confidenceTierFromScore(node.confidence);
    }

    for (const edge of this.edges.values()) {
      if (edge.stability_class === StabilityClass.Static) continue;
      const minutesSinceObserved = Math.max(0, (currentTimestamp - edge.last_observed) / 60_000);
      if (minutesSinceObserved <= 0) continue;

      const lambda = edge.stability_class === StabilityClass.SemiStatic ? 0.01 : 0.1;
      const decayFactor = Math.exp(-lambda * minutesSinceObserved);
      edge.confidence *= decayFactor;
      edge.tier = confidenceTierFromScore(edge.confidence);
    }
  }

  /**
   * Mark an edge as blocked. The edge remains in the graph (it represents
   * a known connection that is currently impassable). Confidence is reduced
   * but floored at Candidate tier — blocked edges should never be pruned,
   * because they record real topology that may become passable again.
   */
  invalidateEdge(edgeId: string, timestamp: number): void {
    const edge = this.edges.get(edgeId);
    if (!edge) return;
    edge.traversability.status = EdgeStatus.Blocked;
    edge.confidence = Math.max(0.15, edge.confidence * 0.5);
    edge.tier = confidenceTierFromScore(edge.confidence);
    edge.last_observed = timestamp;
    edge.version++;
  }

  /**
   * Weaken a node when a robot re-scans the area and finds the layout
   * has changed (e.g., a doorway is now blocked, a passage is gone).
   * Reduces confidence proportionally to the mismatch severity.
   */
  weakenNode(nodeId: string, mismatchSeverity: number, timestamp: number): void {
    const node = this.nodes.get(nodeId);
    if (!node) return;
    const penalty = Math.max(0.1, 1.0 - mismatchSeverity);
    node.confidence *= penalty;
    node.tier = confidenceTierFromScore(node.confidence);
    node.last_observed = timestamp;
    node.version++;
  }

  /**
   * Prune dead-tier entities from the graph.
   * Returns lists of pruned node and edge IDs.
   *
   * Protocol:
   *   1. Remove all dead-tier edges
   *   2. Check for orphaned nodes (no remaining edges above dead tier)
   *   3. Remove orphaned dead-tier nodes
   *   4. If a pruned edge was the only connection between two subgraphs,
   *      flag as disconnection event (don't auto-prune — may indicate error)
   */
  pruneDeadEntities(): { prunedNodeIds: string[]; prunedEdgeIds: string[]; disconnections: string[] } {
    const prunedEdgeIds: string[] = [];
    const prunedNodeIds: string[] = [];
    const disconnections: string[] = [];

    // Step 1: Remove dead-tier edges — but NEVER prune edges that have been
    // traveled or are marked as blocked. These represent confirmed topology
    // that is currently impassable. They should remain on the map as blocked
    // connections, not disappear.
    for (const [edgeId, edge] of this.edges) {
      if (edge.tier !== ConfidenceTier.Dead) continue;

      // Keep traveled edges — they represent real paths that may reopen
      if (edge.traveled) {
        edge.traversability.status = EdgeStatus.Blocked;
        edge.confidence = 0.10;
        edge.tier = ConfidenceTier.Candidate;
        continue;
      }

      // Keep explicitly blocked edges — they're information, not garbage
      if (edge.traversability.status === EdgeStatus.Blocked) {
        edge.confidence = 0.10;
        edge.tier = ConfidenceTier.Candidate;
        continue;
      }

      // Only prune untraveled, non-blocked dead edges (speculative connections
      // that were never confirmed and have decayed)
      this.adjacency.get(edge.source_node)?.delete(edgeId);
      this.adjacency.get(edge.target_node)?.delete(edgeId);
      this.edges.delete(edgeId);
      prunedEdgeIds.push(edgeId);
    }

    // Step 2: Find orphaned dead-tier nodes — but never prune explored nodes
    // (they represent real places the robot has been)
    for (const [nodeId, node] of this.nodes) {
      if (node.tier !== ConfidenceTier.Dead) continue;
      if (node.explored) continue;

      const edges = this.adjacency.get(nodeId);
      const hasLiveEdges = edges ? edges.size > 0 : false;

      if (!hasLiveEdges) {
        this.adjacency.delete(nodeId);
        this.nodes.delete(nodeId);
        prunedNodeIds.push(nodeId);
      }
    }

    // Step 3: Check for disconnection events
    if (prunedEdgeIds.length > 0) {
      const components = this.findConnectedComponents();
      if (components.length > 1) {
        disconnections.push(`Graph split into ${components.length} components after pruning`);
      }
    }

    return { prunedNodeIds, prunedEdgeIds, disconnections };
  }

  /**
   * Re-validate edges from a node by checking LiDAR visibility.
   * Called when a robot arrives at a node and re-scans.
   * Edges whose endpoints are no longer visible get weakened.
   */
  revalidateEdgesFromNode(
    nodeId: string,
    visibleNodeIds: Set<string>,
    timestamp: number,
  ): void {
    const edges = this.getNodeEdges(nodeId);
    for (const edge of edges) {
      const otherId = edge.source_node === nodeId ? edge.target_node : edge.source_node;

      if (visibleNodeIds.has(otherId)) {
        // Still visible — refresh the edge and clear any blocked status
        edge.last_observed = timestamp;
        edge.confidence = Math.min(1.0, edge.confidence + 0.05);
        edge.tier = confidenceTierFromScore(edge.confidence);
        if (edge.traversability.status === EdgeStatus.Blocked) {
          edge.traversability.status = EdgeStatus.Open;
        }
      } else if (edge.traveled || edge.traversability.status === EdgeStatus.Open) {
        // Was a confirmed/open path, now blocked — mark as blocked but keep it
        edge.traversability.status = EdgeStatus.Blocked;
        edge.confidence = Math.max(0.15, edge.confidence * 0.7);
        edge.tier = confidenceTierFromScore(edge.confidence);
        edge.last_observed = timestamp;
        edge.version++;
      } else {
        // Untraveled speculative edge no longer visible — weaken it
        edge.confidence *= 0.5;
        edge.tier = confidenceTierFromScore(edge.confidence);
        edge.version++;
      }
    }
  }

  // =========================================================================
  // Connected Components (for disconnection detection)
  // =========================================================================

  private findConnectedComponents(): string[][] {
    const visited = new Set<string>();
    const components: string[][] = [];

    for (const nodeId of this.nodes.keys()) {
      if (visited.has(nodeId)) continue;
      const component: string[] = [];
      const stack = [nodeId];
      while (stack.length > 0) {
        const current = stack.pop()!;
        if (visited.has(current)) continue;
        visited.add(current);
        component.push(current);
        const neighbors = this.getNeighbors(current);
        for (const n of neighbors) {
          if (!visited.has(n.id)) stack.push(n.id);
        }
      }
      components.push(component);
    }

    return components;
  }

  // =========================================================================
  // Reset
  // =========================================================================

  reset(): void {
    this.nodes.clear();
    this.edges.clear();
    this.adjacency.clear();
  }

  // =========================================================================
  // Private Helpers
  // =========================================================================

  private pickNearestEdge(edges: NetworkEdge[]): NetworkEdge {
    if (edges.length === 1) return edges[0];
    let best = edges[0];
    let bestDist = best.distance;
    for (let i = 1; i < edges.length; i++) {
      if (edges[i].distance < bestDist) {
        bestDist = edges[i].distance;
        best = edges[i];
      }
    }
    return best;
  }
}
