// ===========================================================================
// Behavior Mode Strategies
//
// Each mode is a target-selection function called during the CHOOSING phase.
// All modes share the same ARRIVING, SCANNING, and TRAVELING phases.
// If a mode returns null, the caller falls back to default exploration.
// ===========================================================================

import type { NetworkNode, NetworkEdge, Pose2D, LidarScan } from "../../shared/types";
import { BehaviorMode, EdgeStatus, ConfidenceTier } from "../../shared/types";
import { vec2Distance } from "../../shared/math";
import type { NetworkGraph } from "../network/network-graph";

// ---------------------------------------------------------------------------
// Strategy Result
// ---------------------------------------------------------------------------

export interface BehaviorResult {
  edge: NetworkEdge;
  reason: string;
}

// ---------------------------------------------------------------------------
// Per-Robot Mode State
// ---------------------------------------------------------------------------

export interface BehaviorState {
  mode: BehaviorMode;
  startingNodeId: string | null;
  recheckCooldown: Map<string, number>;
  consolidateAttempted: Set<string>;
  patrolRoute: string[];
  patrolIndex: number;
}

export function createBehaviorState(): BehaviorState {
  return {
    mode: BehaviorMode.ExploreNew,
    startingNodeId: null,
    recheckCooldown: new Map(),
    consolidateAttempted: new Set(),
    patrolRoute: [],
    patrolIndex: 0,
  };
}

// ---------------------------------------------------------------------------
// Mode Dispatcher
// ---------------------------------------------------------------------------

export function chooseBehaviorTarget(
  state: BehaviorState,
  currentNodeId: string,
  graph: NetworkGraph,
  pose: Pose2D,
  currentTick: number,
): BehaviorResult | null {
  switch (state.mode) {
    case BehaviorMode.ExploreNew:
      return chooseExploreNew(currentNodeId, graph);
    case BehaviorMode.RecheckMap:
      return chooseRecheckMap(state, currentNodeId, graph, currentTick);
    case BehaviorMode.ConsolidatePathing:
      return chooseConsolidatePathing(state, currentNodeId, graph);
    case BehaviorMode.CheckStale:
      return chooseCheckStale(currentNodeId, graph);
    case BehaviorMode.CheckMainPathways:
      return chooseCheckMainPathways(state, currentNodeId, graph);
    case BehaviorMode.FindBoxes:
      return chooseFindBoxes(currentNodeId, graph, pose);
    case BehaviorMode.Idle:
      return null;
    case BehaviorMode.ReturnToStart:
      return chooseReturnToStart(state, currentNodeId, graph);
  }
}

// ===========================================================================
// 1. Explore New Areas (default behavior)
// ===========================================================================

function chooseExploreNew(
  currentNodeId: string,
  graph: NetworkGraph,
): BehaviorResult | null {
  return graph.chooseExplorationEdge(currentNodeId);
}

// ===========================================================================
// 2. Recheck Map — revisit oldest explored nodes
// ===========================================================================

function chooseRecheckMap(
  state: BehaviorState,
  currentNodeId: string,
  graph: NetworkGraph,
  currentTick: number,
): BehaviorResult | null {
  const cooldownTicks = 200;

  // Find explored nodes sorted by staleness (oldest last_explored_at first)
  const explored = graph.getAllNodes().filter(n => {
    if (!n.explored) return false;
    if (n.id === currentNodeId) return false;
    const lastRecheck = state.recheckCooldown.get(n.id) ?? 0;
    if (currentTick - lastRecheck < cooldownTicks) return false;
    return true;
  });

  if (explored.length === 0) return null;

  explored.sort((a, b) => a.last_explored_at - b.last_explored_at);

  // Prefer junctions (high edge count) among the oldest
  const stalest = explored.slice(0, Math.min(5, explored.length));
  stalest.sort((a, b) => {
    const aEdges = graph.getNodeEdges(a.id).length;
    const bEdges = graph.getNodeEdges(b.id).length;
    return bEdges - aEdges;
  });

  const target = stalest[0];
  const path = graph.findPath(currentNodeId, target.id);
  if (!path || path.length === 0) return null;

  state.recheckCooldown.set(target.id, currentTick);

  return {
    edge: graph.getEdge(path[0].edgeId)!,
    reason: "recheck_stale_node",
  };
}

// ===========================================================================
// 3. Consolidate Pathing — travel untraveled edges to verify them
// ===========================================================================

function chooseConsolidatePathing(
  state: BehaviorState,
  currentNodeId: string,
  graph: NetworkGraph,
): BehaviorResult | null {
  // Untraveled edges from current node, not already attempted
  const localUntraveled = graph.getUntraveledEdgesForNode(currentNodeId)
    .filter(e => {
      if (state.consolidateAttempted.has(e.id)) return false;
      if (e.traversability.status === EdgeStatus.Blocked) return false;
      return true;
    });

  if (localUntraveled.length > 0) {
    // Prefer lowest confidence (most speculative)
    localUntraveled.sort((a, b) => a.confidence - b.confidence);
    state.consolidateAttempted.add(localUntraveled[0].id);
    return { edge: localUntraveled[0], reason: "consolidate_local_untraveled" };
  }

  // No local untraveled — BFS to find nearest node with untraveled edges
  const allUntraveled = graph.getAllUntraveledEdges()
    .filter(e => {
      if (state.consolidateAttempted.has(e.id)) return false;
      if (e.traversability.status === EdgeStatus.Blocked) return false;
      return true;
    });

  if (allUntraveled.length === 0) {
    state.consolidateAttempted.clear();
    return null;
  }

  // Find the nearest untraveled edge via path length
  let bestPath: { nodeId: string; edgeId: string }[] | null = null;
  let bestEdgeId: string | null = null;

  for (const ue of allUntraveled) {
    const srcPath = graph.findPath(currentNodeId, ue.source_node);
    const tgtPath = graph.findPath(currentNodeId, ue.target_node);
    const path = srcPath && tgtPath
      ? (srcPath.length <= tgtPath.length ? srcPath : tgtPath)
      : srcPath ?? tgtPath;
    if (path && (!bestPath || path.length < bestPath.length)) {
      bestPath = path;
      bestEdgeId = ue.id;
    }
  }

  if (!bestPath || bestPath.length === 0) return null;

  if (bestEdgeId) state.consolidateAttempted.add(bestEdgeId);
  return {
    edge: graph.getEdge(bestPath[0].edgeId)!,
    reason: "consolidate_remote_untraveled",
  };
}

// ===========================================================================
// 4. Check Stale Areas — target low-confidence entities
// ===========================================================================

function chooseCheckStale(
  currentNodeId: string,
  graph: NetworkGraph,
): BehaviorResult | null {
  // Find nodes below Committed tier
  const staleNodes = graph.getAllNodes().filter(n => {
    if (n.id === currentNodeId) return false;
    return n.tier === ConfidenceTier.Candidate || n.tier === ConfidenceTier.Provisional;
  });

  if (staleNodes.length === 0) return null;

  // Sort by confidence (lowest first), break ties by edge count (junctions first)
  staleNodes.sort((a, b) => {
    if (a.confidence !== b.confidence) return a.confidence - b.confidence;
    return graph.getNodeEdges(b.id).length - graph.getNodeEdges(a.id).length;
  });

  // Path to the most stale node
  for (const target of staleNodes.slice(0, 10)) {
    const path = graph.findPath(currentNodeId, target.id);
    if (path && path.length > 0) {
      return {
        edge: graph.getEdge(path[0].edgeId)!,
        reason: "check_stale_node",
      };
    }
  }

  return null;
}

// ===========================================================================
// 5. Check Main Pathways — patrol high-traffic edges
// ===========================================================================

function chooseCheckMainPathways(
  state: BehaviorState,
  currentNodeId: string,
  graph: NetworkGraph,
): BehaviorResult | null {
  // Build patrol route if empty
  if (state.patrolRoute.length === 0) {
    const traveledEdges = graph.getAllEdges()
      .filter(e => e.traveled)
      .sort((a, b) => b.traveled_by.length - a.traveled_by.length);

    // Collect unique node IDs from high-traffic edges (junctions first)
    const nodeSet = new Set<string>();
    for (const e of traveledEdges) {
      const src = graph.getNode(e.source_node);
      const tgt = graph.getNode(e.target_node);
      if (src && graph.getNodeEdges(src.id).length >= 3) nodeSet.add(src.id);
      if (tgt && graph.getNodeEdges(tgt.id).length >= 3) nodeSet.add(tgt.id);
    }

    state.patrolRoute = Array.from(nodeSet);
    state.patrolIndex = 0;

    if (state.patrolRoute.length === 0) return null;
  }

  // Navigate to the next patrol node
  if (state.patrolIndex >= state.patrolRoute.length) {
    state.patrolIndex = 0;
  }

  const targetId = state.patrolRoute[state.patrolIndex];
  if (targetId === currentNodeId) {
    state.patrolIndex++;
    if (state.patrolIndex >= state.patrolRoute.length) state.patrolIndex = 0;
    return null;
  }

  const path = graph.findPath(currentNodeId, targetId);
  if (!path || path.length === 0) {
    state.patrolIndex++;
    return null;
  }

  state.patrolIndex++;
  return {
    edge: graph.getEdge(path[0].edgeId)!,
    reason: "patrol_main_pathway",
  };
}

// ===========================================================================
// 6. Find Boxes — look for rectangular occupancy clusters
// ===========================================================================

function chooseFindBoxes(
  currentNodeId: string,
  graph: NetworkGraph,
  pose: Pose2D,
): BehaviorResult | null {
  // Target unexplored nodes near obstacles (nodes with many short-range features).
  // In a warehouse, boxes are typically in aisles between shelving.
  // Go to areas with the most unexplored neighbors first.
  const candidates = graph.getAllNodes()
    .filter(n => n.id !== currentNodeId)
    .map(n => ({
      node: n,
      unexploredNeighbors: graph.getNeighbors(n.id).filter(nb => !nb.explored).length,
      dist: vec2Distance(pose, n.position),
    }))
    .filter(c => c.unexploredNeighbors > 0)
    .sort((a, b) => {
      if (b.unexploredNeighbors !== a.unexploredNeighbors) {
        return b.unexploredNeighbors - a.unexploredNeighbors;
      }
      return a.dist - b.dist;
    });

  for (const c of candidates.slice(0, 5)) {
    const path = graph.findPath(currentNodeId, c.node.id);
    if (path && path.length > 0) {
      return {
        edge: graph.getEdge(path[0].edgeId)!,
        reason: "find_boxes_area",
      };
    }
  }

  // Fallback: explore normally
  return graph.chooseExplorationEdge(currentNodeId);
}

// ===========================================================================
// 7. Idle — handled in dispatcher (returns null)
// ===========================================================================

// ===========================================================================
// 8. Return to Start — navigate back to first node
// ===========================================================================

function chooseReturnToStart(
  state: BehaviorState,
  currentNodeId: string,
  graph: NetworkGraph,
): BehaviorResult | null {
  if (!state.startingNodeId) return null;
  if (currentNodeId === state.startingNodeId) return null;

  const path = graph.findPath(currentNodeId, state.startingNodeId);
  if (!path || path.length === 0) return null;

  return {
    edge: graph.getEdge(path[0].edgeId)!,
    reason: "return_to_start",
  };
}
