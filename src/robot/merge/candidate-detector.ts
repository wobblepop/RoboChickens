// ===========================================================================
// Candidate Detector — Merge Phases 1-2
// Discovers and evaluates potential node matches between two independently
// built network graphs. Phase 1 generates lightweight summaries suitable
// for broadcast; Phase 2 compares summaries across graphs and returns
// scored candidate match pairs.
// ===========================================================================

import type { Vec2, NetworkNode, NetworkEdge } from "../../shared/types";
import { FeatureType } from "../../shared/types";
import { vec2Distance, clamp } from "../../shared/math";

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

export interface NodeSummary {
  id: string;
  type: FeatureType;
  position: Vec2;
  confidence: number;
  edge_count: number;
  local_map_hash: string;
}

export interface CandidateMatch {
  node_a_id: string;
  node_b_id: string;
  geometry_similarity: number;
  feature_similarity: number;
  topology_similarity: number;
  composite_score: number;
}

// ---------------------------------------------------------------------------
// Default Options
// ---------------------------------------------------------------------------

interface CandidateDetectorOptions {
  compositeThreshold: number;
  geometryFloor: number;
  featureFloor: number;
  topologyFloor: number;
}

const DEFAULT_OPTIONS: CandidateDetectorOptions = {
  compositeThreshold: 0.70,
  geometryFloor: 0.6,
  featureFloor: 0.4,
  topologyFloor: 0.5,
};

// ---------------------------------------------------------------------------
// Composite Weights
// ---------------------------------------------------------------------------

const GEOMETRY_WEIGHT = 0.4;
const FEATURE_WEIGHT = 0.3;
const TOPOLOGY_WEIGHT = 0.3;

// Maximum position distance (meters) for any geometry similarity
const MAX_POSITION_DISTANCE = 5.0;

// ---------------------------------------------------------------------------
// Candidate Detector
// ---------------------------------------------------------------------------

export class CandidateDetector {
  private readonly options: CandidateDetectorOptions;

  constructor(options?: Partial<CandidateDetectorOptions>) {
    this.options = { ...DEFAULT_OPTIONS, ...options };

    if (this.options.compositeThreshold < 0 || this.options.compositeThreshold > 1) {
      throw new Error("compositeThreshold must be in [0, 1]");
    }
    if (this.options.geometryFloor < 0 || this.options.geometryFloor > 1) {
      throw new Error("geometryFloor must be in [0, 1]");
    }
    if (this.options.featureFloor < 0 || this.options.featureFloor > 1) {
      throw new Error("featureFloor must be in [0, 1]");
    }
    if (this.options.topologyFloor < 0 || this.options.topologyFloor > 1) {
      throw new Error("topologyFloor must be in [0, 1]");
    }
  }

  // =========================================================================
  // Phase 1: Generate Summaries
  // =========================================================================

  /**
   * Generate lightweight summaries of each node for broadcasting to peers.
   * Each summary includes the information needed for remote candidate matching
   * without transmitting the full graph.
   */
  generateSummaries(nodes: NetworkNode[], edges: NetworkEdge[]): NodeSummary[] {
    if (!Array.isArray(nodes)) {
      throw new Error("generateSummaries: nodes must be an array");
    }
    if (!Array.isArray(edges)) {
      throw new Error("generateSummaries: edges must be an array");
    }

    // Pre-compute edge counts per node
    const edgeCounts = new Map<string, number>();
    for (const edge of edges) {
      edgeCounts.set(edge.source_node, (edgeCounts.get(edge.source_node) ?? 0) + 1);
      edgeCounts.set(edge.target_node, (edgeCounts.get(edge.target_node) ?? 0) + 1);
    }

    const summaries: NodeSummary[] = [];
    for (const node of nodes) {
      summaries.push({
        id: node.id,
        type: node.type,
        position: { x: node.position.x, y: node.position.y },
        confidence: node.confidence,
        edge_count: edgeCounts.get(node.id) ?? 0,
        local_map_hash: node.local_map_hash,
      });
    }

    return summaries;
  }

  // =========================================================================
  // Phase 2: Find Candidates
  // =========================================================================

  /**
   * Compare node summaries from two graphs and return scored candidate matches.
   * Each pair is evaluated on geometry, feature, and topology similarity.
   * Only pairs meeting all per-dimension floors and the composite threshold
   * are returned.
   */
  findCandidates(summariesA: NodeSummary[], summariesB: NodeSummary[]): CandidateMatch[] {
    if (!Array.isArray(summariesA)) {
      throw new Error("findCandidates: summariesA must be an array");
    }
    if (!Array.isArray(summariesB)) {
      throw new Error("findCandidates: summariesB must be an array");
    }

    const candidates: CandidateMatch[] = [];

    for (const a of summariesA) {
      for (const b of summariesB) {
        const geometry = this.computeGeometrySimilarity(a, b);
        const feature = this.computeFeatureSimilarity(a, b);
        const topology = this.computeTopologySimilarity(a, b);

        // Per-dimension floor check (0 scores are exempt — they indicate
        // the dimension is inapplicable, not that it failed)
        if (geometry !== 0 && geometry < this.options.geometryFloor) continue;
        if (feature !== 0 && feature < this.options.featureFloor) continue;
        if (topology !== 0 && topology < this.options.topologyFloor) continue;

        const composite =
          GEOMETRY_WEIGHT * geometry +
          FEATURE_WEIGHT * feature +
          TOPOLOGY_WEIGHT * topology;

        if (composite >= this.options.compositeThreshold) {
          candidates.push({
            node_a_id: a.id,
            node_b_id: b.id,
            geometry_similarity: geometry,
            feature_similarity: feature,
            topology_similarity: topology,
            composite_score: composite,
          });
        }
      }
    }

    // Sort by composite score descending for convenience
    candidates.sort((a, b) => b.composite_score - a.composite_score);
    return candidates;
  }

  // =========================================================================
  // Per-Dimension Similarity Computations
  // =========================================================================

  /**
   * Geometry similarity: combines position proximity and edge-count proximity.
   * Positions beyond MAX_POSITION_DISTANCE yield 0.
   */
  private computeGeometrySimilarity(a: NodeSummary, b: NodeSummary): number {
    const dist = vec2Distance(a.position, b.position);
    if (dist > MAX_POSITION_DISTANCE) return 0;

    // Position component: linear decay from 1 at dist=0 to 0 at dist=5m
    const positionScore = 1.0 - dist / MAX_POSITION_DISTANCE;

    // Edge-count component: ratio of smaller to larger (identical = 1.0)
    const maxEdges = Math.max(a.edge_count, b.edge_count);
    const edgeCountScore = maxEdges === 0
      ? 1.0
      : Math.min(a.edge_count, b.edge_count) / maxEdges;

    // Blend: position is more diagnostic than edge count
    return clamp(0.7 * positionScore + 0.3 * edgeCountScore, 0, 1);
  }

  /**
   * Feature similarity: combines local_map_hash comparison and feature type match.
   * An exact hash match is a strong signal; type match is weaker but still useful.
   */
  private computeFeatureSimilarity(a: NodeSummary, b: NodeSummary): number {
    // Hash comparison: exact match = 1.0, mismatch = 0.0
    const hashMatch =
      a.local_map_hash.length > 0 &&
      b.local_map_hash.length > 0 &&
      a.local_map_hash === b.local_map_hash
        ? 1.0
        : 0.0;

    // Type comparison: exact match = 1.0, mismatch = 0.0
    const typeMatch = a.type === b.type ? 1.0 : 0.0;

    // If both hashes are empty/missing, rely entirely on type
    if (a.local_map_hash.length === 0 || b.local_map_hash.length === 0) {
      return typeMatch;
    }

    return clamp(0.6 * hashMatch + 0.4 * typeMatch, 0, 1);
  }

  /**
   * Topology similarity: based on edge-count ratio.
   * Nodes with the same connectivity pattern are more likely to be the same place.
   */
  private computeTopologySimilarity(a: NodeSummary, b: NodeSummary): number {
    const maxEdges = Math.max(a.edge_count, b.edge_count);
    if (maxEdges === 0) {
      // Both isolated nodes — topologically identical but uninformative
      return 1.0;
    }
    return Math.min(a.edge_count, b.edge_count) / maxEdges;
  }
}
