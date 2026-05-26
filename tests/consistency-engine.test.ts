import {
  ConsistencyEngine,
  ConsistencyGraphAdapter,
} from "../src/robot/consistency-engine";
import type { NetworkNode, NetworkEdge } from "../src/shared/types";
import {
  FeatureType,
  StabilityClass,
  ConfidenceTier,
  EdgeType,
  EdgeStatus,
  defaultNodeExtensions,
  defaultEdgeExtensions,
  defaultTraversability,
} from "../src/shared/types";

// ---------------------------------------------------------------------------
// Mock Graph Adapter
// ---------------------------------------------------------------------------

function makeNode(overrides: Partial<NetworkNode> = {}): NetworkNode {
  return {
    id: overrides.id ?? `node-${Math.random().toString(36).slice(2, 8)}`,
    type: overrides.type ?? FeatureType.Junction,
    position: overrides.position ?? { x: 0, y: 0 },
    confidence: overrides.confidence ?? 0.8,
    observation_count: overrides.observation_count ?? 1,
    exit_count: overrides.exit_count ?? 3,
    local_map_hash: overrides.local_map_hash ?? "",
    discovered_by: overrides.discovered_by ?? "robot-1",
    discovered_at: overrides.discovered_at ?? 1000,
    ...defaultNodeExtensions(),
    ...overrides,
  };
}

function makeEdge(overrides: Partial<NetworkEdge> = {}): NetworkEdge {
  return {
    id: overrides.id ?? `edge-${Math.random().toString(36).slice(2, 8)}`,
    source_node: overrides.source_node ?? "a",
    target_node: overrides.target_node ?? "b",
    distance: overrides.distance ?? 5.0,
    bearing_change: overrides.bearing_change ?? 0,
    bidirectional: overrides.bidirectional ?? true,
    confidence: overrides.confidence ?? 0.7,
    corridor_map_hash: overrides.corridor_map_hash ?? "",
    discovered_by: overrides.discovered_by ?? "robot-1",
    discovered_at: overrides.discovered_at ?? 1000,
    ...defaultEdgeExtensions(),
    ...overrides,
  };
}

class MockGraph implements ConsistencyGraphAdapter {
  private nodes: Map<string, NetworkNode> = new Map();
  private edges: NetworkEdge[] = [];

  addNode(node: NetworkNode): void {
    this.nodes.set(node.id, node);
  }

  addEdge(edge: NetworkEdge): void {
    this.edges.push(edge);
  }

  getAllNodes(): NetworkNode[] {
    return Array.from(this.nodes.values());
  }

  getAllEdges(): NetworkEdge[] {
    return [...this.edges];
  }

  getNodeEdges(nodeId: string): NetworkEdge[] {
    return this.edges.filter(
      (e) => e.source_node === nodeId || e.target_node === nodeId,
    );
  }

  getNeighbors(nodeId: string): NetworkNode[] {
    const neighborIds = new Set<string>();
    for (const edge of this.edges) {
      if (edge.source_node === nodeId) neighborIds.add(edge.target_node);
      if (edge.target_node === nodeId) neighborIds.add(edge.source_node);
    }
    const result: NetworkNode[] = [];
    for (const id of neighborIds) {
      const node = this.nodes.get(id);
      if (node) result.push(node);
    }
    return result;
  }

  getNode(nodeId: string): NetworkNode | null {
    return this.nodes.get(nodeId) ?? null;
  }
}

describe("ConsistencyEngine", () => {
  let engine: ConsistencyEngine;

  beforeEach(() => {
    engine = new ConsistencyEngine();
  });

  // -----------------------------------------------------------------------
  // checkDegreeConstraints
  // -----------------------------------------------------------------------

  describe("checkDegreeConstraints", () => {
    it("produces a violation for a Junction with <3 edges", () => {
      const graph = new MockGraph();
      const junction = makeNode({ id: "j1", type: FeatureType.Junction });
      const neighbor = makeNode({ id: "n1", type: FeatureType.Corner });
      graph.addNode(junction);
      graph.addNode(neighbor);

      // Only 1 edge — junction requires 3+
      graph.addEdge(makeEdge({ id: "e1", source_node: "j1", target_node: "n1" }));

      const violations = engine.checkDegreeConstraints(graph);
      expect(violations.length).toBeGreaterThan(0);

      const junctionViolation = violations.find((v) =>
        v.affected_entity_ids.includes("j1"),
      );
      expect(junctionViolation).toBeDefined();
      expect(junctionViolation!.check).toBe("degree");
    });

    it("produces no violation for a Junction with 3+ edges", () => {
      const graph = new MockGraph();
      const junction = makeNode({ id: "j1", type: FeatureType.Junction });
      const n1 = makeNode({ id: "n1", type: FeatureType.Corner });
      const n2 = makeNode({ id: "n2", type: FeatureType.Corner });
      const n3 = makeNode({ id: "n3", type: FeatureType.Corner });
      graph.addNode(junction);
      graph.addNode(n1);
      graph.addNode(n2);
      graph.addNode(n3);

      graph.addEdge(makeEdge({ id: "e1", source_node: "j1", target_node: "n1" }));
      graph.addEdge(makeEdge({ id: "e2", source_node: "j1", target_node: "n2" }));
      graph.addEdge(makeEdge({ id: "e3", source_node: "j1", target_node: "n3" }));

      const violations = engine.checkDegreeConstraints(graph);
      const junctionViolations = violations.filter((v) =>
        v.affected_entity_ids.includes("j1"),
      );
      expect(junctionViolations.length).toBe(0);
    });

    it("produces a violation for a DeadEnd with >1 edge", () => {
      const graph = new MockGraph();
      const deadEnd = makeNode({ id: "d1", type: FeatureType.DeadEnd });
      const n1 = makeNode({ id: "n1", type: FeatureType.Corner });
      const n2 = makeNode({ id: "n2", type: FeatureType.Corner });
      graph.addNode(deadEnd);
      graph.addNode(n1);
      graph.addNode(n2);

      graph.addEdge(makeEdge({ id: "e1", source_node: "d1", target_node: "n1" }));
      graph.addEdge(makeEdge({ id: "e2", source_node: "d1", target_node: "n2" }));

      const violations = engine.checkDegreeConstraints(graph);
      const deadEndViolation = violations.find((v) =>
        v.affected_entity_ids.includes("d1"),
      );
      expect(deadEndViolation).toBeDefined();
    });
  });

  // -----------------------------------------------------------------------
  // checkDistanceConsistency
  // -----------------------------------------------------------------------

  describe("checkDistanceConsistency", () => {
    it("produces no violation for a consistent triangle", () => {
      const graph = new MockGraph();
      // Triangle: A(0,0) - B(3,0) - C(0,4), distances: 3, 4, 5
      graph.addNode(makeNode({ id: "a", type: FeatureType.Corner, position: { x: 0, y: 0 } }));
      graph.addNode(makeNode({ id: "b", type: FeatureType.Corner, position: { x: 3, y: 0 } }));
      graph.addNode(makeNode({ id: "c", type: FeatureType.Corner, position: { x: 0, y: 4 } }));

      graph.addEdge(makeEdge({ id: "e-ab", source_node: "a", target_node: "b", distance: 3 }));
      graph.addEdge(makeEdge({ id: "e-bc", source_node: "b", target_node: "c", distance: 5 }));
      graph.addEdge(makeEdge({ id: "e-ac", source_node: "a", target_node: "c", distance: 4 }));

      const violations = engine.checkDistanceConsistency(graph);
      expect(violations.length).toBe(0);
    });

    it("produces a violation for an inconsistent triangle", () => {
      const graph = new MockGraph();
      graph.addNode(makeNode({ id: "a", type: FeatureType.Corner }));
      graph.addNode(makeNode({ id: "b", type: FeatureType.Corner }));
      graph.addNode(makeNode({ id: "c", type: FeatureType.Corner }));

      // d(A,C) = 100 is much greater than d(A,B) + d(B,C) = 3 + 4 = 7
      graph.addEdge(makeEdge({ id: "e-ab", source_node: "a", target_node: "b", distance: 3 }));
      graph.addEdge(makeEdge({ id: "e-bc", source_node: "b", target_node: "c", distance: 4 }));
      graph.addEdge(makeEdge({ id: "e-ac", source_node: "a", target_node: "c", distance: 100 }));

      const violations = engine.checkDistanceConsistency(graph);
      expect(violations.length).toBeGreaterThan(0);
      expect(violations[0].check).toBe("distance");
    });
  });

  // -----------------------------------------------------------------------
  // checkEdgeCrossings
  // -----------------------------------------------------------------------

  describe("checkEdgeCrossings", () => {
    it("produces no violation for non-crossing edges", () => {
      const graph = new MockGraph();
      // Two parallel horizontal edges that don't cross
      graph.addNode(makeNode({ id: "a", position: { x: 0, y: 0 } }));
      graph.addNode(makeNode({ id: "b", position: { x: 10, y: 0 } }));
      graph.addNode(makeNode({ id: "c", position: { x: 0, y: 5 } }));
      graph.addNode(makeNode({ id: "d", position: { x: 10, y: 5 } }));

      graph.addEdge(makeEdge({ id: "e1", source_node: "a", target_node: "b" }));
      graph.addEdge(makeEdge({ id: "e2", source_node: "c", target_node: "d" }));

      const violations = engine.checkEdgeCrossings(graph);
      expect(violations.length).toBe(0);
    });

    it("produces a violation for crossing edges", () => {
      const graph = new MockGraph();
      // X pattern: (0,0)-(10,10) crosses (10,0)-(0,10)
      graph.addNode(makeNode({ id: "a", position: { x: 0, y: 0 } }));
      graph.addNode(makeNode({ id: "b", position: { x: 10, y: 10 } }));
      graph.addNode(makeNode({ id: "c", position: { x: 10, y: 0 } }));
      graph.addNode(makeNode({ id: "d", position: { x: 0, y: 10 } }));

      graph.addEdge(makeEdge({ id: "e1", source_node: "a", target_node: "b" }));
      graph.addEdge(makeEdge({ id: "e2", source_node: "c", target_node: "d" }));

      const violations = engine.checkEdgeCrossings(graph);
      expect(violations.length).toBeGreaterThan(0);
      expect(violations[0].check).toBe("crossing");
    });
  });

  // -----------------------------------------------------------------------
  // runAllChecks
  // -----------------------------------------------------------------------

  describe("runAllChecks", () => {
    it("returns score 1.0 with no violations for an empty graph", () => {
      const graph = new MockGraph();
      const result = engine.runAllChecks(graph);

      expect(result.score).toBe(1.0);
      expect(result.violations.length).toBe(0);
    });
  });

  // -----------------------------------------------------------------------
  // Entity scores
  // -----------------------------------------------------------------------

  describe("entity scores", () => {
    it("node with violation has score < 1.0", () => {
      const graph = new MockGraph();
      const junction = makeNode({ id: "j1", type: FeatureType.Junction });
      const neighbor = makeNode({ id: "n1", type: FeatureType.Corner });
      graph.addNode(junction);
      graph.addNode(neighbor);

      // Only 1 edge — Junction needs 3+
      graph.addEdge(makeEdge({ id: "e1", source_node: "j1", target_node: "n1" }));

      const result = engine.runAllChecks(graph);
      const junctionScore = result.entity_scores.get("j1");
      expect(junctionScore).toBeDefined();
      expect(junctionScore!).toBeLessThan(1.0);
      expect(junctionScore!).toBeGreaterThanOrEqual(0.1); // floor
    });
  });
});
