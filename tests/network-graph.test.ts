import { NetworkGraph } from "../src/robot/network/network-graph";
import {
  FeatureType,
  ConfidenceTier,
  confidenceTierFromScore,
} from "../src/shared/types";

describe("NetworkGraph", () => {
  const ROBOT_ID = "robot-test";
  let graph: NetworkGraph;

  beforeEach(() => {
    graph = new NetworkGraph(ROBOT_ID);
  });

  // -----------------------------------------------------------------------
  // createNodeAtPosition
  // -----------------------------------------------------------------------

  describe("createNodeAtPosition", () => {
    it("returns a node with correct position and explored=false", () => {
      const node = graph.createNodeAtPosition(
        { x: 10, y: 20 },
        FeatureType.Junction,
        3,
        1000,
      );

      expect(node.position.x).toBe(10);
      expect(node.position.y).toBe(20);
      expect(node.type).toBe(FeatureType.Junction);
      expect(node.exit_count).toBe(3);
      expect(node.explored).toBe(false);
      expect(node.discovered_by).toBe(ROBOT_ID);
      expect(node.id).toBeTruthy();
    });
  });

  // -----------------------------------------------------------------------
  // markNodeExplored
  // -----------------------------------------------------------------------

  describe("markNodeExplored", () => {
    it("sets explored=true and adds robot to explored_by", () => {
      const node = graph.createNodeAtPosition(
        { x: 5, y: 5 },
        FeatureType.Corner,
        2,
        1000,
      );
      expect(node.explored).toBe(false);

      graph.markNodeExplored(node.id, ROBOT_ID, 2000);

      const updated = graph.getNode(node.id)!;
      expect(updated.explored).toBe(true);
      expect(updated.explored_by).toContain(ROBOT_ID);
      expect(updated.last_explored_at).toBe(2000);
    });

    it("does not duplicate robot in explored_by on second call", () => {
      const node = graph.createNodeAtPosition(
        { x: 5, y: 5 },
        FeatureType.Corner,
        2,
        1000,
      );
      graph.markNodeExplored(node.id, ROBOT_ID, 2000);
      graph.markNodeExplored(node.id, ROBOT_ID, 3000);

      const updated = graph.getNode(node.id)!;
      const robotCount = updated.explored_by.filter((r) => r === ROBOT_ID).length;
      expect(robotCount).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // createEdge
  // -----------------------------------------------------------------------

  describe("createEdge", () => {
    it("creates an untraveled edge with candidate tier when traveled=false", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);

      const edge = graph.createEdge(n1.id, n2.id, 10, 0, 1000, false);
      expect(edge).not.toBeNull();
      expect(edge!.traveled).toBe(false);
      expect(edge!.tier).toBe(ConfidenceTier.Candidate);
      expect(edge!.traveled_by).toEqual([]);
    });

    it("creates a traveled edge with provisional tier when traveled=true", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);

      const edge = graph.createEdge(n1.id, n2.id, 10, 0, 1000, true);
      expect(edge).not.toBeNull();
      expect(edge!.traveled).toBe(true);
      expect(edge!.tier).toBe(ConfidenceTier.Provisional);
      expect(edge!.traveled_by).toContain(ROBOT_ID);
    });

    it("returns null when source node does not exist", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const edge = graph.createEdge(n1.id, "nonexistent", 10, 0, 1000);
      expect(edge).toBeNull();
    });

    it("returns null for duplicate edge between same nodes", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);

      const first = graph.createEdge(n1.id, n2.id, 10, 0, 1000);
      const second = graph.createEdge(n1.id, n2.id, 10, 0, 2000);
      expect(first).not.toBeNull();
      expect(second).toBeNull();
    });
  });

  // -----------------------------------------------------------------------
  // markEdgeTraveled
  // -----------------------------------------------------------------------

  describe("markEdgeTraveled", () => {
    it("sets traveled=true and updates distance", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);
      const edge = graph.createEdge(n1.id, n2.id, 10, 0, 1000, false)!;

      graph.markEdgeTraveled(edge.id, ROBOT_ID, 9.5, 0.1, 2000);

      const updated = graph.getEdge(edge.id)!;
      expect(updated.traveled).toBe(true);
      expect(updated.distance).toBe(9.5);
      expect(updated.bearing_change).toBe(0.1);
      expect(updated.traveled_by).toContain(ROBOT_ID);
    });

    it("increases confidence when traveled", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);
      const edge = graph.createEdge(n1.id, n2.id, 10, 0, 1000, false)!;
      const originalConfidence = edge.confidence;

      graph.markEdgeTraveled(edge.id, ROBOT_ID, null, null, 2000);

      const updated = graph.getEdge(edge.id)!;
      expect(updated.confidence).toBeGreaterThan(originalConfidence);
    });
  });

  // -----------------------------------------------------------------------
  // getUnexploredNodes
  // -----------------------------------------------------------------------

  describe("getUnexploredNodes", () => {
    it("returns only unexplored nodes", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);
      const n3 = graph.createNodeAtPosition({ x: 20, y: 0 }, FeatureType.Corner, 2, 1000);

      graph.markNodeExplored(n1.id, ROBOT_ID, 2000);

      const unexplored = graph.getUnexploredNodes();
      const ids = unexplored.map((n) => n.id);
      expect(ids).not.toContain(n1.id);
      expect(ids).toContain(n2.id);
      expect(ids).toContain(n3.id);
    });
  });

  // -----------------------------------------------------------------------
  // getUntraveledEdgesForNode
  // -----------------------------------------------------------------------

  describe("getUntraveledEdgesForNode", () => {
    it("returns only untraveled edges", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);
      const n3 = graph.createNodeAtPosition({ x: 0, y: 10 }, FeatureType.Corner, 2, 1000);

      const e1 = graph.createEdge(n1.id, n2.id, 10, 0, 1000, true)!;  // traveled
      const e2 = graph.createEdge(n1.id, n3.id, 10, 0, 1000, false)!; // untraveled

      const untraveled = graph.getUntraveledEdgesForNode(n1.id);
      const ids = untraveled.map((e) => e.id);
      expect(ids).toContain(e2.id);
      expect(ids).not.toContain(e1.id);
    });
  });

  // -----------------------------------------------------------------------
  // chooseExplorationEdge
  // -----------------------------------------------------------------------

  describe("chooseExplorationEdge", () => {
    it("priority 1: picks edge to unexplored adjacent node", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);
      const n3 = graph.createNodeAtPosition({ x: 0, y: 10 }, FeatureType.Corner, 2, 1000);

      graph.markNodeExplored(n1.id, ROBOT_ID, 2000);
      // n2 explored, n3 unexplored
      graph.markNodeExplored(n2.id, ROBOT_ID, 2000);

      graph.createEdge(n1.id, n2.id, 10, 0, 1000, true);
      graph.createEdge(n1.id, n3.id, 10, 0, 1000, true);

      const choice = graph.chooseExplorationEdge(n1.id);
      expect(choice).not.toBeNull();
      expect(choice!.reason).toBe("unexplored_adjacent_node");
      // The edge should lead to n3 (the unexplored node)
      const otherNodeId =
        choice!.edge.source_node === n1.id
          ? choice!.edge.target_node
          : choice!.edge.source_node;
      expect(otherNodeId).toBe(n3.id);
    });

    it("priority 2: picks untraveled edge when all adjacent nodes are explored", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const n2 = graph.createNodeAtPosition({ x: 10, y: 0 }, FeatureType.Corner, 2, 1000);

      graph.markNodeExplored(n1.id, ROBOT_ID, 2000);
      graph.markNodeExplored(n2.id, ROBOT_ID, 2000);

      // untraveled edge between two explored nodes
      graph.createEdge(n1.id, n2.id, 10, 0, 1000, false);

      const choice = graph.chooseExplorationEdge(n1.id);
      expect(choice).not.toBeNull();
      expect(choice!.reason).toBe("untraveled_edge");
    });

    it("returns null when no edges exist", () => {
      const n1 = graph.createNodeAtPosition({ x: 0, y: 0 }, FeatureType.Corner, 2, 1000);
      const choice = graph.chooseExplorationEdge(n1.id);
      expect(choice).toBeNull();
    });
  });

  // -----------------------------------------------------------------------
  // strengthenNode
  // -----------------------------------------------------------------------

  describe("strengthenNode", () => {
    it("uses diminishing returns formula (1 - e^(-0.25*n))", () => {
      const node = graph.createNodeAtPosition(
        { x: 5, y: 5 },
        FeatureType.Corner,
        2,
        1000,
      );

      // observation_count starts at 1. After strengthenNode it becomes 2.
      graph.strengthenNode(node.id);
      const after1 = graph.getNode(node.id)!;
      expect(after1.observation_count).toBe(2);

      const expectedConf = 1 - Math.exp(-0.25 * 2);
      expect(after1.confidence).toBeCloseTo(expectedConf, 5);

      // Capture confidence before second strengthen (getNode returns same ref)
      const confidenceAfter1 = after1.confidence;

      // Strengthen again: observation_count becomes 3
      graph.strengthenNode(node.id);
      const after2 = graph.getNode(node.id)!;
      expect(after2.observation_count).toBe(3);

      const expectedConf2 = 1 - Math.exp(-0.25 * 3);
      expect(after2.confidence).toBeCloseTo(expectedConf2, 5);

      // Confidence should be monotonically increasing with diminishing returns
      expect(after2.confidence).toBeGreaterThan(confidenceAfter1);
    });

    it("updates the tier to match the new confidence score", () => {
      const node = graph.createNodeAtPosition(
        { x: 5, y: 5 },
        FeatureType.Corner,
        2,
        1000,
      );

      // Strengthen many times to push confidence high
      for (let i = 0; i < 15; i++) {
        graph.strengthenNode(node.id);
      }

      const updated = graph.getNode(node.id)!;
      expect(updated.tier).toBe(confidenceTierFromScore(updated.confidence));
    });
  });
});
