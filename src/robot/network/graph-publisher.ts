// ===========================================================================
// Graph Publisher
// Publishes SLAM discoveries (nodes and edges) to the Gun.js network
// so other robots can see them. Each robot publishes its own discoveries.
// ===========================================================================

import type { NetworkNode, NetworkEdge } from "../../shared/types";

// ---------------------------------------------------------------------------
// Publisher Interface (abstracts Gun.js for testability)
// ---------------------------------------------------------------------------

export interface IGraphNetwork {
  publishNode(node: NetworkNode): void;
  publishEdge(edge: NetworkEdge): void;
  updateNode(nodeId: string, updates: Partial<NetworkNode>): void;
  updateEdge(edgeId: string, updates: Partial<NetworkEdge>): void;
}

// ---------------------------------------------------------------------------
// Graph Publisher
// ---------------------------------------------------------------------------

export class GraphPublisher {
  private readonly robotId: string;
  private network: IGraphNetwork | null = null;
  private publishedNodeIds: Set<string> = new Set();
  private publishedEdgeIds: Set<string> = new Set();
  private pendingNodes: NetworkNode[] = [];
  private pendingEdges: NetworkEdge[] = [];

  constructor(robotId: string) {
    this.robotId = robotId;
  }

  /**
   * Connect to the Gun.js network.
   */
  connect(network: IGraphNetwork): void {
    this.network = network;
    // Flush any pending publishes
    for (const node of this.pendingNodes) {
      this.network.publishNode(node);
      this.publishedNodeIds.add(node.id);
    }
    for (const edge of this.pendingEdges) {
      this.network.publishEdge(edge);
      this.publishedEdgeIds.add(edge.id);
    }
    this.pendingNodes = [];
    this.pendingEdges = [];
  }

  /**
   * Publish a newly discovered node.
   */
  publishNode(node: NetworkNode): void {
    if (this.publishedNodeIds.has(node.id)) return;

    if (this.network) {
      this.network.publishNode(node);
      this.publishedNodeIds.add(node.id);
    } else {
      this.pendingNodes.push(node);
    }
  }

  /**
   * Publish a newly discovered edge.
   */
  publishEdge(edge: NetworkEdge): void {
    if (this.publishedEdgeIds.has(edge.id)) return;

    if (this.network) {
      this.network.publishEdge(edge);
      this.publishedEdgeIds.add(edge.id);
    } else {
      this.pendingEdges.push(edge);
    }
  }

  /**
   * Update a node's confidence/position (e.g., after loop closure).
   */
  updateNode(nodeId: string, updates: Partial<NetworkNode>): void {
    if (this.network) {
      this.network.updateNode(nodeId, updates);
    }
  }

  /**
   * Update an edge (e.g., after correction).
   */
  updateEdge(edgeId: string, updates: Partial<NetworkEdge>): void {
    if (this.network) {
      this.network.updateEdge(edgeId, updates);
    }
  }

  /** Reset state */
  reset(): void {
    this.publishedNodeIds.clear();
    this.publishedEdgeIds.clear();
    this.pendingNodes = [];
    this.pendingEdges = [];
  }
}
