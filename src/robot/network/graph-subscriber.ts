// ===========================================================================
// Graph Subscriber
// Listens to the Gun.js network for nodes and edges published by other robots.
// Integrates external discoveries into this robot's local network graph.
// ===========================================================================

import type { NetworkNode, NetworkEdge, Vec2 } from "../../shared/types";
import { vec2Distance } from "../../shared/math";
import type { NetworkGraph } from "./network-graph";

// ---------------------------------------------------------------------------
// Subscriber Interface
// ---------------------------------------------------------------------------

export interface IGraphNetworkSubscriber {
  onNodeReceived(callback: (node: NetworkNode) => void): () => void;
  onEdgeReceived(callback: (edge: NetworkEdge) => void): () => void;
  onNodeUpdated(callback: (nodeId: string, updates: Partial<NetworkNode>) => void): () => void;
  onEdgeUpdated(callback: (edgeId: string, updates: Partial<NetworkEdge>) => void): () => void;
}

// ---------------------------------------------------------------------------
// Graph Subscriber
// ---------------------------------------------------------------------------

export class GraphSubscriber {
  private readonly robotId: string;
  private readonly graph: NetworkGraph;
  private unsubscribes: (() => void)[] = [];

  /** Coordinate offset applied to external nodes to align with our SLAM frame */
  private coordinateOffset: Vec2 = { x: 0, y: 0 };
  private coordinateRotation: number = 0;
  private alignmentEstablished: boolean = false;

  constructor(robotId: string, graph: NetworkGraph) {
    this.robotId = robotId;
    this.graph = graph;
  }

  /**
   * Connect to the Gun.js network and start receiving updates.
   */
  connect(network: IGraphNetworkSubscriber): void {
    this.disconnect(); // clean up any previous subscriptions

    this.unsubscribes.push(
      network.onNodeReceived((node) => this.handleNodeReceived(node)),
    );
    this.unsubscribes.push(
      network.onEdgeReceived((edge) => this.handleEdgeReceived(edge)),
    );
    this.unsubscribes.push(
      network.onNodeUpdated((id, updates) => this.handleNodeUpdated(id, updates)),
    );
    this.unsubscribes.push(
      network.onEdgeUpdated((id, updates) => this.handleEdgeUpdated(id, updates)),
    );
  }

  /**
   * Disconnect from the network.
   */
  disconnect(): void {
    for (const unsub of this.unsubscribes) unsub();
    this.unsubscribes = [];
  }

  /**
   * Set the coordinate alignment between this robot's SLAM frame and the
   * publishing robot's SLAM frame. This is established when two robots
   * discover they're mapping the same area (merge detection).
   */
  setCoordinateAlignment(offset: Vec2, rotation: number): void {
    this.coordinateOffset = offset;
    this.coordinateRotation = rotation;
    this.alignmentEstablished = true;
  }

  /** Reset state */
  reset(): void {
    this.disconnect();
    this.coordinateOffset = { x: 0, y: 0 };
    this.coordinateRotation = 0;
    this.alignmentEstablished = false;
    this.pendingExternalNodes = [];
    this.pendingExternalEdges = [];
    this.pendingNodeIds.clear();
    this.pendingEdgeIds.clear();
  }

  // -------------------------------------------------------------------------
  // Private Handlers
  // -------------------------------------------------------------------------

  /** Buffer of external nodes waiting for alignment to be established */
  private pendingExternalNodes: NetworkNode[] = [];
  private pendingExternalEdges: NetworkEdge[] = [];
  private pendingNodeIds: Set<string> = new Set();
  private pendingEdgeIds: Set<string> = new Set();

  private handleNodeReceived(node: NetworkNode): void {
    if (node.discovered_by === this.robotId) return;
    if (this.graph.getNode(node.id)) return;

    // Don't add external nodes until coordinate alignment is established.
    // Without alignment, external nodes are in the wrong frame and would
    // create garbage edges through walls.
    if (!this.alignmentEstablished) {
      // Buffer for later processing once alignment is computed
      if (!this.pendingNodeIds.has(node.id)) {
        this.pendingNodeIds.add(node.id);
        this.pendingExternalNodes.push(node);
      }
      return;
    }

    this.integrateExternalNode(node);
  }

  private integrateExternalNode(node: NetworkNode): void {
    const transformedNode = this.transformNode(node);

    const nearby = this.graph.findNodesInRadius(transformedNode.position, 2.0);
    if (nearby.length > 0) {
      let closestId = nearby[0].id;
      let closestDist = vec2Distance(transformedNode.position, nearby[0].position);
      for (let i = 1; i < nearby.length; i++) {
        const d = vec2Distance(transformedNode.position, nearby[i].position);
        if (d < closestDist) {
          closestDist = d;
          closestId = nearby[i].id;
        }
      }
      this.graph.strengthenNode(closestId);
      return;
    }

    this.graph.addExternalNode(transformedNode);
  }

  /**
   * Called after alignment is established to flush buffered external nodes.
   */
  flushPendingNodes(): void {
    for (const node of this.pendingExternalNodes) {
      if (!this.graph.getNode(node.id)) {
        this.integrateExternalNode(node);
      }
    }
    this.pendingExternalNodes = [];
    this.pendingNodeIds.clear();

    for (const edge of this.pendingExternalEdges) {
      if (!this.graph.getEdge(edge.id)) {
        this.handleEdgeReceived(edge);
      }
    }
    this.pendingExternalEdges = [];
    this.pendingEdgeIds.clear();
  }

  private handleEdgeReceived(edge: NetworkEdge): void {
    if (edge.discovered_by === this.robotId) return;
    if (this.graph.getEdge(edge.id)) return;

    if (!this.alignmentEstablished) {
      if (!this.pendingEdgeIds.has(edge.id)) {
        this.pendingEdgeIds.add(edge.id);
        this.pendingExternalEdges.push(edge);
      }
      return;
    }

    if (!this.graph.getNode(edge.source_node)) return;
    if (!this.graph.getNode(edge.target_node)) return;

    this.graph.addExternalEdge(edge);
  }

  private handleNodeUpdated(nodeId: string, updates: Partial<NetworkNode>): void {
    const node = this.graph.getNode(nodeId);
    if (!node) return;

    // Apply updates
    if (updates.confidence !== undefined) node.confidence = updates.confidence;
    if (updates.observation_count !== undefined) node.observation_count = updates.observation_count;
    if (updates.position !== undefined && this.alignmentEstablished) {
      node.position = this.transformPosition(updates.position);
    }
  }

  private handleEdgeUpdated(edgeId: string, updates: Partial<NetworkEdge>): void {
    const edge = this.graph.getEdge(edgeId);
    if (!edge) return;

    if (updates.confidence !== undefined) edge.confidence = updates.confidence;
    if (updates.distance !== undefined) edge.distance = updates.distance;
  }

  // -------------------------------------------------------------------------
  // Coordinate Transformation
  // -------------------------------------------------------------------------

  private transformNode(node: NetworkNode): NetworkNode {
    return {
      ...node,
      position: this.transformPosition(node.position),
    };
  }

  private transformPosition(pos: Vec2): Vec2 {
    if (!this.alignmentEstablished) return pos;

    const c = Math.cos(this.coordinateRotation);
    const s = Math.sin(this.coordinateRotation);
    return {
      x: pos.x * c - pos.y * s + this.coordinateOffset.x,
      y: pos.x * s + pos.y * c + this.coordinateOffset.y,
    };
  }
}
