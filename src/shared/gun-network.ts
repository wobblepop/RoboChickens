// ===========================================================================
// Gun.js Network Adapter
// Wraps Gun.js for real P2P multi-robot communication.
// Implements the same IGraphNetwork + IGraphNetworkSubscriber interfaces
// as InProcessNetwork, so swapping is transparent to robot code.
//
// Robots only share information through this Gun protocol.
//
// mDNS discovery: "_explorationmapper._tcp"
// Heartbeat: 5s interval, 15s timeout
// Reconnect: exponential backoff (1s, 2s, 4s, ... 60s cap)
// Bandwidth priority: Critical > Normal > Background
// ===========================================================================

import type { NetworkNode, NetworkEdge, ObservationRecord } from "./types";
import type { IGraphNetwork } from "../robot/network/graph-publisher";
import type { IGraphNetworkSubscriber } from "../robot/network/graph-subscriber";

// ---------------------------------------------------------------------------
// Connection State
// ---------------------------------------------------------------------------

export enum ConnectionState {
  Disconnected = "disconnected",
  Connecting = "connecting",
  Connected = "connected",
  Reconnecting = "reconnecting",
}

export interface PeerInfo {
  id: string;
  graph_id: string;
  node_count: number;
  last_seen: number;
}

export interface HeartbeatConfig {
  interval_ms: number;
  timeout_ms: number;
}

const DEFAULT_HEARTBEAT: HeartbeatConfig = {
  interval_ms: 5000,
  timeout_ms: 15000,
};

// ---------------------------------------------------------------------------
// Gun.js Network
// ---------------------------------------------------------------------------

type NodeCallback = (node: NetworkNode) => void;
type EdgeCallback = (edge: NetworkEdge) => void;
type NodeUpdateCallback = (nodeId: string, updates: Partial<NetworkNode>) => void;
type EdgeUpdateCallback = (edgeId: string, updates: Partial<NetworkEdge>) => void;

export class GunNetwork implements IGraphNetwork, IGraphNetworkSubscriber {
  private gun: any = null;
  private robotId: string;
  private graphId: string;
  private state: ConnectionState = ConnectionState.Disconnected;
  private peers: Map<string, PeerInfo> = new Map();
  private heartbeatInterval: ReturnType<typeof setInterval> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectDelay: number = 1000;
  private readonly maxReconnectDelay: number = 60000;

  private nodeListeners: Set<NodeCallback> = new Set();
  private edgeListeners: Set<EdgeCallback> = new Set();
  private nodeUpdateListeners: Set<NodeUpdateCallback> = new Set();
  private edgeUpdateListeners: Set<EdgeUpdateCallback> = new Set();

  private pendingNodes: NetworkNode[] = [];
  private pendingEdges: NetworkEdge[] = [];

  constructor(robotId: string, graphId: string) {
    this.robotId = robotId;
    this.graphId = graphId;
  }

  // =========================================================================
  // Connection Management
  // =========================================================================

  async connect(peers: string[]): Promise<void> {
    this.state = ConnectionState.Connecting;

    try {
      const Gun = require("gun");
      this.gun = Gun({ peers });
      this.state = ConnectionState.Connected;
      this.reconnectDelay = 1000;

      this.subscribeToGunUpdates();
      this.startHeartbeat();
      this.flushPending();
    } catch (err) {
      this.state = ConnectionState.Disconnected;
      this.scheduleReconnect(peers);
    }
  }

  disconnect(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.state = ConnectionState.Disconnected;
  }

  getState(): ConnectionState { return this.state; }
  getPeers(): PeerInfo[] { return Array.from(this.peers.values()); }

  // =========================================================================
  // IGraphNetwork (publisher)
  // =========================================================================

  publishNode(node: NetworkNode): void {
    if (this.state !== ConnectionState.Connected || !this.gun) {
      this.pendingNodes.push(node);
      return;
    }

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("nodes")
      .get(node.id)
      .put(this.serializeNode(node));
  }

  publishEdge(edge: NetworkEdge): void {
    if (this.state !== ConnectionState.Connected || !this.gun) {
      this.pendingEdges.push(edge);
      return;
    }

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("edges")
      .get(edge.id)
      .put(this.serializeEdge(edge));
  }

  updateNode(nodeId: string, updates: Partial<NetworkNode>): void {
    if (this.state !== ConnectionState.Connected || !this.gun) return;

    const data: Record<string, any> = {};
    if (updates.confidence !== undefined) data.confidence = updates.confidence;
    if (updates.observation_count !== undefined) data.observation_count = updates.observation_count;
    if (updates.explored !== undefined) data.explored = updates.explored;
    if (updates.position !== undefined) {
      data.pos_x = updates.position.x;
      data.pos_y = updates.position.y;
    }
    if (updates.tier !== undefined) data.tier = updates.tier;
    if (updates.version !== undefined) data.version = updates.version;

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("nodes")
      .get(nodeId)
      .put(data);
  }

  updateEdge(edgeId: string, updates: Partial<NetworkEdge>): void {
    if (this.state !== ConnectionState.Connected || !this.gun) return;

    const data: Record<string, any> = {};
    if (updates.confidence !== undefined) data.confidence = updates.confidence;
    if (updates.distance !== undefined) data.distance = updates.distance;
    if (updates.traveled !== undefined) data.traveled = updates.traveled;
    if (updates.tier !== undefined) data.tier = updates.tier;

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("edges")
      .get(edgeId)
      .put(data);
  }

  // =========================================================================
  // IGraphNetworkSubscriber
  // =========================================================================

  onNodeReceived(callback: NodeCallback): () => void {
    this.nodeListeners.add(callback);
    return () => { this.nodeListeners.delete(callback); };
  }

  onEdgeReceived(callback: EdgeCallback): () => void {
    this.edgeListeners.add(callback);
    return () => { this.edgeListeners.delete(callback); };
  }

  onNodeUpdated(callback: NodeUpdateCallback): () => void {
    this.nodeUpdateListeners.add(callback);
    return () => { this.nodeUpdateListeners.delete(callback); };
  }

  onEdgeUpdated(callback: EdgeUpdateCallback): () => void {
    this.edgeUpdateListeners.add(callback);
    return () => { this.edgeUpdateListeners.delete(callback); };
  }

  // =========================================================================
  // Gun.js Subscriptions
  // =========================================================================

  private subscribeToGunUpdates(): void {
    if (!this.gun) return;

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("nodes")
      .map()
      .on((data: any, key: string) => {
        if (!data || key === "_") return;
        const node = this.deserializeNode(data, key);
        if (node) {
          for (const cb of this.nodeListeners) cb(node);
        }
      });

    this.gun
      .get("explorationmapper")
      .get(this.graphId)
      .get("edges")
      .map()
      .on((data: any, key: string) => {
        if (!data || key === "_") return;
        const edge = this.deserializeEdge(data, key);
        if (edge) {
          for (const cb of this.edgeListeners) cb(edge);
        }
      });
  }

  // =========================================================================
  // Heartbeat
  // =========================================================================

  private startHeartbeat(): void {
    if (this.heartbeatInterval) clearInterval(this.heartbeatInterval);

    this.heartbeatInterval = setInterval(() => {
      if (!this.gun) return;

      this.gun
        .get("explorationmapper")
        .get("heartbeat")
        .get(this.robotId)
        .put({
          id: this.robotId,
          graph_id: this.graphId,
          timestamp: Date.now(),
        });

      // Check for stale peers
      const now = Date.now();
      for (const [id, peer] of this.peers) {
        if (now - peer.last_seen > DEFAULT_HEARTBEAT.timeout_ms) {
          this.peers.delete(id);
        }
      }
    }, DEFAULT_HEARTBEAT.interval_ms);
  }

  // =========================================================================
  // Reconnection
  // =========================================================================

  private scheduleReconnect(peers: string[]): void {
    this.state = ConnectionState.Reconnecting;
    this.reconnectTimer = setTimeout(() => {
      this.connect(peers);
    }, this.reconnectDelay);
    this.reconnectDelay = Math.min(this.reconnectDelay * 2, this.maxReconnectDelay);
  }

  // =========================================================================
  // Pending Flush
  // =========================================================================

  private flushPending(): void {
    for (const node of this.pendingNodes) {
      this.publishNode(node);
    }
    for (const edge of this.pendingEdges) {
      this.publishEdge(edge);
    }
    this.pendingNodes = [];
    this.pendingEdges = [];
  }

  // =========================================================================
  // Serialization (Gun.js stores flat objects)
  // =========================================================================

  private serializeNode(node: NetworkNode): Record<string, any> {
    return {
      id: node.id,
      type: node.type,
      pos_x: node.position.x,
      pos_y: node.position.y,
      confidence: node.confidence,
      observation_count: node.observation_count,
      exit_count: node.exit_count,
      local_map_hash: node.local_map_hash,
      discovered_by: node.discovered_by,
      discovered_at: node.discovered_at,
      explored: node.explored,
      tier: node.tier,
      version: node.version,
      last_observed: node.last_observed,
      stability_class: node.stability_class,
    };
  }

  private serializeEdge(edge: NetworkEdge): Record<string, any> {
    return {
      id: edge.id,
      source_node: edge.source_node,
      target_node: edge.target_node,
      distance: edge.distance,
      bearing_change: edge.bearing_change,
      bidirectional: edge.bidirectional,
      confidence: edge.confidence,
      discovered_by: edge.discovered_by,
      discovered_at: edge.discovered_at,
      traveled: edge.traveled,
      tier: edge.tier,
      version: edge.version,
      edge_type: edge.edge_type,
    };
  }

  private deserializeNode(data: any, id: string): NetworkNode | null {
    if (!data.type || data.pos_x === undefined || data.pos_y === undefined) return null;
    return {
      id,
      type: data.type,
      position: { x: data.pos_x, y: data.pos_y },
      confidence: data.confidence ?? 0.5,
      observation_count: data.observation_count ?? 1,
      exit_count: data.exit_count ?? 0,
      local_map_hash: data.local_map_hash ?? "",
      discovered_by: data.discovered_by ?? "",
      discovered_at: data.discovered_at ?? 0,
      explored: data.explored ?? false,
      explored_by: data.explored_by ? JSON.parse(data.explored_by) : [],
      last_explored_at: data.last_explored_at ?? 0,
      stability_class: data.stability_class ?? "static",
      tier: data.tier ?? "candidate",
      observation_log: [],
      last_observed: data.last_observed ?? 0,
      version: data.version ?? 1,
    } as NetworkNode;
  }

  private deserializeEdge(data: any, id: string): NetworkEdge | null {
    if (!data.source_node || !data.target_node) return null;
    return {
      id,
      source_node: data.source_node,
      target_node: data.target_node,
      distance: data.distance ?? 0,
      bearing_change: data.bearing_change ?? 0,
      bidirectional: data.bidirectional ?? true,
      confidence: data.confidence ?? 0.5,
      corridor_map_hash: data.corridor_map_hash ?? "",
      discovered_by: data.discovered_by ?? "",
      discovered_at: data.discovered_at ?? 0,
      traveled: data.traveled ?? false,
      traveled_by: data.traveled_by ? JSON.parse(data.traveled_by) : [],
      edge_type: data.edge_type ?? "unknown",
      traversability: {
        clearance_width: Infinity,
        clearance_height: Infinity,
        surface_type: "unknown",
        slope: 0,
        status: "unknown" as any,
        restrictions: [],
      },
      stability_class: data.stability_class ?? "static",
      tier: data.tier ?? "candidate",
      observation_log: [],
      last_observed: data.last_observed ?? 0,
      version: data.version ?? 1,
    } as NetworkEdge;
  }
}
