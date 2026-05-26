// ===========================================================================
// Gun.js Graph Schema & In-Process Network
//
// CRDT semantics: observation-level merge via set union.
// The network layer NEVER resolves spatial conflicts directly.
// It only merges observation sets. Derived values (position, confidence,
// geometry) are computed locally by each peer from the merged observation log.
//
// For simulation, InProcessNetwork replaces Gun.js with a shared event bus.
// The interface is identical, so swapping to real Gun.js is transparent.
// ===========================================================================

import type { NetworkNode, NetworkEdge, ObservationRecord } from "./types";
import type { IGraphNetwork } from "../robot/network/graph-publisher";
import type { IGraphNetworkSubscriber } from "../robot/network/graph-subscriber";

// ---------------------------------------------------------------------------
// Observation Store (CRDT Set — append-only, merge = union)
// ---------------------------------------------------------------------------

export class ObservationStore {
  private observations: Map<string, ObservationRecord> = new Map();

  add(observation: ObservationRecord): boolean {
    if (this.observations.has(observation.id)) return false;
    this.observations.set(observation.id, observation);
    return true;
  }

  merge(other: ObservationStore): ObservationRecord[] {
    const newObs: ObservationRecord[] = [];
    for (const [id, obs] of other.observations) {
      if (!this.observations.has(id)) {
        this.observations.set(id, obs);
        newObs.push(obs);
      }
    }
    return newObs;
  }

  getAll(): ObservationRecord[] {
    return Array.from(this.observations.values());
  }

  get size(): number { return this.observations.size; }

  clear(): void { this.observations.clear(); }
}

// ---------------------------------------------------------------------------
// In-Process Shared Network (CRDT-aware)
// ---------------------------------------------------------------------------

type NodeCallback = (node: NetworkNode) => void;
type EdgeCallback = (edge: NetworkEdge) => void;
type NodeUpdateCallback = (nodeId: string, updates: Partial<NetworkNode>) => void;
type EdgeUpdateCallback = (edgeId: string, updates: Partial<NetworkEdge>) => void;
type ObservationCallback = (entityId: string, observation: ObservationRecord) => void;

export class InProcessNetwork implements IGraphNetwork, IGraphNetworkSubscriber {
  private nodes: Map<string, NetworkNode> = new Map();
  private edges: Map<string, NetworkEdge> = new Map();

  private entityObservations: Map<string, ObservationStore> = new Map();

  private nodeListeners: Set<NodeCallback> = new Set();
  private edgeListeners: Set<EdgeCallback> = new Set();
  private nodeUpdateListeners: Set<NodeUpdateCallback> = new Set();
  private edgeUpdateListeners: Set<EdgeUpdateCallback> = new Set();
  private observationListeners: Set<ObservationCallback> = new Set();

  // --- IGraphNetwork (publisher interface) ---

  publishNode(node: NetworkNode): void {
    this.nodes.set(node.id, { ...node });
    for (const cb of this.nodeListeners) {
      cb(node);
    }
  }

  publishEdge(edge: NetworkEdge): void {
    this.edges.set(edge.id, { ...edge });
    for (const cb of this.edgeListeners) {
      cb(edge);
    }
  }

  updateNode(nodeId: string, updates: Partial<NetworkNode>): void {
    const existing = this.nodes.get(nodeId);
    if (existing) {
      if (updates.confidence !== undefined) existing.confidence = updates.confidence;
      if (updates.observation_count !== undefined) existing.observation_count = updates.observation_count;
      if (updates.position !== undefined) existing.position = updates.position;
      if (updates.explored !== undefined) existing.explored = updates.explored;
      if (updates.tier !== undefined) existing.tier = updates.tier;
      if (updates.version !== undefined) existing.version = updates.version;
      if (updates.last_observed !== undefined) existing.last_observed = updates.last_observed;
    }
    for (const cb of this.nodeUpdateListeners) {
      cb(nodeId, updates);
    }
  }

  updateEdge(edgeId: string, updates: Partial<NetworkEdge>): void {
    const existing = this.edges.get(edgeId);
    if (existing) {
      if (updates.confidence !== undefined) existing.confidence = updates.confidence;
      if (updates.distance !== undefined) existing.distance = updates.distance;
      if (updates.traveled !== undefined) existing.traveled = updates.traveled;
      if (updates.tier !== undefined) existing.tier = updates.tier;
      if (updates.version !== undefined) existing.version = updates.version;
    }
    for (const cb of this.edgeUpdateListeners) {
      cb(edgeId, updates);
    }
  }

  // --- Observation CRDT (set union) ---

  addObservation(entityId: string, observation: ObservationRecord): boolean {
    let store = this.entityObservations.get(entityId);
    if (!store) {
      store = new ObservationStore();
      this.entityObservations.set(entityId, store);
    }
    const isNew = store.add(observation);
    if (isNew) {
      for (const cb of this.observationListeners) {
        cb(entityId, observation);
      }
    }
    return isNew;
  }

  getObservations(entityId: string): ObservationRecord[] {
    return this.entityObservations.get(entityId)?.getAll() ?? [];
  }

  mergeObservationStores(entityId: string, other: ObservationStore): ObservationRecord[] {
    let store = this.entityObservations.get(entityId);
    if (!store) {
      store = new ObservationStore();
      this.entityObservations.set(entityId, store);
    }
    const newObs = store.merge(other);
    for (const obs of newObs) {
      for (const cb of this.observationListeners) {
        cb(entityId, obs);
      }
    }
    return newObs;
  }

  // --- IGraphNetworkSubscriber (subscriber interface) ---

  onNodeReceived(callback: NodeCallback): () => void {
    this.nodeListeners.add(callback);
    for (const node of this.nodes.values()) {
      callback(node);
    }
    return () => { this.nodeListeners.delete(callback); };
  }

  onEdgeReceived(callback: EdgeCallback): () => void {
    this.edgeListeners.add(callback);
    for (const edge of this.edges.values()) {
      callback(edge);
    }
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

  onObservationReceived(callback: ObservationCallback): () => void {
    this.observationListeners.add(callback);
    return () => { this.observationListeners.delete(callback); };
  }

  // --- Accessors ---

  getAllNodes(): NetworkNode[] {
    return Array.from(this.nodes.values());
  }

  getAllEdges(): NetworkEdge[] {
    return Array.from(this.edges.values());
  }

  getNode(id: string): NetworkNode | null {
    return this.nodes.get(id) ?? null;
  }

  getEdge(id: string): NetworkEdge | null {
    return this.edges.get(id) ?? null;
  }

  reset(): void {
    this.nodes.clear();
    this.edges.clear();
    this.entityObservations.clear();
    this.nodeListeners.clear();
    this.edgeListeners.clear();
    this.nodeUpdateListeners.clear();
    this.edgeUpdateListeners.clear();
    this.observationListeners.clear();
  }
}
