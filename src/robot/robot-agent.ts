// ===========================================================================
// Robot Agent — 4-Phase Exploration Cycle
//
// Robots do NOT share information except through the Gun protocol.
// Each robot independently:
//   1. ARRIVING: reach target node (starting location is the first node)
//   2. SCANNING: identify new nodes + edges via LiDAR feature detection
//   3. CHOOSING: pick next edge using exploration priority
//   4. TRAVELING: drive to destination, collecting corridor scans for
//      retroactive junction detection on arrival
// ===========================================================================

import type {
  LidarScan,
  OdometryReading,
  MotorCommand,
  Pose2D,
  Vec2,
  DetectedFeature,
  NetworkNode,
  NetworkEdge,
  SimulationConfig,
} from "../shared/types";
import { DEFAULT_SIMULATION_CONFIG, FeatureType, EdgeStatus, BehaviorMode, confidenceTierFromScore } from "../shared/types";
import { chooseBehaviorTarget, createBehaviorState } from "./behavior/behavior-modes";
import type { BehaviorState } from "./behavior/behavior-modes";
import { vec2Distance, normalizeAngle } from "../shared/math";
import { TypedEventEmitter } from "../shared/events";

import { SlamEngine } from "./slam/slam-engine";
import { hasLineOfSight as gridHasLineOfSight } from "../shared/geometry";
import { worldToGrid, gridInBounds } from "./slam/occupancy-grid";
import { FeatureDetector } from "./feature-detection/feature-detector";
import { EdgeMapManager } from "./treadmill/edge-map-manager";
import { NodeMapManager } from "./treadmill/node-map-manager";
import { NetworkGraph } from "./network/network-graph";
import { GraphPublisher } from "./network/graph-publisher";
import type { IGraphNetwork } from "./network/graph-publisher";
import { GraphSubscriber } from "./network/graph-subscriber";
import type { IGraphNetworkSubscriber } from "./network/graph-subscriber";
import { MotorController } from "./navigation/motor-controller";
import { CandidateDetector } from "./merge/candidate-detector";
import { GraphAligner } from "./merge/alignment";

// ---------------------------------------------------------------------------
// Exploration Phase
// ---------------------------------------------------------------------------

enum ExplorationPhase {
  ARRIVING = "arriving",
  SCANNING = "scanning",
  CHOOSING = "choosing",
  TRAVELING = "traveling",
}

// ---------------------------------------------------------------------------
// Robot Events
// ---------------------------------------------------------------------------

export interface DriftCorrectionReport {
  nodeId: string;
  dx: number;
  dy: number;
  confidence: number;
  method: string;
  correlationScore: number;
  overlapCells: number;
  propagatedNodes: number;
}

export interface RobotAgentEvents {
  "node_created": { node: NetworkNode; feature: DetectedFeature };
  "edge_created": { edge: NetworkEdge };
  "edge_traveled": { edge: NetworkEdge };
  "pose_updated": { pose: Pose2D; confidence: number };
  "phase_changed": { phase: string; reason: string };
  "drift_correction": DriftCorrectionReport;
}

// ---------------------------------------------------------------------------
// Robot Agent
// ---------------------------------------------------------------------------

export class RobotAgent {
  readonly id: string;
  readonly events: TypedEventEmitter<RobotAgentEvents>;

  // Subsystems
  private readonly slam: SlamEngine;
  private readonly featureDetector: FeatureDetector;
  private readonly edgeManager: EdgeMapManager;
  private readonly nodeManager: NodeMapManager;
  readonly graph: NetworkGraph;
  private readonly publisher: GraphPublisher;
  private readonly subscriber: GraphSubscriber;
  private readonly motor: MotorController;
  private readonly candidateDetector: CandidateDetector;
  private readonly graphAligner: GraphAligner;
  private alignmentEstablished: boolean = false;

  // Configuration
  private readonly minNodeSeparation: number;
  private readonly scanRadius: number;

  // Phase state
  private phase: ExplorationPhase = ExplorationPhase.ARRIVING;
  private currentNodeId: string | null = null;
  private targetNodeId: string | null = null;
  private targetPosition: Vec2 | null = null;
  private activeEdgeId: string | null = null;
  private scanTicksRemaining: number = 0;

  // Tracking
  private lastPose: Pose2D = { x: 0, y: 0, theta: 0 };
  private tickCount: number = 0;
  private trail: Vec2[] = [];
  private readonly maxTrailLength: number = 150;
  private lastScan: LidarScan | null = null;
  private initialized: boolean = false;

  // Travel tracking
  private travelStartTick: number = 0;
  private travelStuckThreshold: number = 60;
  private travelAccumulatedDistance: number = 0;
  private travelStartPose: Pose2D = { x: 0, y: 0, theta: 0 };

  // Corridor scan log: samples collected during TRAVELING for retroactive
  // junction detection on arrival at the destination node.
  private corridorScanLog: { pose: Pose2D; scan: LidarScan; tick: number }[] = [];
  private lastCorridorSamplePos: Vec2 = { x: 0, y: 0 };

  // Manual target: multi-hop path through the graph.
  // Consumed one hop at a time, with a full scan cycle at each node.
  private manualPathQueue: { nodeId: string; edgeId: string }[] = [];

  // Behavior mode
  private behaviorState: BehaviorState = createBehaviorState();

  constructor(id: string, config: SimulationConfig = DEFAULT_SIMULATION_CONFIG) {
    this.id = id;
    this.events = new TypedEventEmitter();
    this.minNodeSeparation = config.min_node_separation;
    this.scanRadius = config.lidar.max_range;

    this.slam = new SlamEngine({
      particle_count: config.slam_particle_count,
      grid_resolution: config.slam_resolution,
      grid_size: config.slam_grid_size,
      odom_linear_noise: config.odometry.linear_noise_fraction,
      odom_angular_noise: config.odometry.angular_noise_fraction,
      odom_linear_floor: config.odometry.linear_noise_floor,
      odom_angular_floor: config.odometry.angular_noise_floor,
    });
    this.featureDetector = new FeatureDetector({
      min_separation: config.min_node_separation,
      range_jump_threshold: 1.5,
    });
    this.edgeManager = new EdgeMapManager();
    this.nodeManager = new NodeMapManager();
    this.graph = new NetworkGraph(id);
    this.publisher = new GraphPublisher(id);
    this.subscriber = new GraphSubscriber(id, this.graph);
    this.motor = new MotorController({
      max_linear_speed: config.robot_physics.max_linear_speed * 0.9,
      max_angular_speed: config.robot_physics.max_angular_speed * 0.9,
      ahead_clear_threshold: 0.8,
      robot_radius: config.robot_physics.radius,
    });
    this.candidateDetector = new CandidateDetector();
    this.graphAligner = new GraphAligner();
  }

  // =========================================================================
  // Main Tick
  // =========================================================================

  processTick(lidar: LidarScan, odometry: OdometryReading): MotorCommand {
    this.tickCount++;
    this.lastScan = lidar;

    if (!this.initialized) {
      this.slam.initialize();
      this.initialized = true;
      this.phase = ExplorationPhase.ARRIVING;

      // First tick: use exact SLAM origin for the starting node.
      // Do NOT run processSensorData — it would move the pose away from origin
      // due to particle noise. The grid will be populated during the 5-tick
      // scanning phase (ticks 2-6) when processSensorData runs normally.
      const firstPose: Pose2D = { x: 0, y: 0, theta: 0 };
      this.lastPose = { ...firstPose };
      this.lastScan = lidar;
      const cmd = this.doArriving(lidar, firstPose);
      this.events.emit("pose_updated", { pose: firstPose, confidence: 1.0 });
      return cmd;
    }

    const slamResult = this.slam.processSensorData(odometry, lidar);
    const pose = slamResult.pose;

    // Track trail
    if (this.trail.length === 0 ||
        vec2Distance({ x: pose.x, y: pose.y }, { x: this.lastPose.x, y: this.lastPose.y }) > 0.3) {
      this.trail.push({ x: pose.x, y: pose.y });
      if (this.trail.length > this.maxTrailLength) this.trail.shift();
    }

    // Track travel distance during movement phase
    if (this.phase === ExplorationPhase.TRAVELING) {
      this.travelAccumulatedDistance += vec2Distance(
        { x: pose.x, y: pose.y },
        { x: this.lastPose.x, y: this.lastPose.y },
      );
      this.edgeManager.update(pose, this.lastPose);

      // Collect corridor scan samples every 0.3m for retroactive junction detection
      if (vec2Distance({ x: pose.x, y: pose.y }, this.lastCorridorSamplePos) >= 0.3) {
        this.corridorScanLog.push({
          pose: { ...pose },
          scan: lidar,
          tick: this.tickCount,
        });
        this.lastCorridorSamplePos = { x: pose.x, y: pose.y };
      }
    }

    let command: MotorCommand;
    switch (this.phase) {
      case ExplorationPhase.ARRIVING:
        command = this.doArriving(lidar, pose);
        break;
      case ExplorationPhase.SCANNING:
        command = this.doScanning(lidar, pose);
        break;
      case ExplorationPhase.CHOOSING:
        command = this.doChoosing(lidar, pose);
        break;
      case ExplorationPhase.TRAVELING:
        command = this.doTraveling(lidar, pose);
        break;
    }

    // Periodic freshness decay and dead-entity pruning (every 100 ticks, any phase)
    if (this.tickCount % 100 === 0) {
      this.graph.applyFreshnessDecay(lidar.timestamp);
      this.graph.pruneDeadEntities();
    }

    this.lastPose = { ...pose };
    this.events.emit("pose_updated", { pose, confidence: slamResult.confidence });
    return command;
  }

  // =========================================================================
  // Step 1: ARRIVE at target node
  // =========================================================================

  private doArriving(scan: LidarScan, pose: Pose2D): MotorCommand {
    if (!this.currentNodeId) {
      const startFeature: DetectedFeature = {
        type: FeatureType.Junction,
        position: { x: pose.x, y: pose.y },
        confidence: 0.8,
        heading: pose.theta,
        exit_directions: [],
        local_radius: 1.5,
      };
      // Try createNode first (runs ±20° clearance adjustment), fall back to
      // createNodeAtPosition if the ring check rejects (e.g. tight start spot)
      let node = this.createNode(startFeature, scan);
      if (!node) {
        node = this.graph.createNodeAtPosition(
          { x: pose.x, y: pose.y },
          FeatureType.Junction,
          0,
          scan.timestamp,
        );
        this.featureDetector.registerKnownFeature({ x: pose.x, y: pose.y });
        this.publisher.publishNode(node);
        this.events.emit("node_created", { node, feature: startFeature });
      }
      this.currentNodeId = node.id;
      if (!this.behaviorState.startingNodeId) {
        this.behaviorState.startingNodeId = node.id;
      }
      this.nodeManager.createNodeMap(node.id, startFeature, this.slam.grid, scan);
    }

    this.graph.markNodeExplored(this.currentNodeId!, this.id, scan.timestamp);

    // SNAP NODE TO ROBOT POSITION on every arrival.
    // With noise, the SLAM pose drifts. The robot IS here — update the node
    // to where the robot actually reached. This corrects accumulated drift.
    const node = this.graph.getNode(this.currentNodeId!);
    if (node) {
      node.position.x = pose.x;
      node.position.y = pose.y;

      // Push away from walls using LiDAR (no SLAM grid — LiDAR is ground truth)
      const clearance = 0.3;
      const checkDist = 0.5;
      let pushX = 0, pushY = 0;
      const dirs = [
        { dx: 1, dy: 0 }, { dx: -1, dy: 0 },
        { dx: 0, dy: 1 }, { dx: 0, dy: -1 },
      ];
      for (const d of dirs) {
        const relAngle = Math.atan2(d.dy, d.dx) - pose.theta;
        let wallDist = checkDist;
        for (let i = 0; i < scan.angles.length; i++) {
          if (Math.abs(normalizeAngle(scan.angles[i] - relAngle)) < 0.15) {
            if (scan.ranges[i] < wallDist) wallDist = scan.ranges[i];
          }
        }
        if (wallDist < clearance) {
          const push = (clearance - wallDist) * 0.8;
          pushX -= d.dx * push;
          pushY -= d.dy * push;
        }
      }
      if (Math.abs(pushX) > 0.01 || Math.abs(pushY) > 0.01) {
        // Verify the push destination is clear via LiDAR: check that
        // the ray toward the push direction reaches far enough
        const pushAngle = normalizeAngle(Math.atan2(pushY, pushX) - pose.theta);
        let pushClear = true;
        for (let i = 0; i < scan.angles.length; i++) {
          if (Math.abs(normalizeAngle(scan.angles[i] - pushAngle)) < 0.15) {
            const pushDist = Math.sqrt(pushX * pushX + pushY * pushY);
            if (scan.ranges[i] < pushDist + 0.2) {
              pushClear = false;
              break;
            }
          }
        }
        if (pushClear) {
          node.position.x += pushX;
          node.position.y += pushY;
        }
      }
      node.version++;
    }

    // Capture SLAM data: create on first visit, update on revisit
    const existingMap = this.nodeManager.getNodeMap(this.currentNodeId!);
    if (existingMap) {
      this.nodeManager.updateNodeMap(this.currentNodeId!, this.slam.grid, scan);
    } else {
      const node = this.graph.getNode(this.currentNodeId!);
      if (node) {
        this.nodeManager.createNodeMap(this.currentNodeId!, {
          type: node.type, position: { ...node.position },
          confidence: node.confidence, heading: pose.theta,
          exit_directions: [], local_radius: 1.5,
        }, this.slam.grid, scan);
        node.local_map_hash = this.nodeManager.getContentHash(this.currentNodeId!);
      }
    }

    // Node-arrival scan match: the ONLY place correlative scan matching runs.
    // This anchors the pose against the accumulated grid topology.
    if (this.tickCount > 50 && this.graph.exploredNodeCount > 2) {
      this.slam.refinePoseAtNode(scan);
    }

    this.scanTicksRemaining = 5;
    this.setPhase(ExplorationPhase.SCANNING, "arrived_at_node");
    return this.motor.stop(scan.timestamp);
  }

  // =========================================================================
  // Step 2: SCAN — identify new unvisited nodes in every direction
  // =========================================================================

  private doScanning(scan: LidarScan, pose: Pose2D): MotorCommand {
    this.scanTicksRemaining--;
    if (this.scanTicksRemaining > 0) {
      return this.motor.stop(scan.timestamp);
    }

    const features = this.featureDetector.detectFromScan(scan, pose);

    for (const feature of features) {
      const distFromRobot = vec2Distance(feature.position, { x: pose.x, y: pose.y });
      if (distFromRobot < 0.5) continue;
      if (distFromRobot > this.scanRadius * 0.85) continue;

      if (!this.currentNodeId) continue;
      if (!this.hasDirectLineOfSight(scan, pose, feature.position.x, feature.position.y, distFromRobot)) continue;

      // Check if there's already a nearby node — connect to it instead of creating new
      const nearbyExisting = this.graph.findNearestNode(feature.position);
      if (nearbyExisting && vec2Distance(feature.position, nearbyExisting.position) < this.minNodeSeparation) {
        if (!this.graph.hasEdgeBetween(this.currentNodeId, nearbyExisting.id) &&
            nearbyExisting.id !== this.currentNodeId) {
          const edgeDist = vec2Distance({ x: pose.x, y: pose.y }, nearbyExisting.position);
          if (!this.hasDirectLineOfSight(scan, pose, nearbyExisting.position.x, nearbyExisting.position.y, edgeDist)) {
            this.graph.strengthenNode(nearbyExisting.id);
            continue;
          }
          const bearing = normalizeAngle(Math.atan2(
            nearbyExisting.position.y - pose.y, nearbyExisting.position.x - pose.x,
          ));
          const edge = this.graph.createEdge(
            this.currentNodeId, nearbyExisting.id, edgeDist, bearing, scan.timestamp, false,
          );
          if (edge) {
            this.publisher.publishEdge(edge);
            this.events.emit("edge_created", { edge });
          }
        }
        this.graph.strengthenNode(nearbyExisting.id);
        continue;
      }

      // createNode handles the ±10° clearance adjustment internally
      const newNode = this.createNode(feature, scan);
      if (newNode) {
        const bearing = normalizeAngle(Math.atan2(
          newNode.position.y - pose.y, newNode.position.x - pose.x,
        ));
        const edgeDist = vec2Distance({ x: pose.x, y: pose.y }, newNode.position);
        const edge = this.graph.createEdge(
          this.currentNodeId, newNode.id, edgeDist, bearing, scan.timestamp, false,
        );
        if (!edge) {
          this.graph.removeNode(newNode.id);
        }
      }
    }

    // Update exit count on current node
    if (this.currentNodeId) {
      const currentNode = this.graph.getNode(this.currentNodeId);
      if (currentNode) {
        const edges = this.graph.getNodeEdges(this.currentNodeId);
        const distantFeatures = features.filter(f =>
          vec2Distance(f.position, { x: pose.x, y: pose.y }) > 1.0,
        );
        currentNode.exit_count = Math.max(
          currentNode.exit_count,
          edges.length + distantFeatures.length,
        );
      }
    }

    // Step 3: identify untraveled edges via visibility check
    this.identifyVisibleEdges(scan, pose);

    // Detect side passages (perpendicular openings) from this position
    this.detectSidePassages(scan, pose);

    // Revalidate existing edges from this node — if a previously visible
    // node is no longer visible (wall moved, path blocked), weaken the edge
    if (this.currentNodeId) {
      const visibleIds = new Set<string>();
      const nodesInRange = this.graph.findNodesInRadius({ x: pose.x, y: pose.y }, this.scanRadius);
      for (const node of nodesInRange) {
        if (node.id === this.currentNodeId) continue;
        if (this.hasLineOfSight(pose.x, pose.y, node.position.x, node.position.y)) {
          visibleIds.add(node.id);
        }
      }
      this.graph.revalidateEdgesFromNode(this.currentNodeId, visibleIds, scan.timestamp);
    }


    // Attempt to discover and align with other robots' graphs
    if (!this.alignmentEstablished && this.tickCount % 50 === 0) {
      this.attemptMergeAlignment();
    }

    this.setPhase(ExplorationPhase.CHOOSING, "scan_complete");
    return this.motor.stop(scan.timestamp);
  }

  // =========================================================================
  // Step 3: CONNECT — identify untraveled edges via visibility check
  // =========================================================================

  private identifyVisibleEdges(scan: LidarScan, pose: Pose2D): void {
    if (!this.currentNodeId) return;

    const nodesInRange = this.graph.findNodesInRadius(
      { x: pose.x, y: pose.y },
      this.scanRadius,
    );

    // Sort by distance — connect to nearest nodes first
    const candidates: { node: NetworkNode; dist: number }[] = [];
    for (const node of nodesInRange) {
      if (node.id === this.currentNodeId) continue;
      if (this.graph.hasEdgeBetween(this.currentNodeId, node.id)) continue;
      const dx = node.position.x - pose.x;
      const dy = node.position.y - pose.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < 1.0) continue;
      candidates.push({ node, dist });
    }
    candidates.sort((a, b) => a.dist - b.dist);

    // Limit to 6 edges per scan, and skip nodes that are behind a closer node
    // in roughly the same direction (within 15°). If n1 is between us and n2,
    // we don't need an edge to n2 — we can reach it through n1.
    let edgesCreated = 0;
    const connectedAngles: number[] = [];
    const maxEdgesPerScan = 6;

    for (const { node, dist } of candidates) {
      if (edgesCreated >= maxEdgesPerScan) break;

      const dx = node.position.x - pose.x;
      const dy = node.position.y - pose.y;
      const angleToNode = Math.atan2(dy, dx);

      // Skip if a closer node is already connected in roughly the same direction.
      // Node ----.----' : n1 is between us and n2, so skip n2.
      let occluded = false;
      for (const prevAngle of connectedAngles) {
        if (Math.abs(normalizeAngle(angleToNode - prevAngle)) < 15 * Math.PI / 180) {
          occluded = true;
          break;
        }
      }
      if (occluded) continue;

      // Edge creation uses DIRECT ray only — no ±20° search.
      // The edge is a straight line from A to B; if the direct ray can't
      // see B, the straight-line edge would go through a wall.
      if (!this.hasDirectLineOfSight(scan, pose, node.position.x, node.position.y, dist)) continue;
      {
        const bearing = normalizeAngle(Math.atan2(
          node.position.y - pose.y, node.position.x - pose.x,
        ));
        const edge = this.graph.createEdge(
          this.currentNodeId, node.id, dist, bearing, scan.timestamp, false,
        );
        if (edge) {
          this.publisher.publishEdge(edge);
          this.events.emit("edge_created", { edge });
          edgesCreated++;
          connectedAngles.push(angleToNode);
        }
      }
    }
  }

  // =========================================================================
  // Step 4: CHOOSE — pick an edge using exploration priority
  // =========================================================================

  private doChoosing(scan: LidarScan, pose: Pose2D): MotorCommand {
    if (!this.currentNodeId) {
      this.setPhase(ExplorationPhase.ARRIVING, "no_current_node");
      return this.motor.stop(scan.timestamp);
    }

    // Priority 0: manual path queue — follow the next hop if available.
    // At each node we do a full ARRIVING->SCANNING->CHOOSING cycle first,
    // so SLAM data and edges are updated before continuing.
    if (this.manualPathQueue.length > 0) {
      const nextHop = this.manualPathQueue[0];
      const edge = this.graph.getEdge(nextHop.edgeId);
      const targetNode = this.graph.getNode(nextHop.nodeId);

      if (!edge || !targetNode || edge.traversability.status === EdgeStatus.Blocked) {
        // Edge blocked or gone — recompute from here to final destination
        const finalId = this.manualPathQueue[this.manualPathQueue.length - 1].nodeId;
        this.manualPathQueue = [];
        const newPath = this.graph.findPath(this.currentNodeId, finalId);
        if (newPath && newPath.length > 0) {
          this.manualPathQueue = newPath;
          return this.doChoosing(scan, pose);
        }
        // Unreachable — clear and fall through to normal exploration
      } else {
        this.manualPathQueue.shift();
        return this.beginHopTo(targetNode, edge, pose, scan.timestamp, "manual_path_hop");
      }
    }

    // Idle mode: stop and wait
    if (this.behaviorState.mode === BehaviorMode.Idle) {
      return this.motor.stop(scan.timestamp);
    }

    // Behavior mode target selection
    const modeChoice = chooseBehaviorTarget(
      this.behaviorState, this.currentNodeId, this.graph, pose, this.tickCount,
    );

    // If mode returned a target, use it
    if (modeChoice) {
      const targetNode = this.graph.getOtherNode(modeChoice.edge, this.currentNodeId);
      if (targetNode) {
        return this.beginHopTo(targetNode, modeChoice.edge, pose, scan.timestamp, modeChoice.reason);
      }
    }

    // Fallback: grid frontier exploration (for any mode that returned null)
    const gridTarget = this.findGridFrontier(pose);
    if (gridTarget) {
      this.targetPosition = gridTarget;
      this.targetNodeId = null;
      this.activeEdgeId = null;
      this.beginTravel(pose, scan.timestamp);
      this.setPhase(ExplorationPhase.TRAVELING, "grid_frontier");
      return this.motor.stop(scan.timestamp);
    }

    // Nothing to do — wait and retry
    this.scanTicksRemaining = 20;
    this.setPhase(ExplorationPhase.SCANNING, "no_targets_retry");
    return this.motor.stop(scan.timestamp);
  }

  private beginHopTo(
    targetNode: NetworkNode,
    edge: NetworkEdge,
    pose: Pose2D,
    timestamp: number,
    reason: string,
  ): MotorCommand {
    this.targetNodeId = targetNode.id;
    this.targetPosition = { ...targetNode.position };
    this.activeEdgeId = edge.id;

    this.beginTravel(pose, timestamp);
    this.setPhase(ExplorationPhase.TRAVELING, reason);
    return this.motor.stop(timestamp);
  }

  // =========================================================================
  // Step 5: TRAVEL — drive to destination, collecting corridor scans
  // =========================================================================

  private doTraveling(scan: LidarScan, pose: Pose2D): MotorCommand {
    if (!this.targetPosition) {
      this.setPhase(ExplorationPhase.ARRIVING, "no_target");
      return this.motor.stop(scan.timestamp);
    }

    if (this.checkWallStop(scan, pose, this.targetPosition)) {
      return this.motor.stop(scan.timestamp);
    }

    // Live target update: if the target node position was corrected (e.g.,
    // another robot visited it and snapped it), use the updated position.
    if (this.targetNodeId) {
      const tNode = this.graph.getNode(this.targetNodeId);
      if (tNode) {
        this.targetPosition = { ...tNode.position };
      }
    }

    const distToTarget = vec2Distance({ x: pose.x, y: pose.y }, this.targetPosition);

    // Arrive: close enough to destination
    if (distToTarget < 0.5) {
      this.completeTravel(scan, pose);
      this.analyzeCorridorOnArrival(scan.timestamp);
      this.setPhase(ExplorationPhase.ARRIVING, "arrived_at_destination");
      return this.motor.stop(scan.timestamp);
    }

    if (this.tickCount - this.travelStartTick > this.travelStuckThreshold * 2) {
      return this.handleStuck(scan, pose);
    }

    return this.motor.driveToward(this.targetPosition, pose, scan, scan.timestamp);
  }

  // =========================================================================
  // Side Passage Detection (during travel)
  // Scans perpendicular to travel direction for openings (wall→free transition).
  // When found: creates junction node at the perpendicular point on the path
  // and a target node at 75% vision range into the opening.
  // =========================================================================

  private lastSidePassageCheck: Vec2 = { x: 0, y: 0 };

  private detectSidePassages(scan: LidarScan, pose: Pose2D): void {
    // Check every 0.5m of travel (connectors are only 1m wide)
    if (vec2Distance({ x: pose.x, y: pose.y }, this.lastSidePassageCheck) < 0.5) return;
    this.lastSidePassageCheck = { x: pose.x, y: pose.y };

    // Don't detect side passages in very narrow corridors — the junction
    // node would land in a wall. Check that both side walls are > robot radius away.
    let leftWall = scan.max_range, rightWall = scan.max_range;
    for (let i = 0; i < scan.angles.length; i++) {
      if (Math.abs(scan.angles[i] - Math.PI / 2) < 0.15) {
        if (scan.ranges[i] < leftWall) leftWall = scan.ranges[i];
      }
      if (Math.abs(scan.angles[i] + Math.PI / 2) < 0.15) {
        if (scan.ranges[i] < rightWall) rightWall = scan.ranges[i];
      }
    }
    const corridorWidth = leftWall + rightWall;
    if (corridorWidth < 1.0) return; // Too narrow for a junction node

    const jumpThreshold = 1.5;

    for (let i = 3; i < scan.ranges.length - 3; i++) {
      const r = scan.ranges[i];
      const rNext = scan.ranges[i + 1];

      // Detect short→long transition (wall → opening)
      if (rNext - r < jumpThreshold) continue;
      if (scan.ranges[i + 2] - r < jumpThreshold) continue;

      // The short ray hits a wall; the long ray looks into the opening
      const wallDist = r;
      if (wallDist < 0.3 || wallDist > this.scanRadius * 0.8) continue;

      const openAngle = scan.angles[i + 1];
      const absAngle = Math.abs(openAngle);

      // Only care about side openings (roughly perpendicular to heading)
      if (absAngle < Math.PI / 4 || absAngle > 3 * Math.PI / 4) continue;

      const openRange = Math.max(scan.ranges[i + 1], scan.ranges[i + 2]);
      if (openRange < 2.0) continue;

      const absOpenAngle = pose.theta + openAngle;

      // Check if there's already a node visible through this opening
      const nodesInRange = this.graph.findNodesInRadius(
        { x: pose.x, y: pose.y }, openRange,
      );
      let existingNode: NetworkNode | null = null;
      for (const node of nodesInRange) {
        const dx = node.position.x - pose.x;
        const dy = node.position.y - pose.y;
        const angleToNode = normalizeAngle(Math.atan2(dy, dx) - pose.theta);
        if (Math.abs(normalizeAngle(angleToNode - openAngle)) < 0.3) {
          existingNode = node;
          break;
        }
      }

      // Find or create the anchor (nearest existing node to current position)
      let anchorId = this.currentNodeId;
      const nearestNode = this.graph.findNearestNode({ x: pose.x, y: pose.y });
      if (nearestNode && vec2Distance({ x: pose.x, y: pose.y }, nearestNode.position) < 1.5) {
        anchorId = nearestNode.id;
      }

      if (existingNode && anchorId) {
        // Case 1: Can see an existing node through the opening → edge to it
        if (!this.graph.hasEdgeBetween(anchorId, existingNode.id)) {
          const anchorPos = this.graph.getNode(anchorId)!.position;
          if (this.hasLineOfSight(anchorPos.x, anchorPos.y, existingNode.position.x, existingNode.position.y)) {
            const dist = vec2Distance(anchorPos, existingNode.position);
            const bearing = normalizeAngle(Math.atan2(
              existingNode.position.y - anchorPos.y,
              existingNode.position.x - anchorPos.x,
            ));
            const edge = this.graph.createEdge(
              anchorId, existingNode.id, dist, bearing, scan.timestamp, false,
            );
            if (edge) {
              this.publisher.publishEdge(edge);
              this.events.emit("edge_created", { edge });
            }
          }
        }
      } else {
        // Case 2: No node visible → create junction + target node
        // Junction node at the current robot position (on the path)
        const junctionPos: Vec2 = { x: pose.x, y: pose.y };
        const nearJunction = this.graph.findNearestNode(junctionPos);
        let junctionId: string | null = null;

        if (nearJunction && vec2Distance(junctionPos, nearJunction.position) < 1.5) {
          junctionId = nearJunction.id;
        } else {
          const junctionFeature: DetectedFeature = {
            type: FeatureType.Junction,
            position: junctionPos,
            confidence: 0.7,
            heading: pose.theta,
            exit_directions: [pose.theta, absOpenAngle],
            local_radius: 1.0,
          };
          if (anchorId) {
            const ap = this.graph.getNode(anchorId)!.position;
            if (this.hasLineOfSight(ap.x, ap.y, junctionPos.x, junctionPos.y)) {
              const jNode = this.createNode(junctionFeature, scan);
              if (jNode) {
                junctionId = jNode.id;
                const dist = vec2Distance(ap, jNode.position);
                const jBearing = normalizeAngle(Math.atan2(
                  jNode.position.y - ap.y, jNode.position.x - ap.x,
                ));
                const edge = this.graph.createEdge(anchorId, junctionId, dist, jBearing, scan.timestamp, true);
                if (!edge) {
                  this.graph.removeNode(jNode.id);
                  junctionId = null;
                }
              }
            }
          }
        }

        // Target node at 75% of visible range into the opening
        if (junctionId) {
          const targetDist = openRange * 0.75;
          const targetPos: Vec2 = {
            x: pose.x + targetDist * Math.cos(absOpenAngle),
            y: pose.y + targetDist * Math.sin(absOpenAngle),
          };

          const nearTarget = this.graph.findNearestNode(targetPos);
          if (!nearTarget || vec2Distance(targetPos, nearTarget.position) > 1.5) {
            {
              const targetFeature: DetectedFeature = {
                type: FeatureType.Corner,
                position: targetPos,
                confidence: 0.5,
                heading: absOpenAngle,
                exit_directions: [absOpenAngle],
                local_radius: 1.0,
              };
              const jp = this.graph.getNode(junctionId)!.position;
              if (this.hasLineOfSight(jp.x, jp.y, targetPos.x, targetPos.y)) {
                const tNode = this.createNode(targetFeature, scan);
                if (tNode) {
                  const edgeDist = vec2Distance(jp, tNode.position);
                  const edge = this.graph.createEdge(
                    junctionId, tNode.id, edgeDist, absOpenAngle, scan.timestamp, false,
                  );
                  if (edge) {
                    this.publisher.publishEdge(edge);
                    this.events.emit("edge_created", { edge });
                  } else {
                    this.graph.removeNode(tNode.id);
                  }
                }
              }
            }
          }
        }
      }

      i += 10; // skip ahead to avoid double-detecting the same opening
    }
  }

  // =========================================================================
  // Travel Management
  // =========================================================================

  private beginTravel(pose: Pose2D, timestamp: number): void {
    this.travelStartTick = this.tickCount;
    this.travelAccumulatedDistance = 0;
    this.travelStartPose = { ...pose };
    this.corridorScanLog = [];
    this.lastCorridorSamplePos = { x: pose.x, y: pose.y };
    if (this.currentNodeId) {
      this.edgeManager.startEdge(this.currentNodeId, pose, timestamp);
    }
  }

  private completeTravel(scan: LidarScan, pose: Pose2D): void {
    const activeEdge = this.edgeManager.getActiveEdge();
    if (activeEdge && this.currentNodeId) {
      let arrivalNodeId = this.targetNodeId;

      if (!arrivalNodeId || !this.graph.getNode(arrivalNodeId)) {
        const feature: DetectedFeature = {
          type: FeatureType.Junction,
          position: { x: pose.x, y: pose.y },
          confidence: 0.7,
          heading: pose.theta,
          exit_directions: [],
          local_radius: 1.5,
        };
        const node = this.createNode(feature, scan);
        if (node) {
          arrivalNodeId = node.id;
        } else {
          this.edgeManager.cancelActiveEdge();
          return;
        }
      }

      // Pass the graph edge ID so the corridor map is stored under the correct key
      const graphEdgeId = this.activeEdgeId ?? undefined;
      const completedEdgeData = this.edgeManager.completeEdge(
        arrivalNodeId, this.slam.grid, scan.timestamp, graphEdgeId,
      );

      if (this.activeEdgeId) {
        // Geometric bearing from start to end position (not heading delta)
        const measuredBearing = normalizeAngle(Math.atan2(
          pose.y - this.travelStartPose.y,
          pose.x - this.travelStartPose.x,
        ));
        this.graph.markEdgeTraveled(
          this.activeEdgeId,
          this.id,
          this.travelAccumulatedDistance,
          measuredBearing,
          scan.timestamp,
        );
        const edge = this.graph.getEdge(this.activeEdgeId);
        if (edge) {
          this.events.emit("edge_traveled", { edge });
        }
      } else if (arrivalNodeId) {
        const edge = this.graph.createEdge(
          this.currentNodeId,
          arrivalNodeId,
          this.travelAccumulatedDistance,
          normalizeAngle(Math.atan2(pose.y - this.travelStartPose.y, pose.x - this.travelStartPose.x)),
          scan.timestamp,
          true,
        );
        if (edge) {
          if (completedEdgeData) completedEdgeData.id = edge.id; // update to actual graph ID
          this.publisher.publishEdge(edge);
          this.events.emit("edge_traveled", { edge });
        }
      }

      this.currentNodeId = arrivalNodeId;
    }

    this.targetNodeId = null;
    this.targetPosition = null;
    this.activeEdgeId = null;

    this.travelAccumulatedDistance = 0;
  }

  // =========================================================================
  // Retroactive Corridor Junction Detection
  //
  // After arriving at the destination, analyze the corridor scan log collected
  // during TRAVELING. For each pair of consecutive samples, compute
  // perpendicular ranges (left/right of travel direction). A sudden jump
  // (range increase > 1.5m) indicates a side passage. Valid passages
  // (width > 0.5m) produce junction nodes on the A->B line, splitting the
  // traveled edge into sub-edges and creating untraveled branch edges into
  // the detected passages.
  // =========================================================================

  private analyzeCorridorOnArrival(timestamp: number): void {
    if (this.corridorScanLog.length < 2 || !this.currentNodeId) {
      this.corridorScanLog = [];
      return;
    }

    // Determine the departure node (A) and arrival node (B).
    // After completeTravel, currentNodeId is B. The first sample's pose
    // is near A. Find A as the nearest node to the first sample.
    const firstSample = this.corridorScanLog[0];
    const departureNode = this.graph.findNearestNode({
      x: firstSample.pose.x,
      y: firstSample.pose.y,
    });
    const arrivalNodeId = this.currentNodeId;
    const arrivalNode = this.graph.getNode(arrivalNodeId);

    if (!departureNode || !arrivalNode || departureNode.id === arrivalNodeId) {
      this.corridorScanLog = [];
      return;
    }

    // Find the traveled edge between A and B (the one just completed)
    const edges = this.graph.getNodeEdges(departureNode.id);
    let traveledEdgeId: string | null = null;
    for (const e of edges) {
      const other = e.source_node === departureNode.id ? e.target_node : e.source_node;
      if (other === arrivalNodeId && e.traveled) {
        traveledEdgeId = e.id;
        break;
      }
    }

    // Travel direction: A -> B
    const travelDx = arrivalNode.position.x - departureNode.position.x;
    const travelDy = arrivalNode.position.y - departureNode.position.y;
    const travelDist = Math.sqrt(travelDx * travelDx + travelDy * travelDy);
    if (travelDist < 0.1) {
      this.corridorScanLog = [];
      return;
    }
    const travelAngle = Math.atan2(travelDy, travelDx);
    // Perpendicular angles: left and right of travel direction
    const perpLeft = travelAngle + Math.PI / 2;
    const perpRight = travelAngle - Math.PI / 2;

    // For each sample, compute perpendicular ranges (left and right)
    interface CorridorSample {
      pose: Pose2D;
      scan: LidarScan;
      tick: number;
      leftRange: number;
      rightRange: number;
    }
    const samples: CorridorSample[] = [];
    for (const entry of this.corridorScanLog) {
      const { pose: p, scan: s } = entry;
      let leftRange = s.max_range;
      let rightRange = s.max_range;

      // Find the best ray matching perpendicular directions
      const coneWidth = 0.25; // ~15 degrees
      for (let i = 0; i < s.angles.length; i++) {
        const absRayAngle = normalizeAngle(s.angles[i]); // relative to robot heading
        const rayWorldAngle = normalizeAngle(p.theta + absRayAngle);

        const diffLeft = Math.abs(normalizeAngle(rayWorldAngle - perpLeft));
        if (diffLeft < coneWidth && s.ranges[i] < leftRange) {
          leftRange = s.ranges[i];
        }
        const diffRight = Math.abs(normalizeAngle(rayWorldAngle - perpRight));
        if (diffRight < coneWidth && s.ranges[i] < rightRange) {
          rightRange = s.ranges[i];
        }
      }

      samples.push({
        pose: entry.pose,
        scan: entry.scan,
        tick: entry.tick,
        leftRange,
        rightRange,
      });
    }

    // Detect jumps: range increase > 1.5m between consecutive samples
    const jumpThreshold = 1.5;
    const minPassageWidth = 0.5;

    interface DetectedPassage {
      position: Vec2;       // position on the A->B line (projection of sample pose)
      direction: number;    // angle into the passage
      range: number;        // how far the passage extends
      sampleIdx: number;
    }
    const passages: DetectedPassage[] = [];

    for (let i = 1; i < samples.length; i++) {
      const prev = samples[i - 1];
      const curr = samples[i];

      // Check left side
      if (curr.leftRange - prev.leftRange > jumpThreshold) {
        // Passage opened on the left
        // Verify the passage has meaningful width by checking if the
        // next sample also sees the opening
        const nextLeftRange = (i + 1 < samples.length) ? samples[i + 1].leftRange : curr.leftRange;
        if (nextLeftRange > prev.leftRange + minPassageWidth) {
          passages.push({
            position: { x: curr.pose.x, y: curr.pose.y },
            direction: perpLeft,
            range: curr.leftRange,
            sampleIdx: i,
          });
        }
      }

      // Check right side
      if (curr.rightRange - prev.rightRange > jumpThreshold) {
        const nextRightRange = (i + 1 < samples.length) ? samples[i + 1].rightRange : curr.rightRange;
        if (nextRightRange > prev.rightRange + minPassageWidth) {
          passages.push({
            position: { x: curr.pose.x, y: curr.pose.y },
            direction: perpRight,
            range: curr.rightRange,
            sampleIdx: i,
          });
        }
      }
    }

    if (passages.length === 0) {
      this.corridorScanLog = [];
      return;
    }

    // De-duplicate passages that are too close together (within 1.0m)
    const dedupedPassages: DetectedPassage[] = [];
    for (const p of passages) {
      let isDuplicate = false;
      for (const existing of dedupedPassages) {
        if (vec2Distance(p.position, existing.position) < 1.0 &&
            Math.abs(normalizeAngle(p.direction - existing.direction)) < 0.5) {
          isDuplicate = true;
          break;
        }
      }
      if (!isDuplicate) {
        dedupedPassages.push(p);
      }
    }

    // For each passage, check if a node already exists nearby. Skip if so.
    // Otherwise, create a junction node on the A->B line and a target node
    // into the passage.

    // Collect junction nodes that will split the A->B edge
    const junctions: { nodeId: string; distFromA: number }[] = [];

    for (const passage of dedupedPassages) {
      // Project the passage position onto the A->B line
      const apx = passage.position.x - departureNode.position.x;
      const apy = passage.position.y - departureNode.position.y;
      const t = Math.max(0.05, Math.min(0.95,
        (apx * travelDx + apy * travelDy) / (travelDist * travelDist),
      ));
      const projX = departureNode.position.x + t * travelDx;
      const projY = departureNode.position.y + t * travelDy;
      const junctionPos: Vec2 = { x: projX, y: projY };

      // Skip if too close to an existing node
      const nearestNode = this.graph.findNearestNode(junctionPos);
      if (nearestNode && vec2Distance(junctionPos, nearestNode.position) < this.minNodeSeparation) {
        // Use existing node as the junction instead of creating a new one
        // Still create the branch edge if the passage target doesn't exist
        const targetDist = passage.range * 0.75;
        if (targetDist < 1.0) continue;
        const targetPos: Vec2 = {
          x: junctionPos.x + targetDist * Math.cos(passage.direction),
          y: junctionPos.y + targetDist * Math.sin(passage.direction),
        };
        const nearTarget = this.graph.findNearestNode(targetPos);
        if (!nearTarget || vec2Distance(targetPos, nearTarget.position) > this.minNodeSeparation) {
          {
            const tFeature: DetectedFeature = {
              type: FeatureType.Corner,
              position: targetPos,
              confidence: 0.5,
              heading: passage.direction,
              exit_directions: [passage.direction],
              local_radius: 1.0,
            };
            if (this.hasLineOfSight(nearestNode.position.x, nearestNode.position.y, targetPos.x, targetPos.y)) {
              const tNode = this.createNode(tFeature, samples[passage.sampleIdx].scan);
              if (tNode) {
                const edgeDist = vec2Distance(nearestNode.position, tNode.position);
                const bearing = normalizeAngle(Math.atan2(
                  tNode.position.y - nearestNode.position.y,
                  tNode.position.x - nearestNode.position.x,
                ));
                const edge = this.graph.createEdge(
                  nearestNode.id, tNode.id, edgeDist, bearing, timestamp, false,
                );
                if (!edge) {
                  this.graph.removeNode(tNode.id);
                }
              }
            }
          }
        }
        continue;
      }

      // Create junction node on the A->B line

      const jFeature: DetectedFeature = {
        type: FeatureType.Junction,
        position: junctionPos,
        confidence: 0.7,
        heading: travelAngle,
        exit_directions: [travelAngle, normalizeAngle(travelAngle + Math.PI), passage.direction],
        local_radius: 1.0,
      };
      const jNode = this.createNode(jFeature, samples[passage.sampleIdx].scan);
      if (!jNode) continue;

      const distFromA = t * travelDist;
      junctions.push({ nodeId: jNode.id, distFromA });

      // Create target node into the passage at 75% of visible range
      const targetDist = passage.range * 0.75;
      if (targetDist >= 1.0) {
        const targetPos: Vec2 = {
          x: junctionPos.x + targetDist * Math.cos(passage.direction),
          y: junctionPos.y + targetDist * Math.sin(passage.direction),
        };
        const nearTarget = this.graph.findNearestNode(targetPos);
        if (!nearTarget || vec2Distance(targetPos, nearTarget.position) > this.minNodeSeparation) {
          {
            const tFeature: DetectedFeature = {
              type: FeatureType.Corner,
              position: targetPos,
              confidence: 0.5,
              heading: passage.direction,
              exit_directions: [passage.direction],
              local_radius: 1.0,
            };
            const tNode = this.createNode(tFeature, samples[passage.sampleIdx].scan);
            if (tNode) {
              const edgeDist = vec2Distance(jNode.position, tNode.position);
              const bearing = normalizeAngle(Math.atan2(
                tNode.position.y - jNode.position.y,
                tNode.position.x - jNode.position.x,
              ));
              const edge = this.graph.createEdge(
                jNode.id, tNode.id, edgeDist, bearing, timestamp, false,
              );
              if (!edge) {
                this.graph.removeNode(tNode.id);
              }
            }
          }
        }
      }
    }

    // Sort junctions by distance from A, then split the A->B edge
    if (junctions.length > 0 && traveledEdgeId) {
      junctions.sort((a, b) => a.distFromA - b.distFromA);

      // Remove the original A->B edge
      const originalEdge = this.graph.getEdge(traveledEdgeId);
      if (originalEdge) {
        this.graph.removeEdge(traveledEdgeId);
      }

      // Build chain: A -> J1 -> J2 -> ... -> B
      const chain = [
        { nodeId: departureNode.id, dist: 0 },
        ...junctions,
        { nodeId: arrivalNodeId, dist: travelDist },
      ];

      for (let i = 0; i < chain.length - 1; i++) {
        const fromId = chain[i].nodeId;
        const toId = chain[i + 1].nodeId;
        const fromNode = this.graph.getNode(fromId);
        const toNode = this.graph.getNode(toId);
        if (!fromNode || !toNode) continue;

        const segDist = vec2Distance(fromNode.position, toNode.position);
        const segBearing = normalizeAngle(Math.atan2(
          toNode.position.y - fromNode.position.y,
          toNode.position.x - fromNode.position.x,
        ));
        const edge = this.graph.createEdge(
          fromId, toId, segDist, segBearing, timestamp, true,
        );
        if (!edge) {
          // Edge creation failed — remove the junction node if it was just created
          // (don't remove departure or arrival nodes)
          if (fromId !== departureNode.id && fromId !== arrivalNodeId) {
            // Check if the junction node has any other edges
            const nodeEdges = this.graph.getNodeEdges(fromId);
            if (nodeEdges.length === 0) {
              this.graph.removeNode(fromId);
            }
          }
        }
      }

      // Verify all junction nodes have at least one edge. Remove orphans.
      for (const j of junctions) {
        const nodeEdges = this.graph.getNodeEdges(j.nodeId);
        if (nodeEdges.length === 0) {
          this.graph.removeNode(j.nodeId);
        }
      }
    } else if (junctions.length > 0) {
      // No traveled edge to split, but we created junction nodes.
      // Connect them to A and B if possible.
      for (const j of junctions) {
        const jNode = this.graph.getNode(j.nodeId);
        if (!jNode) continue;

        // Try connecting to departure node
        const distToA = vec2Distance(jNode.position, departureNode.position);
        const bearingToA = normalizeAngle(Math.atan2(
          departureNode.position.y - jNode.position.y,
          departureNode.position.x - jNode.position.x,
        ));
        const edgeToA = this.graph.createEdge(
          departureNode.id, j.nodeId, distToA, bearingToA, timestamp, true,
        );

        // Try connecting to arrival node
        const distToB = vec2Distance(jNode.position, arrivalNode.position);
        const bearingToB = normalizeAngle(Math.atan2(
          arrivalNode.position.y - jNode.position.y,
          arrivalNode.position.x - jNode.position.x,
        ));
        this.graph.createEdge(
          j.nodeId, arrivalNodeId, distToB, bearingToB, timestamp, true,
        );

        // Verify the junction has at least one edge; remove if orphaned
        const nodeEdges = this.graph.getNodeEdges(j.nodeId);
        if (nodeEdges.length === 0) {
          this.graph.removeNode(j.nodeId);
        }
      }
    }

    this.corridorScanLog = [];
  }

  /**
   * Wall auto-stop: triggers when the robot has been traveling but hasn't
   * made meaningful progress toward its target. Checks if forward clearance
   * is critically low AND the robot hasn't moved much recently.
   * Creates a node at current position and restarts the scan/plan cycle.
   * Returns true if wall-stop was triggered.
   */
  private checkWallStop(scan: LidarScan, pose: Pose2D, _target: Vec2): boolean {
    const ticksInTravel = this.tickCount - this.travelStartTick;
    if (ticksInTravel < 15) return false;

    // Check if robot is making progress toward the target
    const distFromStart = vec2Distance({ x: pose.x, y: pose.y }, { x: this.travelStartPose.x, y: this.travelStartPose.y });
    const expectedProgress = ticksInTravel * 0.03;
    if (distFromStart > expectedProgress) return false;

    // Robot is not making progress — check if there's a wall close ahead
    let minForward = scan.max_range;
    for (let i = 0; i < scan.angles.length; i++) {
      if (Math.abs(scan.angles[i]) <= 30 * Math.PI / 180) {
        if (scan.ranges[i] < minForward) minForward = scan.ranges[i];
      }
    }
    if (minForward > 0.5) return false;

    // Hit a wall mid-travel. Mark the active edge as blocked so the robot
    // won't try this path again. Create a node at the wall-stop position.
    if (this.activeEdgeId) {
      this.graph.invalidateEdge(this.activeEdgeId, scan.timestamp);
    }
    this.edgeManager.cancelActiveEdge();

    // Create a wall-stop node via createNode (runs ±20° clearance adjustment),
    // fall back to createNodeAtPosition if the ring check rejects
    const wallFeature: DetectedFeature = {
      type: FeatureType.DeadEnd,
      position: { x: pose.x, y: pose.y },
      confidence: 0.7,
      heading: pose.theta,
      exit_directions: [normalizeAngle(pose.theta + Math.PI)],
      local_radius: 1.0,
    };
    let wallNode = this.createNode(wallFeature, scan);
    if (!wallNode) {
      const fallback = this.graph.createNodeAtPosition(
        { x: pose.x, y: pose.y },
        FeatureType.DeadEnd,
        1,
        scan.timestamp,
      );
      this.featureDetector.registerKnownFeature({ x: pose.x, y: pose.y });
      this.publisher.publishNode(fallback);
      wallNode = fallback;
    }

    // Create a traveled edge from the departure node to the wall-stop node
    if (this.currentNodeId && this.currentNodeId !== wallNode.id) {
      const bearing = normalizeAngle(Math.atan2(
        pose.y - this.travelStartPose.y,
        pose.x - this.travelStartPose.x,
      ));
      const edge = this.graph.createEdge(
        this.currentNodeId, wallNode.id,
        this.travelAccumulatedDistance, bearing,
        scan.timestamp, true,
      );
      if (edge) {
        this.publisher.publishEdge(edge);
        this.events.emit("edge_traveled", { edge });
      }
    }

    // Set the wall-stop node as current and do a full arrival scan
    this.currentNodeId = wallNode.id;
    this.targetNodeId = null;
    this.targetPosition = null;
    this.activeEdgeId = null;

    this.travelAccumulatedDistance = 0;
    this.corridorScanLog = [];
    this.setPhase(ExplorationPhase.ARRIVING, "wall_stop_node");
    return true;
  }


  private handleStuck(scan: LidarScan, pose: Pose2D): MotorCommand {
    if (this.activeEdgeId) {
      this.graph.invalidateEdge(this.activeEdgeId, scan.timestamp);
    }
    this.edgeManager.cancelActiveEdge();
    this.targetNodeId = null;
    this.targetPosition = null;
    this.activeEdgeId = null;
    this.corridorScanLog = [];


    const nearest = this.graph.findNearestNode({ x: pose.x, y: pose.y });
    if (!nearest || vec2Distance({ x: pose.x, y: pose.y }, nearest.position) > this.minNodeSeparation) {
      const feature: DetectedFeature = {
        type: FeatureType.Junction,
        position: { x: pose.x, y: pose.y },
        confidence: 0.6,
        heading: pose.theta,
        exit_directions: [],
        local_radius: 1.5,
      };
      const node = this.createNode(feature, scan);
      if (node) {
        // Create edge from departure node to stuck node
        if (this.currentNodeId && this.currentNodeId !== node.id) {
          const bearing = normalizeAngle(Math.atan2(
            pose.y - this.travelStartPose.y, pose.x - this.travelStartPose.x,
          ));
          const edge = this.graph.createEdge(
            this.currentNodeId, node.id, this.travelAccumulatedDistance, bearing, scan.timestamp, true,
          );
          if (!edge) {
            this.graph.removeNode(node.id);
          }
        }
        this.currentNodeId = node.id;
      }
    } else {
      this.currentNodeId = nearest.id;
    }

    this.scanTicksRemaining = 5;
    this.setPhase(ExplorationPhase.ARRIVING, "stuck_recovery");
    return this.motor.stop(scan.timestamp);
  }

  // =========================================================================
  // Node Creation
  // =========================================================================

  private createNode(feature: DetectedFeature, scan: LidarScan): NetworkNode | null {
    const activeScan = this.lastScan ?? scan;
    const pose = this.slam.pose;
    let dist = vec2Distance(pose, feature.position);
    if (dist < 0.3) return null;

    // LiDAR wall check: find the range of the closest ray toward the node.
    // If a wall is closer than the proposed node, pull the node back to
    // (wall distance - clearance) so the robot won't path into the wall.
    const angleToNode = normalizeAngle(
      Math.atan2(feature.position.y - pose.y, feature.position.x - pose.x) - pose.theta,
    );
    let closestRayRange = activeScan.max_range;
    let closestRayDiff = Infinity;
    for (let i = 0; i < activeScan.angles.length; i++) {
      const diff = Math.abs(normalizeAngle(activeScan.angles[i] - angleToNode));
      if (diff < closestRayDiff) {
        closestRayDiff = diff;
        closestRayRange = activeScan.ranges[i];
      }
    }
    if (closestRayRange < dist) {
      const wallClearance = 0.5;
      const pulledDist = closestRayRange - wallClearance;
      if (pulledDist < 1.0) return null;
      const absAngle = angleToNode + pose.theta;
      feature.position.x = pose.x + pulledDist * Math.cos(absAngle);
      feature.position.y = pose.y + pulledDist * Math.sin(absAngle);
      dist = pulledDist;
    }

    // ±10° clearance adjustment: find the direction within ±20° of the target
    // that maximizes wall clearance on both sides (LiDAR-based).
    if (dist > 0.5) {
      const los = this.checkLineOfSight(activeScan, pose, feature.position.x, feature.position.y);
      if (los.clear) {
        const directAngle = Math.atan2(feature.position.y - pose.y, feature.position.x - pose.x);
        const diff = normalizeAngle(los.bestAngle - directAngle);
        if (Math.abs(diff) > 0.01) {
          const adjustedAngle = directAngle + diff;
          const newX = pose.x + dist * Math.cos(adjustedAngle);
          const newY = pose.y + dist * Math.sin(adjustedAngle);
          // Verify the adjusted position is reachable via direct LiDAR ray
          if (this.hasDirectLineOfSight(activeScan, pose, newX, newY, dist)) {
            feature.position.x = newX;
            feature.position.y = newY;
          }
        }
      }
    }

    const node = this.graph.createNode(feature, "", scan.timestamp);
    // Do NOT create a node map here — the robot hasn't been to this position.
    // SLAM data is only captured when the robot physically arrives at a node
    // (in doArriving) or while traveling along an edge (in EdgeMapManager).
    this.featureDetector.registerKnownFeature(feature.position);
    this.publisher.publishNode(node);
    this.events.emit("node_created", { node, feature });
    return node;
  }

  // =========================================================================
  // Grid Frontier Detection (fallback)
  // =========================================================================

  private findGridFrontier(pose: Pose2D): Vec2 | null {
    const resolution = this.slam.grid.config.resolution;
    const maxSearchRadius = 30.0;
    const stepSize = resolution;

    for (let radius = 2.0; radius < maxSearchRadius; radius += stepSize) {
      const angleSteps = Math.max(8, Math.ceil(2 * Math.PI * radius / stepSize));
      for (let i = 0; i < angleSteps; i++) {
        const angle = (i / angleSteps) * 2 * Math.PI;
        const px = pose.x + radius * Math.cos(angle);
        const py = pose.y + radius * Math.sin(angle);

        if (this.slam.isPositionFree(px, py)) {
          const offsets = [
            { dx: resolution, dy: 0 },
            { dx: -resolution, dy: 0 },
            { dx: 0, dy: resolution },
            { dx: 0, dy: -resolution },
          ];
          for (const off of offsets) {
            if (this.slam.isPositionUnknown(px + off.dx, py + off.dy)) {
              if (!this.hasLineOfSight(pose.x, pose.y, px, py)) continue;
              return { x: px, y: py };
            }
          }
        }
      }
    }
    return null;
  }

  // =========================================================================
  // Merge Protocol — Robot-Side Discovery
  // =========================================================================

  /**
   * Attempt to align with external robot graphs by detecting overlapping
   * nodes. This is called periodically during scan phases.
   *
   * The robot compares its own node summaries against nodes it has received
   * from other robots via the Gun network. If enough matching candidates
   * are found, it computes a coordinate alignment transform and sets it
   * on the subscriber so future external nodes are placed correctly.
   */
  private attemptMergeAlignment(): void {
    if (this.alignmentEstablished) return;

    // Collect external nodes received via the network
    // (the subscriber adds them to the graph with discovered_by != this.id)
    const myNodes = this.graph.getAllNodes().filter(n => n.discovered_by === this.id);
    const externalNodes = this.graph.getAllNodes().filter(n => n.discovered_by !== this.id);

    if (myNodes.length < 3 || externalNodes.length < 3) return;

    // Generate summaries for candidate detection
    const myEdges = this.graph.getAllEdges().filter(e => e.discovered_by === this.id);
    const mySummaries = this.candidateDetector.generateSummaries(myNodes, myEdges);

    const extEdges = this.graph.getAllEdges().filter(e => e.discovered_by !== this.id);
    const extSummaries = this.candidateDetector.generateSummaries(externalNodes, extEdges);

    // Find candidate matches
    const candidates = this.candidateDetector.findCandidates(mySummaries, extSummaries);
    if (candidates.length < 2) return;

    // Build anchor pairs from the top candidates
    const anchors = candidates.slice(0, Math.min(candidates.length, 5)).map(c => {
      const myNode = this.graph.getNode(c.node_a_id);
      const extNode = this.graph.getNode(c.node_b_id);
      return {
        a_position: myNode!.position,
        b_position: extNode!.position,
        node_a_id: c.node_a_id,
        node_b_id: c.node_b_id,
      };
    });

    // Compute alignment transform
    const transform = this.graphAligner.align(anchors);
    if (transform.quality === "tentative") return;

    // Set the alignment on the subscriber
    this.subscriber.setCoordinateAlignment(
      transform.translation,
      transform.rotation,
    );
    this.alignmentEstablished = true;

    // Now that alignment is established, flush buffered external nodes
    this.subscriber.flushPendingNodes();
  }

  /**
   * Check LOS and find the best clear path within ±10° of the target.
   * Returns { clear, bestAngle } — the angle that maximizes wall clearance.
   * If clear is false, no path exists. If clear is true, bestAngle is the
   * direction that has the most room on both sides (avoids corner clipping).
   */
  private checkLineOfSight(scan: LidarScan, pose: Pose2D, toX: number, toY: number): { clear: boolean; bestAngle: number } {
    const dx = toX - pose.x;
    const dy = toY - pose.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist < 0.1) return { clear: true, bestAngle: 0 };
    if (dist > scan.max_range) return { clear: false, bestAngle: 0 };

    const angleToTarget = normalizeAngle(Math.atan2(dy, dx) - pose.theta);
    // Search ±20° for the ray that clears the target AND maximizes the
    // minimum distance to walls on BOTH sides.
    // Rays outside the ±20° window are treated as walls (range = 0).
    const searchHalf = 20 * Math.PI / 180;

    // Build set of ray indices in the window
    const windowIndices: number[] = [];
    for (let i = 0; i < scan.angles.length; i++) {
      if (Math.abs(normalizeAngle(scan.angles[i] - angleToTarget)) <= searchHalf) {
        windowIndices.push(i);
      }
    }
    if (windowIndices.length === 0) return { clear: false, bestAngle: 0 };

    const windowSet = new Set(windowIndices);
    let bestIdx = -1;
    let bestScore = -1;

    for (const i of windowIndices) {
      if (scan.ranges[i] < dist) continue;

      // Find the minimum distance to the left wall and right wall
      // by scanning outward from this ray in both directions.
      // Rays outside the ±20° window count as distance 0 (unknown = wall).
      let leftWallDist = 0;
      for (let j = 1; j <= 10; j++) {
        const li = i + j;
        if (!windowSet.has(li)) break; // Edge of window = wall
        if (scan.ranges[li] < dist * 0.5) break; // Hit a wall
        leftWallDist = j; // Still clear
      }

      let rightWallDist = 0;
      for (let j = 1; j <= 10; j++) {
        const ri = i - j;
        if (!windowSet.has(ri)) break; // Edge of window = wall
        if (scan.ranges[ri] < dist * 0.5) break; // Hit a wall
        rightWallDist = j; // Still clear
      }

      // Score = minimum of left and right clearance (maximize the tightest side)
      const score = Math.min(leftWallDist, rightWallDist);
      if (score > bestScore) {
        bestScore = score;
        bestIdx = i;
      }
    }

    if (bestIdx < 0) return { clear: false, bestAngle: 0 };

    return { clear: true, bestAngle: scan.angles[bestIdx] + pose.theta };
  }

  private hasLineOfSight(fromX: number, fromY: number, toX: number, toY: number): boolean {
    const grid = this.slam.grid;
    const { config } = grid;
    return gridHasLineOfSight(
      fromX, fromY, toX, toY,
      config.origin_x, config.origin_y,
      config.resolution,
      config.width, config.height,
      (gx: number, gy: number) => {
        if (!gridInBounds(gx, gy, config)) return true;
        return grid.cells[gy * config.width + gx] > 0.5;
      },
    );
  }

  /**
   * Direct line-of-sight: single ray closest to the target, must reach full distance.
   * No ±20° search. Used for EDGE creation where the edge is a straight line.
   */
  private hasDirectLineOfSight(scan: LidarScan, pose: Pose2D, toX: number, toY: number, dist: number): boolean {
    const dx = toX - pose.x;
    const dy = toY - pose.y;
    if (dist < 0.1) return true;
    if (dist > scan.max_range) return false;

    const angleToTarget = normalizeAngle(Math.atan2(dy, dx) - pose.theta);
    let bestIdx = -1;
    let bestDiff = Infinity;
    for (let i = 0; i < scan.angles.length; i++) {
      const diff = Math.abs(normalizeAngle(scan.angles[i] - angleToTarget));
      if (diff < bestDiff) { bestDiff = diff; bestIdx = i; }
    }
    if (bestIdx < 0) return false;
    return scan.ranges[bestIdx] >= dist;
  }

  // =========================================================================
  // Public Accessors
  // =========================================================================

  get pose(): Pose2D { return this.slam.pose; }
  get slamGrid() { return this.slam.grid; }
  get exploredCellCount(): number { return this.slam.getExploredCellCount(); }
  get currentStatus(): string { return this.phase; }
  get ticks(): number { return this.tickCount; }

  getTrail(): Vec2[] { return this.trail; }
  getNodeCount(): number { return this.graph.nodeCount; }
  getEdgeCount(): number { return this.graph.edgeCount; }
  getAllNodes(): NetworkNode[] { return this.graph.getAllNodes(); }
  getAllEdges(): NetworkEdge[] { return this.graph.getAllEdges(); }
  getCurrentNodeId(): string | null { return this.currentNodeId; }

  getNodeLocalMap(nodeId: string) { return this.nodeManager.getNodeMap(nodeId); }
  getEdgeCorridorMap(edgeId: string) { return this.edgeManager.getCorridorMapForEdge(edgeId); }

  getRobotInfo(): {
    status: string;
    slamPose: Pose2D;
    targetPosition: Vec2 | null;
    targetNodeId: string | null;
    currentNodeId: string | null;
    activeEdgeId: string | null;
    tickCount: number;
    nodesDiscovered: number;
    edgesDiscovered: number;
    behaviorMode: BehaviorMode;
  } {
    return {
      status: this.phase,
      slamPose: { ...this.lastPose },
      targetPosition: this.targetPosition ? { ...this.targetPosition } : null,
      targetNodeId: this.targetNodeId,
      currentNodeId: this.currentNodeId,
      activeEdgeId: this.activeEdgeId,
      tickCount: this.tickCount,
      nodesDiscovered: this.graph.nodeCount,
      edgesDiscovered: this.graph.edgeCount,
      behaviorMode: this.behaviorState.mode,
    };
  }

  setBehaviorMode(mode: BehaviorMode): void {
    this.behaviorState.mode = mode;
  }

  getBehaviorMode(): BehaviorMode {
    return this.behaviorState.mode;
  }

  setManualTarget(position: Vec2): void {
    if (!this.currentNodeId) return;

    // Find the nearest known node to the click position
    const targetNode = this.graph.findNearestNode(position);
    if (!targetNode) return;
    if (targetNode.id === this.currentNodeId) {
      this.scanTicksRemaining = 5;
      this.setPhase(ExplorationPhase.SCANNING, "manual_rescan");
      return;
    }

    // Compute full path through the graph via BFS (avoids blocked edges)
    const path = this.graph.findPath(this.currentNodeId, targetNode.id);
    if (!path || path.length === 0) return;

    // Queue the path — each hop will be consumed in doChoosing after a
    // full ARRIVING->SCANNING->CHOOSING cycle at each intermediate node.
    // This means SLAM data and edge visibility are updated at every stop.
    this.manualPathQueue = path;

    // If currently in a scanning/choosing phase, the queue will be picked
    // up on the next doChoosing call. If traveling, interrupt and restart.
    if (this.phase === ExplorationPhase.TRAVELING) {
      this.edgeManager.cancelActiveEdge();
      this.scanTicksRemaining = 1;
      this.setPhase(ExplorationPhase.SCANNING, "manual_target_interrupt");
    }
  }

  connectToNetwork(pub: IGraphNetwork, sub: IGraphNetworkSubscriber): void {
    this.publisher.connect(pub);
    this.subscriber.connect(sub);
  }

  reset(): void {
    this.slam.initialize();
    this.featureDetector.reset();
    this.edgeManager.reset();
    this.nodeManager.reset();
    this.graph.reset();
    this.publisher.reset();
    this.subscriber.reset();
    this.motor.reset();
    this.phase = ExplorationPhase.ARRIVING;
    this.currentNodeId = null;
    this.targetNodeId = null;
    this.targetPosition = null;
    this.activeEdgeId = null;

    this.scanTicksRemaining = 0;
    this.lastPose = { x: 0, y: 0, theta: 0 };
    this.tickCount = 0;
    this.trail = [];
    this.lastScan = null;
    this.initialized = false;
    this.travelStartTick = 0;
    this.travelAccumulatedDistance = 0;
    this.travelStartPose = { x: 0, y: 0, theta: 0 };
    this.corridorScanLog = [];
    this.lastCorridorSamplePos = { x: 0, y: 0 };
    this.manualPathQueue = [];
    this.behaviorState = createBehaviorState();
    this.alignmentEstablished = false;
  }

  // =========================================================================
  // Private Helpers
  // =========================================================================

  private setPhase(phase: ExplorationPhase, reason: string): void {
    this.phase = phase;
    this.events.emit("phase_changed", { phase, reason });
  }
}
