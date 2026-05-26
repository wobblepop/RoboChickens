// ===========================================================================
// Server ↔ Robot Communication Protocol
// Defines the message format for the data stream between the world server
// and each independent robot agent.
// ===========================================================================

import type {
  LidarScan,
  OdometryReading,
  MotorCommand,
  LidarConfig,
  OdometryConfig,
  RobotPhysicsConfig,
  Vec2,
} from "./types";

// ---------------------------------------------------------------------------
// Server → Robot Messages
// ---------------------------------------------------------------------------

/** Sent once when a robot first connects. Contains its sensor configuration. */
export interface ServerHandshakeMessage {
  type: "handshake";
  robot_id: string;
  lidar_config: LidarConfig;
  odometry_config: OdometryConfig;
  physics_config: RobotPhysicsConfig;
  /** Starting position is NOT given — robot starts at SLAM origin (0,0,0) */
}

/** Sent every tick with the robot's sensor readings. */
export interface ServerSensorMessage {
  type: "sensors";
  timestamp: number;
  tick: number;
  lidar: LidarScan;
  odometry: OdometryReading;
}

/** Sent when the simulation is paused. */
export interface ServerPauseMessage {
  type: "pause";
}

/** Sent when the simulation is resumed. */
export interface ServerResumeMessage {
  type: "resume";
}

/** Sent when the simulation resets (robot should reset all internal state). */
export interface ServerResetMessage {
  type: "reset";
  robot_id: string;
  lidar_config: LidarConfig;
  odometry_config: OdometryConfig;
  physics_config: RobotPhysicsConfig;
}

export type ServerMessage =
  | ServerHandshakeMessage
  | ServerSensorMessage
  | ServerPauseMessage
  | ServerResumeMessage
  | ServerResetMessage;

// ---------------------------------------------------------------------------
// Robot → Server Messages
// ---------------------------------------------------------------------------

/** Robot's motor command for the current tick. */
export interface RobotMotorMessage {
  type: "motor";
  command: MotorCommand;
}

/** Robot reports its internal state for visualization (optional, for debug). */
export interface RobotStateReport {
  type: "state_report";
  robot_id: string;
  /** Robot's SLAM pose estimate (in its own coordinate frame) */
  slam_pose: { x: number; y: number; theta: number };
  /** Number of cells the robot considers explored */
  cells_explored: number;
  /** Robot's current status */
  status: "exploring" | "navigating" | "idle" | "lost";
  /** Network graph nodes this robot knows about */
  node_count: number;
  /** Network graph edges this robot knows about */
  edge_count: number;
}

export type RobotMessage =
  | RobotMotorMessage
  | RobotStateReport;

// ---------------------------------------------------------------------------
// Visualization Protocol (Server → Browser)
// ---------------------------------------------------------------------------

export interface VizInitMessage {
  type: "init";
  data: {
    grid: string[][];
    width: number;
    height: number;
    robots: {
      id: string;
      x: number;
      y: number;
      heading: number;
      status: string;
      cellsExplored: number;
      trail: Vec2[];
    }[];
  };
}

export interface VizFrameMessage {
  type: "frame";
  data: {
    tick: number;
    newlyExploredCells: string[];
    robots: {
      id: string;
      x: number;
      y: number;
      heading: number;
      status: string;
      cellsExplored: number;
      trail: Vec2[];
    }[];
    graph: {
      nodes: {
        id: string;
        x: number;
        y: number;
        type: string;
        tier: string;
        edgeCount: number;
        isFrontier: boolean;
        explored?: boolean;
      }[];
      edges: {
        id: string;
        sourceId: string;
        targetId: string;
        tier: string;
        distance: number;
        status: string;
        traveled?: boolean;
      }[];
    };
    metrics: {
      coverage: number;
      totalTraversable: number;
      totalExplored: number;
      nodeCount: number;
      edgeCount: number;
      tickCount: number;
    };
    complete: boolean;
  };
}

export interface VizCompleteMessage {
  type: "complete";
  data: VizFrameMessage["data"];
}

export interface VizNodeInfoMessage {
  type: "nodeInfo";
  data: {
    id: string;
    type: string;
    x: number;
    y: number;
    confidence: number;
    tier: string;
    observationCount: number;
    discoveredBy: string;
    discoveredAt: number;
    createdAt?: number;
    lastObserved?: number;
    stabilityClass?: string;
    explored?: boolean;
    exploredBy?: string[];
    isFrontier: boolean;
    edgeCount: number;
    edges: {
      id: string;
      targetNodeId: string;
      distance: number;
      tier: string;
      traveled?: boolean;
    }[];
    localMap: { x: number; y: number; state: string; confidence: number }[];
    localMapSize: number;
  };
}

export interface VizEdgeInfoMessage {
  type: "edgeInfo";
  data: {
    id: string;
    type: string;
    sourceNodeId: string;
    targetNodeId: string;
    distance: number;
    confidence: number;
    tier: string;
    status: string;
    traveled?: boolean;
    bidirectional: boolean;
    observationCount: number;
    corridorMap: { x: number; y: number; state: string; confidence: number }[];
    corridorMapSize: number;
  };
}

export interface VizCellEditedMessage {
  type: "cellEdited";
  x: number;
  y: number;
  cellType: string;
}

export type VizServerMessage =
  | VizInitMessage
  | VizFrameMessage
  | VizCompleteMessage
  | VizNodeInfoMessage
  | VizEdgeInfoMessage
  | VizCellEditedMessage;

// ---------------------------------------------------------------------------
// Browser → Server Messages
// ---------------------------------------------------------------------------

export interface VizPlayMessage { type: "play"; }
export interface VizPauseMessage { type: "pause"; }
export interface VizStepMessage { type: "step"; }
export interface VizResetMessage { type: "reset"; map?: string; }
export interface VizSpeedMessage { type: "speed"; value: number; }
export interface VizTargetMessage { type: "target"; x: number; y: number; robotId?: string; }
export interface VizNodeInfoRequest { type: "nodeInfo"; nodeId: string; }
export interface VizEdgeInfoRequest { type: "edgeInfo"; edgeId: string; }
export interface VizEditCellMessage { type: "editCell"; x: number; y: number; cellType: string; }
export interface VizLoadMapMessage { type: "loadMap"; map: string; }
export interface VizValidateMessage { type: "validate"; }

export interface VizRobotInfoRequest { type: "robotInfo"; robotId: string; }

export type VizClientMessage =
  | VizPlayMessage
  | VizPauseMessage
  | VizStepMessage
  | VizResetMessage
  | VizSpeedMessage
  | VizTargetMessage
  | VizNodeInfoRequest
  | VizEdgeInfoRequest
  | VizRobotInfoRequest
  | VizEditCellMessage
  | VizLoadMapMessage
  | VizValidateMessage;
