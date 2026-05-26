// ===========================================================================
// World Server
// The main server that runs the simulation. Manages:
// - Ground truth world (the warehouse map)
// - Physical robot states (positions, collisions)
// - Sensor simulation (LiDAR + odometry per robot)
// - Robot agent instances (each running independently)
// - Visualization WebSocket (to browser)
// ===========================================================================

import * as http from "http";
import * as fs from "fs";
import * as path from "path";
import { WebSocketServer, WebSocket } from "ws";

import type {
  SimulationConfig,
  Vec2,
  NetworkNode,
  NetworkEdge,
} from "../shared/types";
import { DEFAULT_SIMULATION_CONFIG, CellType, BehaviorMode, BEHAVIOR_MODE_LABELS } from "../shared/types";
import type {
  VizInitMessage,
  VizFrameMessage,
  VizClientMessage,
  VizNodeInfoMessage,
  VizEdgeInfoMessage,
} from "../shared/protocol";

import { GroundTruthWorld, loadMap, listMaps } from "./ground-truth-map";
import { PhysicsEngine } from "./physics-engine";
import { LidarSimulator } from "./lidar-simulator";
import { OdometrySimulator } from "./odometry-simulator";
import { RobotConnection } from "./robot-connection";
import { InProcessNetwork } from "../shared/gun-schema";

// ---------------------------------------------------------------------------
// World Server
// ---------------------------------------------------------------------------

export class WorldServer {
  private config: SimulationConfig;
  private world!: GroundTruthWorld;
  private physics!: PhysicsEngine;
  private lidarSim!: LidarSimulator;
  private odomSim!: OdometrySimulator;
  private robots: RobotConnection[] = [];
  private sharedNetwork: InProcessNetwork = new InProcessNetwork();

  private tickCount: number = 0;
  private tickTimer: ReturnType<typeof setTimeout> | null = null;
  private paused: boolean = true;
  private ticksPerSecond: number = 10;
  private ticking: boolean = false;

  private httpServer: http.Server | null = null;
  private wss: WebSocketServer | null = null;
  private clients: Set<WebSocket> = new Set();
  private driftCorrectionLog: any[] = [];

  private mapsDir: string;
  private currentMapFile: string;
  private authPassword: string;

  constructor(config: SimulationConfig = DEFAULT_SIMULATION_CONFIG) {
    this.config = config;
    this.mapsDir = path.resolve(__dirname, "../../warehouse-maps");
    this.currentMapFile = "map-05-two-robots.txt";
    this.authPassword = process.env.SLAM_PASSWORD || "4hadway";
  }

  private checkBasicAuth(req: http.IncomingMessage): boolean {
    const header = req.headers.authorization;
    if (!header || !header.startsWith("Basic ")) return false;
    const decoded = Buffer.from(header.slice(6), "base64").toString("utf-8");
    const password = decoded.includes(":") ? decoded.split(":").slice(1).join(":") : decoded;
    return password === this.authPassword;
  }

  private sendAuthChallenge(res: http.ServerResponse): void {
    res.writeHead(401, {
      "WWW-Authenticate": 'Basic realm="SLAM Explorer"',
      "Content-Type": "text/plain",
    });
    res.end("Unauthorized");
  }

  /**
   * Initialize the world with a map file.
   */
  initialize(mapFile?: string): void {
    if (mapFile) this.currentMapFile = mapFile;

    const mapPath = path.resolve(this.mapsDir, this.currentMapFile);
    const map = loadMap(mapPath);
    this.world = new GroundTruthWorld(map);

    this.physics = new PhysicsEngine(this.world, this.config.robot_physics, this.config.tick_interval_ms);
    this.lidarSim = new LidarSimulator(this.config.lidar);
    this.odomSim = new OdometrySimulator(this.config.odometry);

    // Create robot connections for each robot start position
    this.robots = [];
    this.sharedNetwork = new InProcessNetwork();
    for (const start of map.robot_starts) {
      const conn = new RobotConnection(
        start.id,
        this.world,
        this.physics,
        this.lidarSim,
        this.odomSim,
        start.x,
        start.y,
        this.config,
      );
      this.robots.push(conn);
    }

    // Connect all robots to the shared network so they can see each other
    for (const conn of this.robots) {
      conn.agent.connectToNetwork(this.sharedNetwork, this.sharedNetwork);
    }

    // Subscribe to drift correction events from all robots
    this.driftCorrectionLog = [];
    for (const conn of this.robots) {
      conn.agent.events.on("drift_correction", (report) => {
        this.driftCorrectionLog.push({
          tick: this.tickCount,
          robotId: conn.id,
          ...report,
        });
        // Keep last 50 entries
        if (this.driftCorrectionLog.length > 50) this.driftCorrectionLog.shift();
      });
    }

    this.tickCount = 0;
    this.paused = true;
  }

  /**
   * Start the HTTP + WebSocket server.
   */
  startServer(port: number = 3000): void {
    this.httpServer = http.createServer((req, res) => this.handleHttp(req, res));
    this.wss = new WebSocketServer({
      server: this.httpServer,
      verifyClient: (info, cb) => {
        if (!this.checkBasicAuth(info.req)) {
          cb(false, 401, "Unauthorized");
          return;
        }
        const origin = info.origin || info.req.headers.origin;
        const host = info.req.headers.host;
        if (origin && host && !origin.includes(host)) {
          cb(false, 403, "Forbidden");
          return;
        }
        cb(true);
      },
      maxPayload: 64 * 1024,
    });

    this.wss.on("connection", (ws) => {
      this.clients.add(ws);
      ws.on("message", (data) => this.handleVizMessage(ws, data.toString()));
      ws.on("close", () => this.clients.delete(ws));

      // Send init message
      this.sendInit(ws);
    });

    this.httpServer.listen(port, "0.0.0.0", () => {
      console.log(`World Server running at http://localhost:${port}`);
      console.log(`Map: ${this.currentMapFile} | Robots: ${this.robots.length}`);
    });
  }

  // -------------------------------------------------------------------------
  // Simulation Loop
  // -------------------------------------------------------------------------

  private scheduleTick(): void {
    if (this.paused || this.ticking) return;
    const interval = Math.max(10, Math.floor(1000 / this.ticksPerSecond));
    this.tickTimer = setTimeout(() => this.runTick(), interval);
  }

  private runTick(): void {
    if (this.paused) return;
    this.ticking = true;

    this.tickCount++;
    const timestamp = this.tickCount * this.config.tick_interval_ms;

    // Tick each robot
    for (const robot of this.robots) {
      robot.tick(timestamp);
    }

    // Send frame to visualization clients
    this.broadcastFrame();

    this.ticking = false;
    this.scheduleTick();
  }

  private singleStep(): void {
    this.tickCount++;
    const timestamp = this.tickCount * this.config.tick_interval_ms;
    for (const robot of this.robots) {
      robot.tick(timestamp);
    }
    this.broadcastFrame();
  }

  // -------------------------------------------------------------------------
  // Visualization Messages
  // -------------------------------------------------------------------------

  private sendInit(ws: WebSocket): void {
    const msg: any = {
      type: "init",
      behaviorModes: BEHAVIOR_MODE_LABELS,
      robotModes: Object.fromEntries(this.robots.map(r => [r.id, r.agent.getBehaviorMode()])),
      data: {
        grid: this.world.getGridStrings(),
        width: this.world.width,
        height: this.world.height,
        robots: this.robots.map((r) => {
          const slamPose = r.agent.pose;
          const worldPos = r.slamToWorld(slamPose.x, slamPose.y);
          const actual = r.getActualPose();
          const slamTrail = r.agent.getTrail();
          const worldTrail = slamTrail.map((p) => r.slamToWorld(p.x, p.y));
          return {
            id: r.id,
            x: actual.x,
            y: actual.y,
            heading: actual.theta,
            believedX: worldPos.x,
            believedY: worldPos.y,
            believedHeading: slamPose.theta,
            actualX: actual.x,
            actualY: actual.y,
            actualHeading: actual.theta,
            slamDrift: r.getCumulativeDrift(),
            positionError: r.getPositionError(),
            status: r.agent.currentStatus,
            cellsExplored: r.getExploredCellCount(),
            trail: worldTrail,
          };
        }),
      },
    };
    ws.send(JSON.stringify(msg));
  }

  private broadcastFrame(): void {
    // Collect newly explored cells from all robots
    const newlyExplored: string[] = [];
    for (const robot of this.robots) {
      newlyExplored.push(...robot.getNewlyExploredCells());
    }

    // Collect graph data from all robots, transforming SLAM coords → world coords
    const vizNodes: VizFrameMessage["data"]["graph"]["nodes"] = [];
    const vizEdges: VizFrameMessage["data"]["graph"]["edges"] = [];
    const seenNodeIds = new Set<string>();
    const seenEdgeIds = new Set<string>();

    for (const robot of this.robots) {
      for (const node of robot.agent.getAllNodes()) {
        if (seenNodeIds.has(node.id)) continue;
        seenNodeIds.add(node.id);
        const worldPos = robot.slamToWorld(node.position.x, node.position.y);
        vizNodes.push({
          id: node.id,
          x: worldPos.x,
          y: worldPos.y,
          type: node.type,
          tier: node.tier,
          edgeCount: node.exit_count,
          isFrontier: false,
          explored: node.explored,
        });
      }
      for (const edge of robot.agent.getAllEdges()) {
        if (seenEdgeIds.has(edge.id)) continue;
        seenEdgeIds.add(edge.id);
        vizEdges.push({
          id: edge.id,
          sourceId: edge.source_node,
          targetId: edge.target_node,
          tier: edge.tier,
          distance: edge.distance,
          status: edge.traversability?.status ?? "open",
          traveled: edge.traveled,
        });
      }
    }

    // Compute coverage (deduplicated across robots)
    const allExploredCells = new Set<string>();
    for (const robot of this.robots) {
      for (const cell of robot.getNewlyExploredCells()) {
        allExploredCells.add(cell);
      }
    }
    const totalExplored = this.robots.reduce((sum, r) => sum + r.getExploredCellCount(), 0);
    const totalTraversable = this.world.getTraversableCount();

    // Transform robot positions and trails from SLAM coords to world coords.
    // Both must use the same coordinate source (SLAM) to avoid desync.
    const robotViz = this.robots.map((r) => {
      const slamPose = r.agent.pose;
      const worldPos = r.slamToWorld(slamPose.x, slamPose.y);
      const actual = r.getActualPose();
      const slamTrail = r.agent.getTrail();
      const worldTrail = slamTrail.map((p) => r.slamToWorld(p.x, p.y));
      const dx = worldPos.x - actual.x;
      const dy = worldPos.y - actual.y;
      return {
        id: r.id,
        x: worldPos.x,
        y: worldPos.y,
        heading: slamPose.theta,
        actualX: actual.x,
        actualY: actual.y,
        actualHeading: actual.theta,
        slamDrift: Math.sqrt(dx * dx + dy * dy),
        status: r.agent.currentStatus,
        cellsExplored: r.getExploredCellCount(),
        trail: worldTrail,
      };
    });

    const msg: VizFrameMessage = {
      type: "frame",
      data: {
        tick: this.tickCount,
        newlyExploredCells: newlyExplored,
        robots: robotViz,
        graph: {
          nodes: vizNodes,
          edges: vizEdges,
        },
        metrics: {
          coverage: totalTraversable > 0 ? totalExplored / totalTraversable : 0,
          totalTraversable,
          totalExplored,
          nodeCount: vizNodes.length,
          edgeCount: vizEdges.length,
          tickCount: this.tickCount,
        },
        complete: false,
      },
    };

    // Mark frontier nodes
    for (const robot of this.robots) {
      const frontiers = robot.agent.graph.getFrontierNodes();
      for (const f of frontiers) {
        const vizNode = msg.data.graph.nodes.find((n) => n.id === f.id);
        if (vizNode) vizNode.isFrontier = true;
      }
    }

    const jsonMsg = JSON.stringify(msg);
    for (const client of this.clients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(jsonMsg);
      }
    }
  }

  /**
   * Edit a cell in the ground truth map. This modifies the actual world —
   * robots will immediately see the change via LiDAR (walls block rays,
   * erased walls open up new line-of-sight).
   */
  private editCell(x: number, y: number, cellType: string): void {
    if (!this.world.inBounds(x, y)) return;

    let newCell: CellType;
    switch (cellType) {
      case "wall": newCell = CellType.Wall; break;
      case "obstacle": newCell = CellType.Obstacle; break;
      case "pillar": newCell = CellType.Pillar; break;
      case "floor": newCell = CellType.Floor; break;
      default: newCell = CellType.Floor; break;
    }

    this.world.map.cells[y][x] = newCell;
    // Invalidate traversable count cache
    (this.world as any).traversableCount = -1;
  }

  /**
   * Broadcast a message to all connected visualization clients.
   */
  /**
   * Check if robots have explored overlapping areas and establish
   * coordinate alignment so they can correctly place each other's nodes.
   *
   * Each robot's SLAM frame has origin at its starting position.
   * The offset between Robot A's SLAM frame and Robot B's SLAM frame is:
   *   B_slam = A_slam + (A_startWorld - B_startWorld)
   *
   * We set this on each robot's subscriber so incoming external nodes
   * are correctly transformed into the local SLAM frame.
   */
  private mergedPairs: Set<string> = new Set();

  private checkRobotMerges(): void {
    for (let i = 0; i < this.robots.length; i++) {
      for (let j = i + 1; j < this.robots.length; j++) {
        const rA = this.robots[i];
        const rB = this.robots[j];
        const pairKey = `${rA.id}:${rB.id}`;
        if (this.mergedPairs.has(pairKey)) continue;

        // Check if either robot has enough nodes to merge
        if (rA.agent.getNodeCount() < 3 || rB.agent.getNodeCount() < 3) continue;

        // Compute the SLAM-to-SLAM offset
        // Robot A's world start: slamToWorld(0,0) for A
        // Robot B's world start: slamToWorld(0,0) for B
        // To place B's SLAM node in A's frame: nodeInA = nodeInB + (B_world_start - A_world_start)
        const aWorld = rA.slamToWorld(0, 0);
        const bWorld = rB.slamToWorld(0, 0);

        // Offset to transform B's SLAM coords into A's SLAM frame
        const bToAOffset = { x: bWorld.x - aWorld.x, y: bWorld.y - aWorld.y };
        // Offset to transform A's SLAM coords into B's SLAM frame
        const aToBOffset = { x: aWorld.x - bWorld.x, y: aWorld.y - bWorld.y };

        // Set alignment on both subscribers (rotation = 0 since both start at heading 0)
        (rA.agent as any).subscriber.setCoordinateAlignment(bToAOffset, 0);
        (rB.agent as any).subscriber.setCoordinateAlignment(aToBOffset, 0);

        this.mergedPairs.add(pairKey);
      }
    }
  }

  private broadcastToViz(msg: any): void {
    const jsonMsg = JSON.stringify(msg);
    for (const client of this.clients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(jsonMsg);
      }
    }
  }

  // -------------------------------------------------------------------------
  // HTTP Handler
  // -------------------------------------------------------------------------

  private handleHttp(req: http.IncomingMessage, res: http.ServerResponse): void {
    if (!this.checkBasicAuth(req)) {
      this.sendAuthChallenge(res);
      return;
    }

    const url = req.url ?? "/";

    const securityHeaders: Record<string, string> = {
      "X-Content-Type-Options": "nosniff",
      "X-Frame-Options": "DENY",
      "Referrer-Policy": "strict-origin-when-cross-origin",
      "Content-Security-Policy": "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self' ws: wss:",
    };

    if (url === "/maps") {
      const maps = listMaps(this.mapsDir);
      res.writeHead(200, { "Content-Type": "application/json", ...securityHeaders });
      res.end(JSON.stringify({ maps, current: this.currentMapFile }));
      return;
    }

    // Serve static files from public/
    const publicDir = path.resolve(__dirname, "../../public");
    let filePath: string;
    if (url === "/" || url === "/index.html") {
      filePath = path.resolve(publicDir, "index.html");
    } else {
      const safePath = decodeURIComponent(url).replace(/\.\./g, "").replace(/^\/+/, "");
      filePath = path.resolve(publicDir, safePath);
    }

    if (!filePath.startsWith(publicDir + path.sep) && filePath !== publicDir) {
      res.writeHead(403, securityHeaders);
      res.end("Forbidden");
      return;
    }

    const ext = path.extname(filePath);
    const mimeTypes: Record<string, string> = {
      ".html": "text/html",
      ".js": "application/javascript",
      ".css": "text/css",
      ".json": "application/json",
      ".png": "image/png",
      ".bmp": "image/bmp",
    };

    fs.readFile(filePath, (err, data) => {
      if (err) {
        res.writeHead(404, securityHeaders);
        res.end("Not found");
        return;
      }
      res.writeHead(200, { "Content-Type": mimeTypes[ext] ?? "text/plain", ...securityHeaders });
      res.end(data);
    });
  }

  // -------------------------------------------------------------------------
  // Viz Client Message Handler
  // -------------------------------------------------------------------------

  private handleVizMessage(ws: WebSocket, raw: string): void {
    let msg: VizClientMessage;
    try {
      msg = JSON.parse(raw);
    } catch {
      return;
    }

    try {
    switch (msg.type) {
      case "play":
        this.paused = false;
        this.scheduleTick();
        break;

      case "pause":
        this.paused = true;
        if (this.tickTimer) {
          clearTimeout(this.tickTimer);
          this.tickTimer = null;
        }
        break;

      case "step":
        this.singleStep();
        break;

      case "speed":
        this.ticksPerSecond = Math.max(1, Math.min(50, msg.value));
        break;

      case "reset":
      case "loadMap":
        this.paused = true;
        if (this.tickTimer) {
          clearTimeout(this.tickTimer);
          this.tickTimer = null;
        }
        const rawMapFile = (msg as any).map ?? this.currentMapFile;
        const mapFile = path.basename(rawMapFile);
        const resolvedMap = path.resolve(this.mapsDir, mapFile);
        if (!resolvedMap.startsWith(this.mapsDir + path.sep) && resolvedMap !== this.mapsDir) {
          break;
        }
        this.initialize(mapFile);
        // Broadcast new init to all clients
        for (const client of this.clients) {
          if (client.readyState === WebSocket.OPEN) {
            this.sendInit(client);
          }
        }
        break;

      case "target":
        if (msg.robotId) {
          const robot = this.robots.find((r) => r.id === msg.robotId);
          if (robot) {
            const slamPos = robot.worldToSlam(msg.x, msg.y);
            robot.agent.setManualTarget(slamPos);
          }
        } else if (this.robots.length > 0) {
          const robot = this.robots[0];
          const slamPos = robot.worldToSlam(msg.x, msg.y);
          robot.agent.setManualTarget(slamPos);
        }
        break;

      case "nodeInfo":
        this.sendNodeInfo(ws, msg.nodeId);
        break;

      case "edgeInfo":
        this.sendEdgeInfo(ws, msg.edgeId);
        break;

      case "robotInfo": {
        const robot = this.robots.find((r) => r.id === msg.robotId);
        if (robot) {
          const info = robot.agent.getRobotInfo();
          const actual = robot.getActualPose();
          const believed = robot.slamToWorld(info.slamPose.x, info.slamPose.y);
          ws.send(JSON.stringify({
            type: "robotInfo",
            robotId: robot.id,
            worldPose: { x: actual.x, y: actual.y, theta: actual.theta },
            believedPose: { x: believed.x, y: believed.y, theta: info.slamPose.theta },
            slamPose: info.slamPose,
            positionError: robot.getPositionError(),
            cumulativeDrift: robot.getCumulativeDrift(),
            status: info.status,
            targetPosition: info.targetPosition,
            targetNodeId: info.targetNodeId,
            currentNodeId: info.currentNodeId,
            activeEdgeId: info.activeEdgeId,
            tickCount: info.tickCount,
            nodesDiscovered: info.nodesDiscovered,
            edgesDiscovered: info.edgesDiscovered,
            behaviorMode: info.behaviorMode,
          }));
        }
        break;
      }

      case "setMode": {
        const robot = this.robots.find((r) => r.id === msg.robotId);
        if (robot && (Object.values(BehaviorMode) as string[]).includes(msg.mode)) {
          robot.agent.setBehaviorMode(msg.mode as BehaviorMode);
          this.broadcastToViz({
            type: "modeChanged",
            robotId: msg.robotId,
            mode: msg.mode,
          });
        }
        break;
      }

      case "editCell":
        this.editCell(msg.x, msg.y, msg.cellType);
        this.broadcastToViz({
          type: "cellEdited",
          x: msg.x,
          y: msg.y,
          cellType: msg.cellType,
        });
        break;

      case "validate":
        this.singleStep();
        break;
    }
    } catch (err) {
      console.error("Error handling WebSocket message:", err);
    }
  }

  private sendNodeInfo(ws: WebSocket, nodeId: string): void {
    // Find the node across all robots
    for (const robot of this.robots) {
      const node = robot.agent.graph.getNode(nodeId);
      if (!node) continue;

      const localMap = robot.agent.getNodeLocalMap(nodeId);
      const edges = robot.agent.graph.getNodeEdges(nodeId);
      const worldPos = robot.slamToWorld(node.position.x, node.position.y);

      const msg: VizNodeInfoMessage = {
        type: "nodeInfo",
        data: {
          id: node.id,
          type: node.type,
          x: worldPos.x,
          y: worldPos.y,
          confidence: node.confidence,
          tier: node.tier,
          observationCount: node.observation_count,
          discoveredBy: node.discovered_by,
          discoveredAt: node.discovered_at,
          createdAt: node.discovered_at,
          lastObserved: node.last_observed,
          stabilityClass: node.stability_class,
          explored: node.explored,
          exploredBy: node.explored_by,
          isFrontier: edges.length < node.exit_count,
          edgeCount: edges.length,
          edges: edges.map((e) => ({
            id: e.id,
            targetNodeId: e.source_node === nodeId ? e.target_node : e.source_node,
            distance: e.distance,
            tier: e.tier,
            traveled: e.traveled,
          })),
          localMap: localMap?.cells ?? [],
          localMapSize: localMap?.cells.length ?? 0,
        },
      };
      ws.send(JSON.stringify(msg));
      return;
    }
  }

  private sendEdgeInfo(ws: WebSocket, edgeId: string): void {
    for (const robot of this.robots) {
      const edge = robot.agent.graph.getEdge(edgeId);
      if (!edge) continue;

      const corridorData = robot.agent.getEdgeCorridorMap(edgeId);

      const msg: VizEdgeInfoMessage = {
        type: "edgeInfo",
        data: {
          id: edge.id,
          type: "corridor",
          sourceNodeId: edge.source_node,
          targetNodeId: edge.target_node,
          distance: edge.distance,
          confidence: edge.confidence,
          tier: edge.tier,
          status: edge.traversability?.status ?? "open",
          traveled: edge.traveled,
          bidirectional: edge.bidirectional,
          observationCount: 1,
          corridorMap: corridorData?.corridor_map_cells ?? [],
          corridorMapSize: corridorData?.corridor_map_cells.length ?? 0,
        },
      };
      ws.send(JSON.stringify(msg));
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// Main Entry Point
// ---------------------------------------------------------------------------

if (require.main === module) {
  const port = parseInt(process.env.PORT || "3000", 10);
  const server = new WorldServer();
  server.initialize();
  server.startServer(port);
}
