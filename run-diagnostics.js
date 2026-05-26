// Headless diagnostic: run each map for 200 ticks and check for errors
const path = require("path");
const { WorldServer } = require("./dist/world/world-server");
const { DEFAULT_SIMULATION_CONFIG } = require("./dist/shared/types");

const mapsDir = path.resolve(__dirname, "warehouse-maps");
const fs = require("fs");
const maps = fs.readdirSync(mapsDir).filter(f => f.startsWith("map-") && f.endsWith(".txt")).sort();

function runDiagnostic(mapFile, ticks) {
  const server = new WorldServer(DEFAULT_SIMULATION_CONFIG);
  server.initialize(mapFile);

  const errors = [];
  const warnings = [];

  for (let t = 1; t <= ticks; t++) {
    const timestamp = t * DEFAULT_SIMULATION_CONFIG.tick_interval_ms;
    for (const robot of server.robots) {
      robot.tick(timestamp);
    }
  }

  // Collect state from all robots
  for (const robot of server.robots) {
    const agent = robot.agent;
    const nodes = agent.getAllNodes();
    const edges = agent.getAllEdges();
    const pose = agent.pose;
    const worldPos = robot.slamToWorld(pose.x, pose.y);

    // CHECK 1: Nodes out of bounds
    for (const node of nodes) {
      const wp = robot.slamToWorld(node.position.x, node.position.y);
      if (wp.x < 0 || wp.y < 0 || wp.x > server.world.width || wp.y > server.world.height) {
        errors.push(`[${robot.id}] Node ${node.id.slice(0,8)} out of bounds at world (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`);
      }
    }

    // CHECK 2: Nodes in wall cells
    for (const node of nodes) {
      const wp = robot.slamToWorld(node.position.x, node.position.y);
      const gx = Math.floor(wp.x);
      const gy = Math.floor(wp.y);
      if (server.world.inBounds(gx, gy) && server.world.isOccupied(gx, gy)) {
        errors.push(`[${robot.id}] Node ${node.id.slice(0,8)} in wall at grid (${gx}, ${gy})`);
      }
    }

    // CHECK 3: Starting node position
    if (nodes.length > 0) {
      const firstNode = nodes.find(n => n.discovered_at <= DEFAULT_SIMULATION_CONFIG.tick_interval_ms * 2);
      if (firstNode) {
        const wp = robot.slamToWorld(firstNode.position.x, firstNode.position.y);
        const startX = robot.slamToWorld(0, 0).x;
        const startY = robot.slamToWorld(0, 0).y;
        const drift = Math.sqrt((wp.x - startX) ** 2 + (wp.y - startY) ** 2);
        if (drift > 0.5) {
          errors.push(`[${robot.id}] Starting node drifted ${drift.toFixed(2)}m from start pos`);
        }
      }
    }

    // CHECK 4: Edges through walls (SLAM grid check)
    for (const edge of edges) {
      const srcNode = agent.graph.getNode(edge.source_node);
      const tgtNode = agent.graph.getNode(edge.target_node);
      if (!srcNode || !tgtNode) continue;

      const sx = srcNode.position.x;
      const sy = srcNode.position.y;
      const tx = tgtNode.position.x;
      const ty = tgtNode.position.y;
      const dist = Math.sqrt((tx-sx)**2 + (ty-sy)**2);
      const steps = Math.ceil(dist / 0.1);
      const dx = (tx - sx) / steps;
      const dy = (ty - sy) / steps;

      let throughWall = false;
      for (let i = 1; i < steps; i++) {
        const px = sx + dx * i;
        const py = sy + dy * i;
        const wpx = robot.slamToWorld(px, py);
        const gx = Math.floor(wpx.x);
        const gy = Math.floor(wpx.y);
        if (server.world.inBounds(gx, gy) && server.world.isOccupied(gx, gy)) {
          throughWall = true;
          break;
        }
      }
      if (throughWall) {
        errors.push(`[${robot.id}] Edge ${edge.id.slice(0,8)} goes through wall (${edge.source_node.slice(0,6)}->${edge.target_node.slice(0,6)}, dist=${dist.toFixed(1)}m)`);
      }
    }

    // CHECK 5: Orphaned nodes (no edges)
    for (const node of nodes) {
      const nodeEdges = agent.graph.getNodeEdges(node.id);
      if (nodeEdges.length === 0 && nodes.length > 1) {
        warnings.push(`[${robot.id}] Orphaned node ${node.id.slice(0,8)} at SLAM (${node.position.x.toFixed(1)}, ${node.position.y.toFixed(1)})`);
      }
    }

    // CHECK 6: Robot stuck (hasn't moved much)
    const totalDist = Math.sqrt(pose.x ** 2 + pose.y ** 2);
    if (totalDist < 1.0 && ticks > 50) {
      warnings.push(`[${robot.id}] Robot barely moved (${totalDist.toFixed(2)}m from origin after ${ticks} ticks)`);
    }

    // CHECK 7: Explored node has no SLAM data (node map)
    for (const node of nodes) {
      if (node.explored) {
        const map = agent.getNodeLocalMap(node.id);
        if (!map || map.cells.length === 0) {
          warnings.push(`[${robot.id}] Explored node ${node.id.slice(0,8)} has no SLAM map data`);
        }
      }
    }

    // CHECK 8: Edges longer than LiDAR range
    for (const edge of edges) {
      if (edge.distance > DEFAULT_SIMULATION_CONFIG.lidar.max_range * 1.1) {
        errors.push(`[${robot.id}] Edge ${edge.id.slice(0,8)} longer than LiDAR range (${edge.distance.toFixed(1)}m > ${DEFAULT_SIMULATION_CONFIG.lidar.max_range}m)`);
      }
    }

    // CHECK 9: Nodes that should have been created but weren't
    // (explored node with fewer edges than expected exit count)
    for (const node of nodes) {
      if (node.explored) {
        const edgeCount = agent.graph.getNodeEdges(node.id).length;
        if (node.exit_count > 0 && edgeCount === 0) {
          errors.push(`[${robot.id}] Explored node ${node.id.slice(0,8)} has ${node.exit_count} exits but 0 edges`);
        }
      }
    }

    // CHECK 10: Duplicate edges between same pair of nodes
    const edgePairs = new Set();
    for (const edge of edges) {
      const key = [edge.source_node, edge.target_node].sort().join("::");
      if (edgePairs.has(key)) {
        errors.push(`[${robot.id}] Duplicate edge between ${edge.source_node.slice(0,6)} and ${edge.target_node.slice(0,6)}`);
      }
      edgePairs.add(key);
    }

    // Stats
    const exploredCount = nodes.filter(n => n.explored).length;
    const traveledCount = edges.filter(e => e.traveled).length;
    console.log(`  ${robot.id}: ${nodes.length} nodes (${exploredCount} explored), ${edges.length} edges (${traveledCount} traveled), pos=(${worldPos.x.toFixed(1)},${worldPos.y.toFixed(1)})`);
  }

  // ASCII visualization: overlay nodes and robots on the map
  const w = server.world.width;
  const h = server.world.height;
  const grid = [];
  for (let y = 0; y < h; y++) {
    const row = [];
    for (let x = 0; x < w; x++) {
      const cell = server.world.getCell(x, y);
      row.push(cell === '#' ? '#' : cell === 'O' ? 'O' : cell === 'D' ? 'D' : '.');
    }
    grid.push(row);
  }
  // Place nodes
  const robotSymbols = ['1','2','3','4','5','6'];
  for (let ri = 0; ri < server.robots.length; ri++) {
    const robot = server.robots[ri];
    for (const node of robot.agent.getAllNodes()) {
      const wp = robot.slamToWorld(node.position.x, node.position.y);
      const gx = Math.floor(wp.x);
      const gy = Math.floor(wp.y);
      if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
        grid[gy][gx] = node.explored ? (node.discovered_by === robot.id ? '*' : '+') : 'o';
      }
    }
    // Place robot
    const pose = robot.agent.pose;
    const rwp = robot.slamToWorld(pose.x, pose.y);
    const rgx = Math.floor(rwp.x);
    const rgy = Math.floor(rwp.y);
    if (rgx >= 0 && rgx < w && rgy >= 0 && rgy < h) {
      grid[rgy][rgx] = robotSymbols[ri] || 'R';
    }
  }
  console.log('  ' + grid.map(r => r.join('')).join('\n  '));

  return { errors, warnings };
}

console.log("=== SLAM Explorer Diagnostics ===\n");

let totalErrors = 0;
let totalWarnings = 0;

for (const mapFile of maps) {
  console.log(`\n--- ${mapFile} (200 ticks) ---`);
  try {
    const { errors, warnings } = runDiagnostic(mapFile, 200);
    totalErrors += errors.length;
    totalWarnings += warnings.length;

    if (errors.length > 0) {
      console.log(`  ERRORS (${errors.length}):`);
      // Show first 5 of each type
      const shown = new Set();
      for (const e of errors) {
        const type = e.replace(/\[.*?\]/, "").replace(/[a-f0-9-]{8,}/g, "XXX").trim();
        if (!shown.has(type)) {
          shown.add(type);
          console.log(`    ${e}`);
        }
        if (shown.size >= 8) break;
      }
      if (errors.length > shown.size) console.log(`    ... and ${errors.length - shown.size} more`);
    }

    if (warnings.length > 0) {
      console.log(`  WARNINGS (${warnings.length}):`);
      const shown = new Set();
      for (const w of warnings) {
        const type = w.replace(/\[.*?\]/, "").replace(/[a-f0-9-]{8,}/g, "XXX").trim();
        if (!shown.has(type)) {
          shown.add(type);
          console.log(`    ${w}`);
        }
        if (shown.size >= 5) break;
      }
    }

    if (errors.length === 0 && warnings.length === 0) {
      console.log("  CLEAN");
    }
  } catch (e) {
    console.log(`  CRASH: ${e.message}`);
    totalErrors++;
  }
}

console.log(`\n=== TOTALS: ${totalErrors} errors, ${totalWarnings} warnings ===`);
process.exit(totalErrors > 0 ? 1 : 0);
