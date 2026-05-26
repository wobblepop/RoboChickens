const { WorldServer } = require('./dist/world/world-server');
const srv = new WorldServer();
srv.initialize('map-01-zigzag.txt');
const robots = srv['robots'];
const config = srv['config'];
let tickCount = 0;
function tick() { tickCount++; for (const r of robots) r.tick(tickCount * config.tick_interval_ms); }
const r = robots[0];

// Run until robot reaches corridor end area
for (let i = 0; i < 200; i++) tick();

console.log('=== After 200 ticks ===');
const a = r.getActualPose();
const pose = r.agent.pose;
console.log('World:', a.x.toFixed(1), a.y.toFixed(1));
console.log('SLAM:', pose.x.toFixed(1), pose.y.toFixed(1));

// Show all nodes
const nodes = r.agent.getAllNodes();
console.log('\nNodes (' + nodes.length + '):');
for (const n of nodes) {
  const wx = n.position.x + (a.x - pose.x);
  const wy = n.position.y + (a.y - pose.y);
  console.log('  ' + n.id.slice(0,8) + ' slam=(' + n.position.x.toFixed(1) + ',' + n.position.y.toFixed(1) + ') world~(' + wx.toFixed(1) + ',' + wy.toFixed(1) + ') ' + n.type + ' exp=' + n.explored);
}

// Show all edges
const graph = r.agent['graph'];
const edges = graph.getAllEdges ? graph.getAllEdges() : [];
console.log('\nEdges (' + edges.length + '):');
for (const e of edges) {
  const sn = graph.getNode(e.source_node);
  const tn = graph.getNode(e.target_node);
  const sName = sn ? sn.id.slice(0,8) : '?';
  const tName = tn ? tn.id.slice(0,8) : '?';
  console.log('  ' + sName + ' -> ' + tName + ' dist=' + e.distance.toFixed(1) + ' status=' + e.traversability.status);
}

// Check: what does chooseExplorationEdge return from the corridor-end nodes?
const endNodes = nodes.filter(n => n.position.x > 18);
console.log('\nChoosing from east-end nodes:');
for (const n of endNodes) {
  const choice = graph.chooseExplorationEdge(n.id);
  console.log('  From ' + n.id.slice(0,8) + ': ' + (choice ? 'edge to ' + graph.getOtherNode(choice.edge, n.id)?.id.slice(0,8) + ' reason=' + choice.reason : 'null'));
}

// Check if the unexplored node has any edges
const unexplored = nodes.filter(n => !n.explored);
console.log('\nUnexplored nodes:');
for (const n of unexplored) {
  const nEdges = graph.getNodeEdges(n.id);
  console.log('  ' + n.id.slice(0,8) + ' slam=(' + n.position.x.toFixed(1) + ',' + n.position.y.toFixed(1) + ') edges=' + nEdges.length);
  for (const e of nEdges) {
    const other = e.source_node === n.id ? e.target_node : e.source_node;
    console.log('    -> ' + other.slice(0,8) + ' dist=' + e.distance.toFixed(1));
  }
}

// Check LOS from corridor end to connector area
const slamOff = { x: a.x - pose.x, y: a.y - pose.y };
// Connector at world (22.5, 2.5) → SLAM
const connSlam = { x: 22.5 - slamOff.x, y: 2.5 - slamOff.y };
console.log('\nConnector SLAM est:', connSlam.x.toFixed(1), connSlam.y.toFixed(1));
// Check from the easternmost explored node
const eastNode = nodes.reduce((best, n) => n.position.x > best.position.x && n.explored ? n : best);
console.log('Easternmost explored node:', eastNode.id.slice(0,8), 'slam=(' + eastNode.position.x.toFixed(1) + ',' + eastNode.position.y.toFixed(1) + ')');
const losToConn = r.agent['hasLineOfSight'](eastNode.position.x, eastNode.position.y, connSlam.x, connSlam.y);
console.log('LOS from east node to connector:', losToConn);

// Grid state around connector
const slam = r.agent['slam'];
const res = slam.grid.config.resolution;
console.log('\nGrid near connector (SLAM):');
for (let y = connSlam.y - 1.0; y <= connSlam.y + 1.0; y += res * 3) {
  let row = '';
  for (let x = connSlam.x - 2.0; x <= connSlam.x + 1.0; x += res) {
    const free = slam.isPositionFree(x, y);
    const occ = slam.isPositionOccupied(x, y);
    if (free) row += '.';
    else if (occ) row += '#';
    else row += '?';
  }
  row += '  y=' + y.toFixed(1);
  console.log('  ' + row);
}
