const { WorldServer } = require('./dist/world/world-server');
const srv = new WorldServer();
srv.initialize('map-01-zigzag.txt');
const robots = srv['robots'];
const config = srv['config'];
let tickCount = 0;
function tick() { tickCount++; for (const r of robots) r.tick(tickCount * config.tick_interval_ms); }
const r = robots[0];

for (let i = 0; i < 500; i++) {
  tick();
  const a = r.getActualPose();
  const agent = r.agent;
  const phase = agent.currentStatus;
  const info = agent.getRobotInfo();
  const slamPose = info.slamPose;

  if (phase === 'choosing') {
    const graph = agent['graph'];
    const nodes = agent.getAllNodes();
    const unexplored = nodes.filter(n => !n.explored);
    const choice = graph.chooseExplorationEdge(info.currentNodeId);
    const frontier = agent['findGridFrontier'](slamPose);

    console.log(
      't' + String(tickCount).padStart(4) +
      ' w=(' + a.x.toFixed(1) + ',' + a.y.toFixed(1) + ')' +
      ' s=(' + slamPose.x.toFixed(1) + ',' + slamPose.y.toFixed(1) + ')' +
      ' N=' + nodes.length + ' unexp=' + unexplored.length +
      ' choice=' + (choice ? choice.reason : 'null') +
      ' frontier=' + (frontier ? '(' + frontier.x.toFixed(1) + ',' + frontier.y.toFixed(1) + ') LOS=true' : 'null')
    );
  }
}

const a = r.getActualPose();
const s = r.agent.getRobotInfo().slamPose;
console.log('\nFinal: world=(' + a.x.toFixed(1) + ',' + a.y.toFixed(1) + ') slam=(' + s.x.toFixed(1) + ',' + s.y.toFixed(1) + ')');
console.log('Nodes:', r.agent.getAllNodes().length, '| Collisions:', r.physicalState.collisionCount || 0);
