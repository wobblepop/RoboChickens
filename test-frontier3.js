const { WorldServer } = require('./dist/world/world-server');
const srv = new WorldServer();
srv.initialize('map-01-zigzag.txt');
const robots = srv['robots'];
const config = srv['config'];
let tickCount = 0;
function tick() { tickCount++; for (const r of robots) r.tick(tickCount * config.tick_interval_ms); }
const r = robots[0];

let maxWorldX = 0;
let maxWorldY = 0;
for (let i = 0; i < 3000; i++) {
  tick();
  const a = r.getActualPose();
  if (a.x > maxWorldX) maxWorldX = a.x;
  if (a.y > maxWorldY) maxWorldY = a.y;
  if (tickCount % 500 === 0) {
    const s = r.agent.getRobotInfo().slamPose;
    console.log('t' + tickCount + ' w=(' + a.x.toFixed(1) + ',' + a.y.toFixed(1) + ') maxX=' + maxWorldX.toFixed(1) + ' maxY=' + maxWorldY.toFixed(1) + ' N=' + r.agent.getAllNodes().length);
  }
}
const a = r.getActualPose();
console.log('\nFinal: world=(' + a.x.toFixed(1) + ',' + a.y.toFixed(1) + ') maxX=' + maxWorldX.toFixed(1) + ' maxY=' + maxWorldY.toFixed(1));
console.log('Nodes:', r.agent.getAllNodes().length, '| Collisions:', r.physicalState.collisionCount || 0);
