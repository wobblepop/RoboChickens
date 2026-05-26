const { WorldServer } = require('./dist/world/world-server');
const srv = new WorldServer();
srv.initialize('map-01-zigzag.txt');
const robots = srv['robots'];
const config = srv['config'];
let tickCount = 0;
function tick() { tickCount++; for (const r of robots) r.tick(tickCount * config.tick_interval_ms); }
const r = robots[0];

// Run 40 ticks to get robot to mid-corridor
for (let i = 0; i < 40; i++) tick();

const agent = r.agent;
const slam = agent['slam'];
const slamPose = agent.getRobotInfo().slamPose;
const resolution = slam.grid.config.resolution;

console.log('Robot at SLAM:', slamPose.x.toFixed(2), slamPose.y.toFixed(2));
console.log('Grid resolution:', resolution);

// Scan east along the corridor center (y=0) to find where free→unknown transition is
let lastState = '';
for (let x = slamPose.x; x < slamPose.x + 20; x += resolution) {
  const free = slam.isPositionFree(x, slamPose.y);
  const occ = slam.isPositionOccupied(x, slamPose.y);
  const unk = slam.isPositionUnknown(x, slamPose.y);
  const state = free ? 'F' : occ ? 'O' : 'U';
  if (state !== lastState) {
    console.log('  x=' + x.toFixed(2) + ' -> ' + state + (free ? ' (free)' : occ ? ' (occupied)' : ' (unknown)'));
    lastState = state;
  }
}

// Now check what findGridFrontier does step by step
const stepSize = resolution * 3;
console.log('\nFrontier search (stepSize=' + stepSize.toFixed(2) + '):');
let found = 0;
for (let radius = 2.0; radius < 15; radius += stepSize) {
  const angleSteps = Math.max(8, Math.ceil(2 * Math.PI * radius / stepSize));
  for (let i = 0; i < angleSteps; i++) {
    const angle = (i / angleSteps) * 2 * Math.PI;
    const px = slamPose.x + radius * Math.cos(angle);
    const py = slamPose.y + radius * Math.sin(angle);
    if (slam.isPositionFree(px, py)) {
      const offsets = [{dx:resolution,dy:0},{dx:-resolution,dy:0},{dx:0,dy:resolution},{dx:0,dy:-resolution}];
      for (const off of offsets) {
        if (slam.isPositionUnknown(px + off.dx, py + off.dy)) {
          const los = agent['hasLineOfSight'](slamPose.x, slamPose.y, px, py);
          if (found < 10) {
            console.log('  r=' + radius.toFixed(2) + ' angle=' + (angle*180/Math.PI).toFixed(0) + '° pos=(' + px.toFixed(2) + ',' + py.toFixed(2) + ') LOS=' + los);
          }
          found++;
          break;
        }
      }
    }
  }
}
console.log('Total frontier candidates found:', found, '(with LOS filter, valid:', found + ' shown above)');
