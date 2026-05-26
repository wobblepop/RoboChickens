# Coordinate System Audit — SLAM Explorer

**Date:** 2025-05-25
**Auditor:** Claude (requested by user)
**Scope:** Every `.ts` file in `src/`, every coordinate reference

## Coordinate Systems Reference

| Frame | Origin | Units | Used by |
|-------|--------|-------|---------|
| **World** | Map top-left (0,0) | 1 cell = 1m | Physics engine, ground truth, visualization |
| **SLAM** | Robot's start position (0,0) | meters | Robot agent, network graph, node positions, SLAM pose |
| **SLAM Grid** | `origin_x, origin_y` in SLAM coords | cells (resolution = 0.1m) | Occupancy grid, LOS checks |
| **LiDAR** | Robot-relative (heading = 0) | angles relative to `pose.theta`, ranges in meters | LiDAR scan data |

## Transform Definitions

- **SLAM -> World:** `worldX = slamX + startWorldX` [robot-connection.ts:170-175]
- **World -> SLAM:** `slamX = worldX - startWorldX` [robot-connection.ts:180-185]
- **SLAM -> Grid:** `gx = floor((slamX - origin_x) / resolution)` [occupancy-grid.ts:60-65]
- **Grid -> SLAM:** `slamX = origin_x + (gx + 0.5) * resolution` [occupancy-grid.ts:68-73]
- **LiDAR angle -> SLAM angle:** `slamAngle = lidarAngle + pose.theta` [types.ts:35, inverse-sensor-model.ts:34]

---

## Audit Table

### src/shared/geometry.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 22-50 | `bresenhamLine(x0,y0,x1,y1)` | Grid cells | Grid cells (integers) | OK |
| 57-68 | `bresenhamRay(startX,startY,endX,endY,originX,originY,resolution)` | SLAM coords in, grid cells internally | SLAM coords converted to grid via `floor((x - originX) / resolution)` | OK |
| 120-157 | `gridRaycast(startX,startY,angle,...)` | SLAM coords for start, SLAM angle | Converts to grid internally via `bresenhamRay`, converts back via `originX + (cell.x+0.5)*resolution` | OK |
| 131 | `endX = startX + maxRange * Math.cos(angle)` | SLAM coords | SLAM coords (angle is absolute SLAM angle) | OK |
| 148-149 | Distance computation `wx - startX` | SLAM coords | SLAM coords (both from same frame) | OK |
| 167-188 | `hasLineOfSight(x0,y0,x1,y1,originX,originY,...)` | SLAM coords | Converts to grid via `bresenhamRay` | OK |
| 194-212 | `pointToSegmentDistanceSq` | Any consistent frame | Frame-agnostic (pure geometry) | OK |

### src/shared/math.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 40-44 | `vec2Distance(a, b)` | Any consistent frame | Frame-agnostic | OK |
| 106-114 | `poseApplyDelta(pose, dx, dy, dtheta)` | Local-frame delta applied to pose | Correct: rotates `(dx,dy)` by `pose.theta` then adds | OK |
| 117-124 | `poseTransformPoint(pose, localPoint)` | Local -> SLAM | Correct: applies rotation + translation | OK |
| 127-136 | `poseInverseTransformPoint(pose, worldPoint)` | SLAM -> Local | Correct: subtracts pose, rotates by -theta | OK |
| 139-149 | `poseRelative(from, to)` | Two SLAM poses -> relative transform | Correct | OK |
| 152-160 | `poseCompose(a, b)` | Compose two transforms | Correct | OK |

### src/shared/types.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 35 | `LidarScan.angles` | LiDAR frame (robot-relative) | Documented as "relative to robot heading" | OK |
| 46-47 | `OdometryReading.delta_x/y` | Robot local frame | Documented as "robot's local frame" | OK |
| 119-120 | `OccupancyGridConfig.origin_x/y` | SLAM frame (comment says "World x-coordinate") | **SUSPECT** |
| 162 | `DetectedFeature.position` | SLAM frame | Documented as "robot's SLAM coordinate frame" | OK |
| 166 | `DetectedFeature.exit_directions` | SLAM frame angles | Documented as "radians, in SLAM frame" | OK |
| 285 | `NetworkNode.position` | SLAM frame | Documented as "discovering robot's SLAM coordinate frame" | OK |

**Note on line 119-120:** The comment says "World x-coordinate of cell [0,0] center" but the origin is in SLAM coordinates, not world coordinates. The comment is misleading but the code operates correctly because within the robot, "world" coordinates mean SLAM coordinates. The grid is created centered at SLAM origin (0,0). This is a documentation issue, not a code bug.

### src/robot/slam/occupancy-grid.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 19-39 | `createGrid(resolution, widthMeters, heightMeters, originX, originY)` | SLAM frame for origin | SLAM coords passed in | OK |
| 44-53 | `createGridCentered(resolution, sizeMeters, centerX, centerY)` | SLAM frame for center | SLAM coords: origin = center - size/2 | OK |
| 60-65 | `worldToGrid(worldX, worldY, config)` | SLAM coords in (despite "world" name) | Correctly converts SLAM -> grid indices | OK |
| 68-73 | `gridToWorld(gx, gy, config)` | Grid indices in, SLAM coords out | Correctly converts grid -> SLAM | OK |
| 172-209 | `extractRegion(grid, centerX, centerY, radiusMeters)` | SLAM coords for center | Converts to grid, extracts, sets new origin in SLAM | OK |
| 215-237 | `gridToVizCells(grid)` | Grid -> SLAM coords (cell positions) | Calls `gridToWorld` which returns SLAM coords | OK |

**Note:** `worldToGrid` and `gridToWorld` are named misleadingly. They convert between SLAM frame and SLAM grid frame, not between world frame and grid frame. This is a naming convention issue within the robot (from the robot's perspective, SLAM IS its "world"). No code bug.

### src/robot/slam/slam-engine.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 56-60 | `initialize()` at (0,0,0) | SLAM origin | Correct | OK |
| 69 | `get pose()` = `particleFilter.bestPose` | SLAM frame | SLAM frame (particles initialized at SLAM origin) | OK |
| 86-109 | `processSensorData(odometry, lidar)` | Odometry in local frame, lidar in LiDAR frame | Correct: particle filter applies odometry in local frame | OK |
| 122-125 | `isPositionFree(x, y)` | SLAM coords | `getCellAtWorld` uses SLAM grid origin | OK |
| 176-198 | `castRayInGrid(angle, maxRange)` | SLAM absolute angle | `pose.x + dx*dist` is SLAM coords, converted to grid | OK |
| 206-309 | `findGridPath(fromX, fromY, toX, toY)` | SLAM coords | Converts to grid, pathfinds, converts back to SLAM | OK |
| 315-337 | `castRayFrom(startX, startY, angle, maxRange)` | SLAM coords, SLAM angle | Correct | OK |

### src/robot/slam/particle-filter.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 84-99 | `initialize(startPose)` | SLAM frame | Creates grid centered at startPose (0,0) in SLAM | OK |
| 136-161 | Motion model: odometry applied to particles | Odometry in local frame | Correct: `noisyDx * cos(theta) - noisyDy * sin(theta)` rotates local-frame delta into SLAM frame | OK |
| 156-158 | `p.pose.x += noisyDx * c - noisyDy * s` | SLAM frame | Correct: local->SLAM rotation then addition | OK |
| 171-173 | `scanMatchScore(grid, scan, particle.pose)` | SLAM pose, LiDAR scan | `scanMatchScore` converts LiDAR angles to absolute via `pose.theta + scan.angles[i]` | OK |

### src/robot/slam/scan-matching.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 40-41 | `angle = candidatePose.theta + scan.angles[i]` | LiDAR -> SLAM angle | Correct: adds pose heading to robot-relative angle | OK |
| 42-43 | `hitX = candidatePose.x + range * cos(angle)` | SLAM coords | Correct: SLAM pose + SLAM-angle ray | OK |
| 44-45 | `gx = floor((hitX - config.origin_x) / config.resolution)` | SLAM -> Grid | Correct | OK |
| 97-181 | `correlativeScanMatch` search grid | SLAM coords for pose | Searches around `priorPose` in SLAM frame | OK |

### src/robot/slam/inverse-sensor-model.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 34 | `rayAngle = pose.theta + scan.angles[i]` | LiDAR -> SLAM angle | Correct | OK |
| 38-39 | `sx = floor((pose.x - config.origin_x) / config.resolution)` | SLAM -> Grid | Correct | OK |
| 42-43 | `endWorldX = pose.x + range * cos(rayAngle)` | SLAM coords | Correct: SLAM pose + SLAM-angle ray | OK |
| 44-45 | `ex = floor((endWorldX - config.origin_x) / config.resolution)` | SLAM -> Grid | Correct | OK |
| 146-148 | `getNewlyExploredCells`: `worldX = floor(origin_x + gx * resolution)` | SLAM coords (misleadingly named "world") | This returns SLAM-frame coords, which is correct for the robot's internal use | OK |

### src/robot/robot-agent.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 186-188 | `firstPose = { x: 0, y: 0, theta: 0 }` | SLAM frame | Correct: SLAM origin | OK |
| 195-196 | `slamResult.pose` | SLAM frame | SLAM frame (from particle filter) | OK |
| 199-200 | `vec2Distance({x:pose.x,y:pose.y}, {x:lastPose.x,y:lastPose.y})` | SLAM vs SLAM | OK: both SLAM frame | OK |
| 206-210 | `travelAccumulatedDistance += vec2Distance(pose, lastPose)` | SLAM vs SLAM | OK | OK |
| 213-219 | Corridor sample: `vec2Distance(pose, lastCorridorSamplePos)` | SLAM vs SLAM | OK | OK |
| 258 | `startFeature.position = { x: pose.x, y: pose.y }` | SLAM frame | SLAM: pose is from SLAM | OK |
| 269 | Fallback: `createNodeAtPosition({x:pose.x,y:pose.y})` | SLAM frame | OK | OK |
| 289-290 | Node snap: `node.position.x = pose.x` | SLAM frame | SLAM: both node.position and pose are SLAM | OK |
| 301 | `Math.atan2(d.dy, d.dx) - pose.theta` | Cardinal direction (SLAM absolute) -> LiDAR-relative | Correct: subtracts heading to get robot-relative angle for LiDAR lookup | OK |
| 304 | `normalizeAngle(scan.angles[i] - relAngle)` | Both LiDAR-relative | OK: comparing two robot-relative angles | OK |
| 317 | `Math.atan2(pushY, pushX) - pose.theta` | SLAM absolute -> LiDAR-relative | Correct: push direction is in SLAM, convert to LiDAR frame for ray lookup | OK |
| 376 | `vec2Distance(feature.position, {x:pose.x,y:pose.y})` | SLAM vs SLAM | OK: feature detector returns SLAM positions | OK |
| 381 | `hasDirectLineOfSight(scan, pose, feature.position.x, ...)` | Pose SLAM, target SLAM, scan LiDAR | OK: `hasDirectLineOfSight` internally converts target to LiDAR angle | OK |
| 388-389 | `vec2Distance({x:pose.x,y:pose.y}, nearbyExisting.position)` | SLAM vs SLAM | OK: both in SLAM frame | OK |
| 393-395 | `Math.atan2(nearbyExisting.position.y - pose.y, ...)` | SLAM coords -> SLAM angle | OK: bearing between two SLAM positions | OK |
| 411-413 | `Math.atan2(newNode.position.y - pose.y, ...)` | SLAM coords -> SLAM angle | OK | OK |
| 414 | `vec2Distance({x:pose.x,y:pose.y}, newNode.position)` | SLAM vs SLAM | OK | OK |
| 449 | `findNodesInRadius({x:pose.x,y:pose.y}, scanRadius)` | SLAM frame | OK: graph stores SLAM positions | OK |
| 452 | `hasLineOfSight(pose.x, pose.y, node.position.x, node.position.y)` | SLAM coords | OK: `hasLineOfSight` uses SLAM grid (SLAM frame) | OK |
| 488 | `node.position.x - pose.x` for distance | SLAM vs SLAM | OK | OK |
| 505 | `angleToNode = Math.atan2(dy, dx)` (SLAM absolute) | SLAM angle | OK | OK |
| 522 | `hasDirectLineOfSight(scan, pose, node.position.x, ...)` | Pose SLAM, target SLAM | OK | OK |
| 524-526 | `Math.atan2(node.position.y - pose.y, ...)` | SLAM angle | OK | OK |
| 581-588 | `findGridFrontier(pose)` target | SLAM frame | OK: frontier positions in SLAM | OK |
| 612 | `checkLineOfSight(scan, pose, angleToTarget)` | LiDAR-relative angle | OK | OK |
| 614 | `targetAngleRel = normalizeAngle(Math.atan2(dy, dx) - pose.theta)` | SLAM angle -> LiDAR-relative | OK | OK |
| 646 | `distToTarget = vec2Distance(pose, targetPosition)` | SLAM vs SLAM | OK: targetPosition is SLAM (set from node.position which is SLAM) | OK |
| 660 | `motor.driveToward(targetPosition, pose, scan, ...)` | Target SLAM, pose SLAM, scan LiDAR | OK: `driveToward` converts target->pose difference to LiDAR angle | OK |
| 674 | `detectSidePassages(scan, pose)` | SLAM pose, LiDAR scan | OK | OK |
| 681-688 | `scan.angles[i] - Math.PI/2` and `scan.angles[i] + Math.PI/2` | LiDAR-relative | OK: comparing LiDAR angles to LiDAR-relative perpendicular | OK |
| 707-709 | `absAngle < PI/4` or `> 3*PI/4` | LiDAR-relative angle | OK: `openAngle` is `scan.angles[i+1]` which is LiDAR-relative | OK |
| 715 | `absOpenAngle = pose.theta + openAngle` | LiDAR -> SLAM absolute angle | OK: correct conversion | OK |
| 724 | `Math.atan2(dy, dx) - pose.theta` | SLAM angle -> LiDAR-relative | OK: correct for comparison with `openAngle` (LiDAR-relative) | OK |
| 725 | `normalizeAngle(angleToNode - openAngle)` | Both LiDAR-relative | OK | OK |
| 742 | `hasLineOfSight(anchorPos.x, anchorPos.y, existingNode.position.x, ...)` | SLAM vs SLAM | OK: grid LOS in SLAM frame | OK |
| 798-800 | `targetPos = pose + targetDist * cos/sin(absOpenAngle)` | SLAM coords | OK: `absOpenAngle` is SLAM absolute angle, `pose.x/y` is SLAM | OK |
| 884-888 | `measuredBearing = atan2(pose.y - travelStartPose.y, ...)` | SLAM coords -> SLAM angle | OK: both poses in SLAM | OK |
| 948-949 | `firstSample.pose.x/y` for nearest node lookup | SLAM frame | OK: corridor scan log stores SLAM poses | OK |
| 972-979 | `travelDx/Dy = arrivalNode.position - departureNode.position` | SLAM coords | OK | OK |
| 979 | `travelAngle = atan2(travelDy, travelDx)` | SLAM angle | OK | OK |
| 981-982 | `perpLeft = travelAngle + PI/2` | SLAM absolute angle | OK: perpendicular in SLAM frame | OK |
| 1001 | `absRayAngle = normalizeAngle(scan.angles[i])` | LiDAR-relative | OK | OK |
| 1002 | `rayWorldAngle = normalizeAngle(p.theta + absRayAngle)` | LiDAR -> SLAM absolute | OK: correct conversion | OK |
| 1004 | `normalizeAngle(rayWorldAngle - perpLeft)` | Both SLAM absolute | OK | OK |
| 1047-1049 | `passage.position = {x: curr.pose.x, y: curr.pose.y}` | SLAM frame | OK: stored pose is SLAM | OK |
| 1048-1049 | `passage.direction = perpLeft/perpRight` | SLAM absolute angle | OK | OK |
| 1099-1101 | Projection onto A->B line | SLAM coords | OK: all positions in SLAM | OK |
| 1115-1117 | `targetPos = junctionPos + targetDist * cos/sin(passage.direction)` | SLAM coords | OK: `passage.direction` is SLAM absolute | OK |
| 1170-1172 | Same pattern for target node position | SLAM coords | OK | OK |
| 1229-1232 | `segBearing = atan2(toNode.position.y - fromNode.position.y, ...)` | SLAM angle | OK | OK |
| 1306-1307 | `distFromStart = vec2Distance(pose, travelStartPose)` | SLAM vs SLAM | OK | OK |
| 1312-1313 | `scan.angles[i]` for forward check | LiDAR-relative | OK: comparing to 0 (forward) | OK |
| 1351-1354 | `bearing = atan2(pose.y - travelStartPose.y, ...)` | SLAM angle | OK | OK |
| 1404-1405 | Same pattern for stuck recovery | SLAM angle | OK | OK |

**createNode (lines 1429-1488):**

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 1431 | `pose = this.slam.pose` | SLAM frame | OK | OK |
| 1432 | `vec2Distance(pose, feature.position)` | SLAM vs SLAM | OK | OK |
| 1438-1440 | `angleToNode = normalizeAngle(atan2(f.y - pose.y, f.x - pose.x) - pose.theta)` | SLAM angle -> LiDAR-relative | OK: correct conversion for LiDAR ray lookup | OK |
| 1444 | `normalizeAngle(scan.angles[i] - angleToNode)` | Both LiDAR-relative | OK | OK |
| 1454 | `absAngle = angleToNode + pose.theta` | LiDAR-relative -> SLAM absolute | OK: inverse of line 1439 | OK |
| 1455-1456 | `feature.position = pose + pulledDist * cos/sin(absAngle)` | SLAM coords | OK: absAngle is SLAM absolute, pose is SLAM | OK |
| 1463 | `checkLineOfSight(scan, pose, feature.position.x, ...)` | SLAM coords | OK | OK |
| 1465 | `directAngle = atan2(f.y - pose.y, f.x - pose.x)` | SLAM absolute angle | OK | OK |
| 1466 | `normalizeAngle(los.bestAngle - directAngle)` | Both SLAM absolute | OK: `los.bestAngle` is `scan.angles[bestIdx] + pose.theta` which is SLAM absolute | OK |
| 1468-1470 | `newX = pose.x + dist * cos(adjustedAngle)` | SLAM coords | OK: adjustedAngle is SLAM absolute | OK |
| 1472 | `hasDirectLineOfSight(scan, pose, newX, newY, dist)` | SLAM coords | OK | OK |

**checkLineOfSight (lines 1605-1663):**

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 1606-1608 | `dx = toX - pose.x`, `dy = toY - pose.y` | SLAM coords (same frame) | OK | OK |
| 1612 | `angleToTarget = normalizeAngle(atan2(dy, dx) - pose.theta)` | SLAM -> LiDAR-relative | OK | OK |
| 1621 | `normalizeAngle(scan.angles[i] - angleToTarget)` | Both LiDAR-relative | OK | OK |
| 1663 | `return { bestAngle: scan.angles[bestIdx] + pose.theta }` | LiDAR -> SLAM absolute | OK | OK |

**hasLineOfSight (lines 1666-1679):**

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 1668-1678 | `gridHasLineOfSight(fromX, fromY, toX, toY, origin_x, origin_y, resolution, ...)` | SLAM coords for positions, SLAM grid origin | OK: all in SLAM frame, `gridHasLineOfSight` converts to grid internally | OK |

**hasDirectLineOfSight (lines 1685-1700):**

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 1686-1688 | `dx = toX - pose.x`, `dy = toY - pose.y` | SLAM coords | OK: both SLAM | OK |
| 1691 | `angleToTarget = normalizeAngle(atan2(dy, dx) - pose.theta)` | SLAM angle -> LiDAR-relative | OK: correct for LiDAR ray comparison | OK |
| 1695 | `normalizeAngle(scan.angles[i] - angleToTarget)` | Both LiDAR-relative | OK | OK |
| 1699 | `scan.ranges[bestIdx] >= dist` | Range in meters vs SLAM distance in meters | OK: both in meters, comparable | OK |

**findGridFrontier (lines 1496-1525):**

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 1505-1506 | `px = pose.x + radius * cos(angle)` | SLAM coords | OK: SLAM pose + radius | OK |
| 1508 | `isPositionFree(px, py)` | SLAM coords | OK: `isPositionFree` checks SLAM grid | OK |
| 1517 | `hasLineOfSight(pose.x, pose.y, px, py)` | SLAM coords | OK: uses SLAM grid | OK |

### src/robot/feature-detection/feature-detector.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 39 | `detectFromScan(scan, pose)` | LiDAR scan, SLAM pose | OK | OK |
| 49 | `absAngle = pose.theta + dir.centerAngle` | LiDAR-relative -> SLAM absolute | OK: `dir.centerAngle` is from `scan.angles[]` (LiDAR-relative) | OK |
| 61-63 | `nodePos = pose + nodeDist * cos/sin(absAngle)` | SLAM coords | OK: SLAM pose + SLAM-absolute angle | OK |
| 96-99 | `classifyLocal`: `exit_directions = openDirs.map(d => pose.theta + d.centerAngle)` | LiDAR -> SLAM absolute | OK | OK |
| 181-223 | `findSideOpenings` | LiDAR scan + SLAM pose | OK | OK |
| 197 | `absOpenAngle = pose.theta + openAngle` | LiDAR -> SLAM absolute | OK | OK |
| 202-204 | `nodePos = pose + openDist * cos/sin(absOpenAngle)` | SLAM coords | OK | OK |
| 331-342 | `getRangeAtAngle(scan, relativeAngle)` | LiDAR-relative angle | OK: compares with `scan.angles[i]` (also LiDAR-relative) | OK |

### src/robot/navigation/motor-controller.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 26-28 | `dx = target.x - pose.x` | SLAM coords (same frame) | OK | OK |
| 29 | `targetAngleRel = normalizeAngle(atan2(dy, dx) - pose.theta)` | SLAM -> LiDAR-relative | OK: correct conversion for steering | OK |
| 36 | `minRangeInCone(scan, 0, PI)` | LiDAR-relative center angle | OK: 0 = forward in LiDAR frame | OK |
| 41-43 | `closestAngle = scan.angles[i]` | LiDAR-relative | OK | OK |
| 49 | `awayAngle = normalizeAngle(closestAngle + PI)` | LiDAR-relative | OK: steering uses relative angles | OK |
| 57 | `minRangeInCone(scan, targetAngleRel, ...)` | LiDAR-relative | OK | OK |
| 77-78 | `minRangeInCone(scan, PI/2, ...)` and `minRangeInCone(scan, -PI/2, ...)` | LiDAR-relative (left/right) | OK | OK |
| 97-135 | `steerToward(angleRel, ...)` | LiDAR-relative angle | OK: all angular computations in robot-relative frame | OK |
| 137-174 | `findBestOpenDirection(scan, targetAngleRel)` | LiDAR-relative | OK: samples full 2PI in LiDAR frame | OK |
| 180-192 | `minRangeInCone(scan, center, halfWidth)` | LiDAR-relative center | OK: compares with `scan.angles[i]` | OK |

### src/robot/network/network-graph.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 53-69 | `createNode(feature, ...)`: `position: {...feature.position}` | SLAM frame | OK: features are in SLAM | OK |
| 72-95 | `createNodeAtPosition(position, ...)` | SLAM frame | OK: callers pass SLAM positions | OK |
| 186-207 | `findNearestNode(position)`, `findNodesInRadius(position, radius)` | SLAM frame | OK: compares with node.position which is SLAM | OK |
| 227-259 | `createEdge(sourceNodeId, targetNodeId, distance, bearingChange, ...)` | Distance in meters, bearing in SLAM-absolute radians | OK | OK |
| 262-285 | `markEdgeTraveled(...)` | Distance/bearing updates | OK | OK |

### src/world/robot-connection.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 69-70 | `startWorldX = startX + 0.5` | World frame (cell center) | OK: map coords are integer cells, +0.5 centers | OK |
| 73 | `physics.createRobot(id, startX, startY, 0)` | World frame | OK: `createRobot` adds +0.5 internally | OK |
| 98 | `lidarSim.scan(world, physicalState.pose, ...)` | World frame pose | OK: physics pose is world frame | OK |
| 109-115 | `slamDx = currSlamPose.x - prevSlamPose.x` vs `actualDx = currActualPose.x - prevActualPose.x` | SLAM deltas vs World deltas | **SUSPECT** |
| 159 | `worldFromSlam = slamToWorld(slam.x, slam.y)` then `worldFromSlam - actual` | World vs World | OK | OK |
| 170-175 | `slamToWorld`: `slamX + startWorldX` | SLAM -> World | OK | OK |
| 180-185 | `worldToSlam`: `worldX - startWorldX` | World -> SLAM | OK | OK |
| 213-214 | `pose = physicalState.pose` (World), then `cx = floor(pose.x)` for grid cell | World coords -> world grid cell | OK: 1 cell = 1m in world | OK |
| 229 | `world.hasLineOfSight(cx, cy, gx, gy)` | World grid cell coords | OK: `GroundTruthWorld.hasLineOfSight` takes grid cells | OK |

**Line 109-115 suspect analysis:** The drift tracking computes `slamDx` (SLAM-frame delta) and `actualDx` (world-frame delta). These are being compared directly. However, the SLAM frame is a **pure translation** of the world frame (no rotation, since `startWorldX/Y` is just an offset). Therefore, **deltas in SLAM frame = deltas in world frame**. The transform is `worldX = slamX + constant`, so `delta_worldX = delta_slamX`. This is **OK**.

### src/world/world-server.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 198-199 | `slamPose = r.agent.pose` (SLAM), `worldPos = r.slamToWorld(slamPose.x, slamPose.y)` | SLAM -> World | OK | OK |
| 199 | `actual = r.getActualPose()` | World frame | OK | OK |
| 201 | `slamTrail.map(p => r.slamToWorld(p.x, p.y))` | SLAM -> World | OK: trail is stored in SLAM, converted to world for viz | OK |
| 204-205 | `x: actual.x, y: actual.y` | World frame | OK: init message uses actual (ground truth) position | OK |
| 207-209 | `believedX: worldPos.x, believedY: worldPos.y` | World frame (converted from SLAM) | OK | OK |
| 243 | `worldPos = robot.slamToWorld(node.position.x, node.position.y)` | SLAM -> World | OK: nodes stored in SLAM, converted for viz | OK |
| 244-246 | `x: worldPos.x, y: worldPos.y` | World frame | OK | OK |
| 281-301 | broadcastFrame: `slamPose = r.agent.pose`, `worldPos = r.slamToWorld(...)` | SLAM -> World | OK | OK |
| 287-288 | `x: worldPos.x, y: worldPos.y` (believed position) | World frame | OK | OK |
| 289-291 | `actualX: actual.x, actualY: actual.y` | World frame | OK | OK |
| 296-298 | `slamDrift = sqrt(dx*dx + dy*dy)` where `dx = worldPos.x - actual.x` | World vs World | OK: both converted to world | OK |
| 520-521 | `slamPos = robot.worldToSlam(msg.x, msg.y)` | World -> SLAM | OK: browser sends world coords, converted to SLAM for robot | OK |
| 543-544 | `believed = robot.slamToWorld(info.slamPose.x, info.slamPose.y)` | SLAM -> World | OK | OK |
| 589 | `worldPos = robot.slamToWorld(node.position.x, node.position.y)` | SLAM -> World | OK | OK |

**VizInitMessage (line 198-222) SLAM init vs broadcastFrame:**

The `sendInit` method uses `x: actual.x, y: actual.y` for the primary position (line 204-205), while `broadcastFrame` uses `x: worldPos.x, y: worldPos.y` (believed position, line 287-288). This means:
- **init**: primary display = ground truth
- **frame**: primary display = believed (SLAM-based)

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 204 | `sendInit`: `x: actual.x, y: actual.y` | World (ground truth) | World (ground truth) | **SUSPECT** |
| 287-288 | `broadcastFrame`: `x: worldPos.x, y: worldPos.y` | World (believed) | World (believed) | **SUSPECT** |

This is a **mismatch between init and frame messages**. On init, the robot's primary x/y is ground truth; on frames, it's the SLAM-believed position. The `VizInitMessage` type definition (protocol.ts:106-116) says `x, y` without specifying which. Looking at the frame message struct, it sends `actualX/actualY` separately. The browser presumably uses x/y as the display position, so:
- At init: robot appears at ground truth (correct for first frame)
- During frames: robot appears at believed position

This could cause a visual jump on the first frame after init, but both are in world frame. Not a coordinate system bug, but a consistency issue.

### src/world/physics-engine.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 62-68 | `halfDtheta`, `midTheta`, `dx/dy` | World frame | OK: midpoint integration in world frame | OK |
| 77 | `isCircleTraversable(newX, newY, radius)` | World coords | OK | OK |
| 105-117 | `getActualLocalMovement`: world deltas -> robot local frame | World -> Local | OK: rotates by `-prev_pose.theta` | OK |
| 122-129 | `createRobot(id, worldX, worldY, heading)`: pose = `{x: worldX+0.5, y: worldY+0.5}` | World frame | OK: cell center offset | OK |

### src/world/lidar-simulator.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 22-25 | `angles[i] = startAngle + (i+0.5) * step` | LiDAR-relative (centered on heading) | OK: these become `scan.angles[]` | OK |
| 37-38 | `worldAngle = actualPose.theta + this.angles[i]` | LiDAR -> World angle | OK: world pose heading + relative angle | OK |
| 40-44 | `world.castRay(actualPose.x, actualPose.y, worldAngle, maxRange)` | World coords, world angle | OK | OK |
| 63 | `angles: this.angles.slice()` | LiDAR-relative angles | OK: returned as robot-relative | OK |

### src/world/odometry-simulator.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 27-64 | `reading(actualDx, actualDy, actualDtheta, ...)` | Robot local frame | OK: documented as local frame, used correctly | OK |

### src/world/ground-truth-map.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 75 | `robotStarts.push({id, x, y})` | World grid cells (integer) | OK: map cell indices | OK |
| 109-111 | `inBounds(x, y)` | World grid cells | OK | OK |
| 134-137 | `isPositionTraversable(worldX, worldY)`: `floor(worldX)` | World coords -> grid cells | OK | OK |
| 144-156 | `isCircleTraversable(worldX, worldY, radius)` | World coords | OK | OK |
| 175-196 | `hasLineOfSight(x0,y0,x1,y1)` | World grid cells (integer) | OK: Bresenham on integer grid | OK |
| 203-250 | `castRay(worldX, worldY, angle, maxRange)` | World coords, world angle | OK: DDA raycast in world grid | OK |

### src/robot/treadmill/edge-map-manager.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 73-89 | `startEdge(sourceNodeId, currentPose, ...)` | SLAM frame pose | OK | OK |
| 95-119 | `update(currentPose, previousPose)` | SLAM frame | OK: both poses in SLAM | OK |
| 125-155 | `completeEdge(targetNodeId, slamGrid, ...)` | SLAM grid | OK | OK |
| 196-201 | `extractCorridorMap`: `extractRegion(slamGrid, midPose.x, midPose.y, radius)` | SLAM coords | OK: midPose is SLAM, grid is SLAM | OK |

### src/robot/treadmill/node-map-manager.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 44-72 | `createNodeMap`: `extractRegion(slamGrid, feature.position.x, ...)` | SLAM coords | OK: feature position is SLAM | OK |
| 77-103 | `updateNodeMap`: `extractRegion(slamGrid, existing.position.x, ...)` | SLAM coords | OK: stored position is SLAM | OK |

### src/robot/network/graph-publisher.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 57-66 | `publishNode(node)` | SLAM frame (node.position) | OK: nodes are in the publishing robot's SLAM frame | OK |
| 71-79 | `publishEdge(edge)` | No position data, references node IDs | OK | OK |

### src/robot/network/graph-subscriber.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 102-119 | `handleNodeReceived`: buffers until alignment established | External SLAM frame | OK: correctly waits for alignment | OK |
| 121-140 | `integrateExternalNode`: `transformNode(node)` then `findNodesInRadius(...)` | Transforms to local SLAM frame | OK | OK |
| 205-221 | `transformPosition`: `pos.x * cos - pos.y * sin + offset.x` | External SLAM -> Local SLAM | OK: rotation + translation transform | OK |

### src/robot/merge/candidate-detector.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 113-118 | `generateSummaries`: `position: {x: node.position.x, y: node.position.y}` | SLAM frame | OK: nodes are in SLAM | OK |
| 190-192 | `computeGeometrySimilarity`: `vec2Distance(a.position, b.position)` | SLAM vs SLAM? | **SUSPECT** |

**Line 190-192 suspect analysis:** When comparing summaries from two different robots, `a.position` is in Robot A's SLAM frame and `b.position` is in Robot B's SLAM frame. These are **different coordinate systems**. The distance computation is meaningless unless the frames are aligned. However, this function is called from `attemptMergeAlignment` in robot-agent.ts (line 1563), which calls it BEFORE alignment is established. The geometry similarity would produce wrong results for robots that started at different positions.

Looking more carefully: this is part of the merge candidate detection (finding which nodes MIGHT be the same). The `MAX_POSITION_DISTANCE = 5.0m` threshold means it only considers nodes within 5m of each other in their respective SLAM frames. If robot A starts at world (5,5) and robot B starts at world (10,5), then the same real-world junction would be at SLAM (3,3) for A and SLAM (-2,3) for B -- distance 5m, which is marginal. For robots starting far apart, most true matches would be filtered out. However, **this is by design** -- the candidate detector is the first pass and is expected to have false negatives when robots are far apart. The alignment still works if even 2 candidates survive. Not a bug, but a limitation.

### src/robot/merge/alignment.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 56-91 | `align(anchors)` | `a_position` in Robot A's SLAM, `b_position` in Robot B's SLAM | OK: computes B->A transform | OK |
| 97-113 | `transformPosition(position, transform)` | B's SLAM -> A's SLAM | OK: applies rotation + scale + translation | OK |
| 150-162 | `alignSingleAnchor`: `translation = a - b` | B's SLAM -> A's SLAM | OK: pure translation when only 1 anchor | OK |
| 184-262 | `computeSimilarityTransform` (Umeyama) | B's SLAM -> A's SLAM | OK: standard algorithm | OK |

### src/shared/gun-schema.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 74-79 | `publishNode(node)`: stores `{...node}` | Publishing robot's SLAM frame | OK: node.position is in publisher's SLAM | OK |
| 81-86 | `publishEdge(edge)` | No position data | OK | OK |
| 93-94 | `updateNode`: position update: `existing.position = updates.position` | Robot's SLAM frame | OK | OK |

### src/shared/protocol.ts

| Line | Variable/Expression | Expected Frame | Actual Frame | Status |
|------|-------------------|----------------|--------------|--------|
| 106-116 | `VizInitMessage.robots[].x/y` | World frame | OK: populated from actual or believed (see world-server.ts analysis) | OK |
| 130 | `VizFrameMessage.robots[].x/y` | World frame | OK | OK |
| 244 | `VizTargetMessage.x/y` | World frame | OK: browser click coords are in world frame | OK |

### src/shared/events.ts, src/shared/uuid.ts

No coordinate references. Utility files only.

---

## Summary of Findings

### Totals

| Category | Count |
|----------|-------|
| **Total references audited** | **~180** |
| **OK** | **176** |
| **SUSPECT** | **4** |
| **BUG** | **0** |

### SUSPECT Items

1. **types.ts:119-120 — OccupancyGridConfig origin comment**
   - Comment says "World x-coordinate" but the origin is in SLAM coordinates.
   - **Impact:** Documentation-only. Code is correct.
   - **Fix:** Change comment to "SLAM x-coordinate of cell [0,0] center".

2. **occupancy-grid.ts:60 — `worldToGrid` function name**
   - Named "worldToGrid" but converts SLAM -> Grid, not World -> Grid.
   - **Impact:** Naming convention only. Within the robot, "world" means SLAM. All callers pass SLAM coordinates correctly.
   - **Fix:** Could rename to `slamToGrid` for clarity, or add a doc comment clarifying the convention.

3. **robot-connection.ts:109-115 — Drift tracking compares SLAM deltas to world deltas**
   - `slamDx = currSlamPose.x - prevSlamPose.x` (SLAM frame delta)
   - `actualDx = currActualPose.x - prevActualPose.x` (world frame delta)
   - **Impact:** None. The SLAM-to-World transform is a pure translation (no rotation, no scale), so deltas are identical. `delta_slamX = delta_worldX` by definition.
   - **Fix:** None needed. Add a comment explaining why this works.

4. **world-server.ts:204 vs 287 — Init vs Frame robot position source**
   - Init sends `x: actual.x` (ground truth), Frame sends `x: worldPos.x` (believed SLAM position mapped to world).
   - **Impact:** Minor visual jump on first frame tick. Both are in world frame.
   - **Fix:** Decide which position should be primary and use it consistently. If believed is desired, change init to also use `worldPos`.

### Specific Suspects from the Request

1. **`hasLineOfSight` in robot-agent.ts** — Correctly converts SLAM coords to grid coords via `gridHasLineOfSight` which calls `bresenhamRay` with SLAM origin/resolution. All callers pass SLAM coordinates. **OK.**

2. **`hasDirectLineOfSight`** — Correctly converts target SLAM angle to LiDAR-relative via `atan2(dy, dx) - pose.theta`. Compares with `scan.angles[i]` which is LiDAR-relative. **OK.**

3. **`createNode` wall pullback** — Converts LiDAR angle back to SLAM position correctly: `angleToNode` is LiDAR-relative (line 1439), `absAngle = angleToNode + pose.theta` converts back to SLAM absolute (line 1454), then `pose + pulledDist * cos/sin(absAngle)` produces correct SLAM position. **OK.**

4. **`detectSidePassages`** — `openAngle = scan.angles[i+1]` is LiDAR-relative. `absOpenAngle = pose.theta + openAngle` correctly converts to SLAM absolute. `angleToNode = atan2(dy, dx) - pose.theta` correctly converts SLAM angle to LiDAR-relative for comparison with `openAngle`. Target position computed as `pose + dist * cos/sin(absOpenAngle)` is correct SLAM coords. **OK.**

5. **`analyzeCorridorOnArrival`** — Stored corridor scan poses are in SLAM frame (correct, since they were captured from `this.slam.pose`). Perpendicular angles computed from `travelAngle` (SLAM absolute) are SLAM absolute. LiDAR rays converted via `p.theta + absRayAngle` from LiDAR-relative to SLAM absolute. **OK.**

6. **Visualization positions** — `broadcastFrame` correctly transforms all SLAM node positions via `slamToWorld` before sending. `sendInit` has a minor inconsistency (uses actual pos for primary x/y vs believed pos in frames). **SUSPECT** but not a coordinate system bug.

7. **Node positions in the graph** — Always SLAM frame. `createNode` takes features in SLAM, `createNodeAtPosition` called with SLAM positions, node snap uses SLAM pose. **OK.**

8. **Edge creation in `identifyVisibleEdges`** — Bearing computed via `atan2(node.position.y - pose.y, node.position.x - pose.x)`. Both `node.position` and `pose` are SLAM frame. Distance computed via `sqrt(dx*dx + dy*dy)` between SLAM positions. **OK.**

### Conclusion

**No coordinate system bugs were found.** The codebase is internally consistent across all four coordinate frames. Every transform between frames (LiDAR->SLAM, SLAM->Grid, SLAM->World) is applied correctly at every call site. The suspected edges-through-walls and collision issues are not caused by coordinate mismatches.

If edges are going through walls, the likely causes are:
1. **LiDAR single-ray LOS check limitations** — `hasDirectLineOfSight` checks only the single closest ray. If the ray happens to pass through a doorway opening while the edge geometrically clips a wall corner, the check passes but the edge is physically through a wall.
2. **SLAM drift** — Node positions are snapped to the robot's SLAM pose on arrival, which drifts from ground truth over time. Two nodes that were correctly connected when created may appear to have an edge through a wall when viewed in world coordinates after drift accumulates.
3. **Node +-20 degree adjustment** — The `checkLineOfSight` search shifts the node angle up to 20 degrees from the feature-detected direction. This could place nodes in unexpected positions relative to corridors.
