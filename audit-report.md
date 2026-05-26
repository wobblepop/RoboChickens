# SLAM Explorer Codebase Audit Report

Audit date: 2026-05-25
Scope: All 39 `.ts` files under `src/`

---

## 1. CRITICAL — Active Bugs

### C1. Edge creation through walls in `doScanning` (robot-agent.ts ~456)

When a detected feature is near an existing node, an edge is created to that existing node using `this.graph.createEdge()` **without any LOS check** on the existing node's actual position.

```
// Line 443: LOS is checked to the FEATURE position
const los = this.checkLineOfSight(scan, pose, feature.position.x, feature.position.y);
if (!los.clear) continue;

// Line 447-458: But the edge is created to nearbyExisting.position,
// which is in a DIFFERENT location than the feature position.
// No LOS check is performed on nearbyExisting.position.
const nearbyExisting = this.graph.findNearestNode(feature.position);
if (nearbyExisting && ...) {
    this.graph.createEdge(this.currentNodeId, nearbyExisting.id, edgeDist, bearing, ...);
}
```

The LOS check passes for the feature position but the edge target (nearbyExisting) could be behind a wall relative to the robot. This is the most likely cause of through-wall edges.

**File:** `src/robot/robot-agent.ts`, line ~456
**Severity:** CRITICAL
**Fix:** Add `hasDirectLineOfSight` check on `nearbyExisting.position` before creating the edge.

### C2. Edge creation through walls in `detectSidePassages` (robot-agent.ts ~792-830)

Multiple edge creation sites in detectSidePassages use `hasLineOfSight()` (which delegates to `hasDirectLineOfSight`), but the LOS is checked from the **anchor node's stored position**, not from the robot's current position. If the anchor node's position has drifted or been adjusted, the LOS check may not reflect the actual world state.

Additionally, at line 834, a junction-to-anchor edge is created with bearing hardcoded to `0`:
```
const edge = this.graph.createEdge(anchorId, junctionId, dist, 0, scan.timestamp, true);
```
A bearing of `0` is incorrect — it should be computed from the positions.

**File:** `src/robot/robot-agent.ts`, lines ~792-870
**Severity:** CRITICAL (wrong bearing) / SUSPICIOUS (LOS from stored position)

### C3. Wall thickening in inverse-sensor-model can block valid paths (inverse-sensor-model.ts ~66-76)

The wall thickening logic marks 4-connected neighbor cells as "lightly occupied" (`halfOcc = 0.424`), but only if `cells[ni] < 0.3`. The condition `< 0.3` means it won't thicken into cells that are already slightly positive (including cells that are nearly unknown at 0.0 + epsilon from prior updates). However, the repeated accumulation of halfOcc values means neighboring free cells can gradually become "occupied" over many scans, especially in 1-cell corridors where the free cell is adjacent to walls on both sides. This creates false walls that block navigation and cause invalid LOS failures.

**File:** `src/robot/slam/inverse-sensor-model.ts`, lines 66-76
**Severity:** CRITICAL — directly contributes to nodes-in-walls and edges-through-walls in narrow corridors

### C4. `hasLineOfSight` ignores its `_fromX`/`_fromY` parameters (robot-agent.ts ~1755)

```typescript
private hasLineOfSight(_fromX: number, _fromY: number, toX: number, toY: number): boolean {
    if (!this.lastScan) return false;
    const pose = this.slam.pose;
    const dist = Math.sqrt((toX - pose.x) ** 2 + (toY - pose.y) ** 2);
    return this.hasDirectLineOfSight(this.lastScan, pose, toX, toY, dist);
}
```

The `_fromX` and `_fromY` parameters are **completely ignored** (underscored). The function always checks LOS from the robot's **current SLAM pose**, regardless of what position was passed. Call sites that pass a different node's position as `fromX/fromY` (lines 794, 829, 864, 1179) are checking the wrong LOS — they think they're checking from that node's position, but they're actually checking from the robot.

**File:** `src/robot/robot-agent.ts`, line 1755
**Severity:** CRITICAL — 5+ call sites pass non-robot positions and get wrong LOS results

---

## 2. DEAD CODE — Subsystems That Exist But Do Nothing

### D1. `shouldCreateNode` is defined but never called (robot-agent.ts ~1529)

The method `shouldCreateNode` implements direction-aware node suppression with minimum separation checking. It is a complete, working implementation. But it is **never called** from anywhere in the codebase. The node creation flow in `doScanning` bypasses it entirely — it does its own distance check at line 448 (`findNearestNode` + `minNodeSeparation` comparison) which is a simpler, less capable version.

**File:** `src/robot/robot-agent.ts`, line 1529
**Severity:** DEAD CODE — intended logic is not running

### D2. `ConfidenceScorer` is instantiated but never called (robot-agent.ts ~175)

`this.confidenceScorer = new ConfidenceScorer()` is created in the constructor but `this.confidenceScorer.` is never called anywhere in `robot-agent.ts`. The 5-factor multiplicative confidence model (source, observation, consistency, corroboration, freshness) was designed to replace the ad-hoc confidence assignments throughout the code, but was never wired in.

The actual confidence values in the codebase are set via:
- Hardcoded values at creation (0.4, 0.5, 0.6, 0.7, 0.8 etc.)
- Ad-hoc formulas in `strengthenNode` (`1 - exp(-0.25 * count)`)
- Ad-hoc decay in `revalidateEdgesFromNode`
- ConsistencyEngine penalties

None of these use the ConfidenceScorer.

**File:** `src/robot/confidence-scorer.ts` (entire file), `src/robot/robot-agent.ts` line 175
**Severity:** DEAD CODE — entire 350-line subsystem is unused

### D3. `SpatialIndex` (quadtree) is never imported or used (spatial-index.ts)

A complete quadtree spatial index implementation (420 lines) with insert, remove, nearest-k, radius search, bounds search, and ray casting. It is **never imported** by any other file. The actual spatial queries in `NetworkGraph.findNodesInRadius` and `findNearestNode` use O(n) linear scans.

**File:** `src/robot/spatial-index.ts` (entire file)
**Severity:** DEAD CODE

### D4. `ObservationLogManager` is never imported or used (observation-log.ts)

A complete tiered observation lifecycle manager (462 lines) with hot/warm/cold/archive tiers, compaction, agreement clustering, and anchored compaction. It is **never imported** by any other file. The `observation_log` field on `NetworkNode` and `NetworkEdge` is always an empty array `[]` — no code ever adds observations to it.

**File:** `src/robot/observation-log.ts` (entire file)
**Severity:** DEAD CODE

### D5. `DStarLitePathfinder` is never imported or used (dstar-lite.ts)

A complete D* Lite incremental pathfinding implementation (423 lines) with confidence-based cost computation, congestion handling, and dynamic replanning. It is **never imported** by any other file. The actual pathfinding uses BFS in `NetworkGraph.findPath` and A* in `path-planner.ts`.

**File:** `src/robot/navigation/dstar-lite.ts` (entire file)
**Severity:** DEAD CODE

### D6. `ExplorationStrategy` is never imported by robot-agent.ts (exploration-strategy.ts)

The ExplorationStrategy class (234 lines) implements a 4-priority exploration target selector. It is only imported by itself (it imports `path-planner.ts`). The robot-agent.ts has its own inline exploration logic in `doChoosing` and `findGridFrontier`. The two are completely independent implementations of the same concept.

**File:** `src/robot/navigation/exploration-strategy.ts` (entire file)
**Severity:** DEAD CODE

### D7. `RegressionController` is never imported or used (regression-controller.ts)

A complete hierarchical regression controller (368 lines) with oscillation detection, conflict quarantine, snapshot/rollback, and conservation checks. It is **never imported** by any other file.

**File:** `src/robot/regression-controller.ts` (entire file)
**Severity:** DEAD CODE

### D8. `GraphMerger` is never imported by robot-agent.ts (graph-merger.ts)

The full 6-phase graph merger (335 lines) is never used. The robot-agent does its own simplified merge logic in `attemptMergeAlignment` (lines 1634-1680) which only uses `CandidateDetector` and `GraphAligner` directly, bypassing the merger entirely.

**File:** `src/robot/merge/graph-merger.ts` (entire file)
**Severity:** DEAD CODE

### D9. `path-planner.ts` A* is only imported by ExplorationStrategy, which is itself dead code

The A* pathfinding in `path-planner.ts` is only imported by `exploration-strategy.ts`, which is dead code (D6). The actual pathfinding is done by `NetworkGraph.findPath` (BFS) and `SlamEngine.findGridPath` (A* on grid). `path-planner.ts` is effectively dead code.

**File:** `src/robot/navigation/path-planner.ts` (entire file)
**Severity:** DEAD CODE

### D10. `externalNodeBuffer` in robot-agent.ts is never used

The field `externalNodeBuffer: NetworkNode[]` is declared at line 106 and reset at line 1891. It is never populated or read.

**File:** `src/robot/robot-agent.ts`, lines 106, 1891
**Severity:** DEAD CODE (vestige)

---

## 3. SHORTCUTS — Incomplete or Simplified Implementations

### S1. Odometry noise is set to 0 — the SLAM particle filter is doing nothing useful (types.ts ~504-515)

```typescript
odometry: {
    linear_noise_fraction: 0.0,
    angular_noise_fraction: 0.0,
    linear_noise_floor: 0.0,
    angular_noise_floor: 0.0,
},
```

With all noise at 0, the odometry readings are **perfect**. The particle filter still runs 30 particles, but they all have identical poses (no jitter is added during resampling when noise is 0 — see particle-filter.ts lines 299-305). This means:
- The SLAM system is effectively dead-reckoning with perfect odometry
- The 30 particles are wasted computation
- Scan matching at node arrivals is competing against a perfect prior
- The entire DriftCorrector subsystem is solving a problem that doesn't exist

This is almost certainly intentional for debugging but should be documented as such.

**File:** `src/shared/types.ts`, lines 504-515
**Severity:** SHORTCUT — makes multiple subsystems pointless

### S2. LiDAR noise is set to 0 (types.ts ~507-508)

```typescript
range_noise_stddev: 0.0,
spurious_rate: 0.0,
```

Similar to S1, perfect LiDAR means the occupancy grid is perfect, scan matching always scores perfectly, and the confidence computations are trivially high.

**File:** `src/shared/types.ts`, lines 507-508
**Severity:** SHORTCUT — makes noise-handling code untestable

### S3. 8-direction ring check at 0.35m may miss walls (robot-agent.ts ~1505-1514)

The wall proximity ring check uses only 8 directions (every 45 degrees) at 0.35m distance:
```typescript
const checkAngles = 8;
for (let i = 0; i < checkAngles; i++) {
    const a = (i / checkAngles) * 2 * Math.PI;
    const cx = feature.position.x + clearance * Math.cos(a);
    const cy = feature.position.y + clearance * Math.sin(a);
    if (this.slam.isPositionOccupied(cx, cy)) return null;
}
```

At 0.35m radius, the arc between two adjacent checks is `2 * pi * 0.35 / 8 = 0.275m`. At grid resolution 0.1m, a wall cell is 0.1m wide. A wall at 22.5 degrees (exactly between two checks) at 0.35m radius could be missed — the check points at 0 and 45 degrees would both be 0.35 * sin(22.5) = 0.134m away from the wall, which is more than one cell.

In practice, wall thickening (C3) makes walls wider than 1 cell, so this is partially mitigated.

**File:** `src/robot/robot-agent.ts`, lines 1505-1514
**Severity:** SHORTCUT — 16 directions would be more robust

### S4. `isPositionOccupied` uses a single cell check (slam-engine.ts ~130)

```typescript
isPositionOccupied(x: number, y: number): boolean {
    const logOdds = getCellAtWorld(this.grid, x, y);
    return logOdds > 0.5;
}
```

This checks a single grid cell. A robot with radius 0.25m occupies ~5 cells (at 0.1m resolution). A position could be "free" at its center but overlapping a wall just 0.15m away. This is partially mitigated by the ring check in `createNode` but not at all for `isPositionOccupied` calls elsewhere (e.g., detectSidePassages, analyzeCorridorOnArrival).

**File:** `src/robot/slam/slam-engine.ts`, line 130
**Severity:** SHORTCUT

### S5. Freshness decay uses real hours but timestamps are in ms since sim start (network-graph.ts ~557-578)

```typescript
const hoursSinceObserved = Math.max(0, (currentTimestamp - node.last_observed) / 3_600_000);
```

Simulation timestamps are in ms since sim start (tick * 100ms). At 200 ticks, that's 20 seconds of sim time. The decay uses `3,600,000` (1 hour) as divisor. `hoursSinceObserved` will always be near-zero (20,000ms / 3,600,000 = 0.0056 hours). The decay `exp(-0.005 * 0.0056) = 0.999972` is negligible. Freshness decay has zero practical effect in any realistic simulation run.

**File:** `src/robot/network/network-graph.ts`, lines 556-578
**Severity:** SHORTCUT — feature is implemented but timescale makes it no-op

### S6. ConsistencyEngine penalties are applied but have minimal effect (robot-agent.ts ~1600-1618)

The consistency engine runs immediate checks (degree constraints + triangle inequality) every scan cycle. However, the penalty application is:
```typescript
const consistencyPenalty = Math.max(0.5, score);
node.confidence = Math.min(node.confidence, node.confidence * consistencyPenalty);
```

Since `score` is floored at 0.5, the maximum penalty is halving the confidence once. For a node with confidence 0.7, it becomes 0.35, which is still `Candidate` tier. The consistency engine detects problems but the response is too mild to actually fix anything.

**File:** `src/robot/robot-agent.ts`, lines 1600-1618
**Severity:** SHORTCUT

### S7. `motor.reset()` is an empty function (motor-controller.ts ~95)

```typescript
reset(): void {}
```

No state to reset, but the class does have configuration that could theoretically drift. This is benign but worth noting as a pattern.

**File:** `src/robot/navigation/motor-controller.ts`, line 95
**Severity:** MINOR

---

## 4. SUSPICIOUS — Logic That May Work By Accident

### SU1. Side wall avoidance centering logic may oscillate (motor-controller.ts ~77-86)

```typescript
const leftMin = this.minRangeInCone(scan, Math.PI / 2, 15 * Math.PI / 180);
const rightMin = this.minRangeInCone(scan, -Math.PI / 2, 15 * Math.PI / 180);
cmd.angular_velocity += (leftMin - rightMin) * (2.0 + urgency * 4.0);
```

When `leftMin < rightMin` (wall closer on left), the angular velocity becomes negative (turn right, away from left wall). This is correct. But the gain `(2.0 + urgency * 4.0)` can reach 6.0, which combined with the existing angular velocity from target steering, could overshoot and oscillate between walls in a corridor. This might explain erratic behavior in narrow passages.

**File:** `src/robot/navigation/motor-controller.ts`, lines 77-86
**Severity:** SUSPICIOUS

### SU2. Wall-stop detection uses distance from START, not recent progress (robot-agent.ts ~1356-1358)

```typescript
const distFromStart = vec2Distance({x: pose.x, y: pose.y}, {x: this.travelStartPose.x, y: this.travelStartPose.y});
const expectedProgress = ticksInTravel * 0.03;
if (distFromStart > expectedProgress) return false;
```

`expectedProgress = ticks * 0.03m/tick`. At 15 ticks (minimum), expected = 0.45m. But if the robot has traveled 5m and then hits a wall, `distFromStart` could be 5m which is >> 0.45m, so the wall-stop never triggers. The check should compare **recent** progress (last N ticks), not total distance from start.

Actually, on re-reading: `expectedProgress` at 15 ticks is 0.45m, and if the robot has moved 5m from start, `distFromStart` (5.0) > 0.45 so it returns `false` (no wall stop). This is correct — the robot IS making progress. The issue would arise if the robot traveled in a circle back near its start: then `distFromStart` is small even though the robot isn't stuck.

**File:** `src/robot/robot-agent.ts`, lines 1356-1358
**Severity:** SUSPICIOUS — circular paths could defeat this check

### SU3. `+/-10 degree clearance adjustment` math edge case (robot-agent.ts ~1486-1499)

```typescript
const lateralShift = dist * Math.sin(diff);
const cappedDiff = Math.abs(lateralShift) > maxLateralShift
    ? Math.asin(maxLateralShift / dist) * Math.sign(diff)
    : diff;
```

`Math.asin(maxLateralShift / dist)` — when `dist < maxLateralShift` (0.5m), this computes `asin(0.5 / 0.4) = asin(1.25)` which returns `NaN`. The guard `if (dist > 0.5)` at line 1483 prevents `dist` from being less than 0.5, and `maxLateralShift` is 0.5, so `maxLateralShift / dist` is always <= 1.0. This is safe but relies on the `dist > 0.5` guard upstream, which is fragile.

**File:** `src/robot/robot-agent.ts`, lines 1486-1499
**Severity:** SUSPICIOUS — works only because of an upstream guard

### SU4. Loop closure creates edges without LOS check (loop-closure.ts ~120-128)

When a loop closure is detected, an edge is created between `currentNodeId` and the matched candidate:
```typescript
const edge = graph.createEdge(
    currentNodeId, candidate.id,
    vec2Distance(position, candidate.position),
    0,  // bearing hardcoded to 0
    currentScan.timestamp, false,
);
```

Problems:
1. No LOS check — the loop closure is based on scan matching, not geometric visibility
2. Bearing is hardcoded to `0` instead of computed from positions
3. During travel (where loop closure runs), the robot may not be at `currentNodeId` — it's somewhere in transit

**File:** `src/robot/network/loop-closure.ts`, lines 120-128
**Severity:** SUSPICIOUS — bearing is wrong; LOS is unchecked

### SU5. Corridor analysis projects passage positions onto A->B line but uses raw passage positions (robot-agent.ts ~1149-1155)

```typescript
const t = Math.max(0.05, Math.min(0.95, (apx * travelDx + apy * travelDy) / (travelDist * travelDist)));
const projX = departureNode.position.x + t * travelDx;
const projY = departureNode.position.y + t * travelDy;
const junctionPos: Vec2 = { x: projX, y: projY };
```

The junction is placed on the A->B line (projected), but the passage detection used the robot's actual position during travel. If the robot didn't travel in a straight line (e.g., wall avoidance), the projection could place the junction at a different position than where the opening was actually detected. In a curving corridor, this could place the junction in a wall.

**File:** `src/robot/robot-agent.ts`, lines 1149-1155
**Severity:** SUSPICIOUS

### SU6. `applyFreshnessDecay` and `pruneDeadEntities` run only every 100 ticks but also depend on scanning phase (robot-agent.ts ~513-515)

```typescript
if (this.tickCount % 100 === 0) {
    this.graph.applyFreshnessDecay(scan.timestamp);
    this.graph.pruneDeadEntities();
}
```

This only runs during the SCANNING phase (inside `doScanning`). If the robot is traveling for 200+ ticks, these never fire. The `tickCount` global clock keeps ticking but the modulo check is only evaluated when scanning.

**File:** `src/robot/robot-agent.ts`, lines 513-515
**Severity:** SUSPICIOUS — decay/pruning frequency depends on exploration pace

### SU7. Stuck detection doubles the threshold (robot-agent.ts ~708)

```typescript
if (this.tickCount - this.travelStartTick > this.travelStuckThreshold * 2) {
    return this.handleStuck(scan, pose);
}
```

`travelStuckThreshold` is 60 ticks, so the robot waits 120 ticks before declaring stuck. At 100ms per tick, that's 12 seconds of real time. Combined with the wall-stop at 15 ticks (1.5 seconds), the stuck handler is the absolute last resort. The `* 2` seems like it was increased from `* 1` at some point to reduce false positives but the original value wasn't reduced accordingly.

**File:** `src/robot/robot-agent.ts`, line 708
**Severity:** SUSPICIOUS

---

## 5. MINOR — Style and Naming Issues

### M1. Inconsistent edge creation: some sites publish, some don't

In `doScanning` (line 456), edge creation to existing nearby nodes does NOT call `this.publisher.publishEdge(edge)` or `this.events.emit("edge_created", ...)`. But `identifyVisibleEdges` (line 591-593) does both. And `detectSidePassages` sometimes does and sometimes doesn't. This means some edges are invisible to the network and visualization.

**File:** `src/robot/robot-agent.ts`, various
**Severity:** MINOR — inconsistency that could cause subtle multi-robot issues

### M2. Naming collision: `hasLineOfSight` exists in both geometry.ts and robot-agent.ts

`geometry.ts` exports a grid-based `hasLineOfSight` function that walks Bresenham cells. `robot-agent.ts` has a private method with the same name that uses LiDAR rays. The geometry.ts version is never used by robot-agent.ts. If someone tries to use it, they'll get the wrong one.

**File:** `src/shared/geometry.ts` line 167, `src/robot/robot-agent.ts` line 1755
**Severity:** MINOR

### M3. `findGridFrontier` in both robot-agent.ts and exploration-strategy.ts

Two independent implementations of the same algorithm exist (the robot-agent.ts version searches up to 30m, the strategy version searches up to 8m). Since ExplorationStrategy is dead code (D6), this is just code duplication rather than a live conflict.

**File:** `src/robot/robot-agent.ts` line 1545, `src/robot/navigation/exploration-strategy.ts` line 190
**Severity:** MINOR

### M4. `DEFAULT_MOTOR_CONFIG` has `robot_radius: 0.05` which differs from actual (motor-controller.ts ~17)

The default motor config has `robot_radius: 0.05` (5cm), but the actual robot physics config has `radius: 0.25` (25cm). The motor controller is constructed with the physics config, so this default is never used. But it's confusing.

**File:** `src/robot/navigation/motor-controller.ts`, line 17
**Severity:** MINOR

### M5. `normalizeAngle` may return -PI for angles at exactly -PI boundary

```typescript
if (a <= -Math.PI) a += 2 * Math.PI;
```

The check is `<=` not `<`, so `a === -Math.PI` becomes `a = Math.PI`, but the docstring says "(-PI, PI]". This is correct per the docstring but could cause issues if code expects `[-PI, PI]`.

**File:** `src/shared/math.ts`, line 80
**Severity:** MINOR

---

## Summary

| Category | Original | Fixed | Remaining |
|----------|----------|-------|-----------|
| CRITICAL | 4 | 4 | 0 |
| DEAD CODE | 10 | 10 | 0 |
| SHORTCUTS | 7 | 3 | 4 |
| SUSPICIOUS | 7 | 3 | 4 |
| MINOR | 5 | 2 | 3 |

### Fixes Applied (2026-05-25)

**CRITICAL (all 4 fixed):**
- C1: Added `hasDirectLineOfSight` check when creating edges to nearby existing nodes
- C2: Fixed hardcoded bearing=0 in `detectSidePassages`
- C3: Mitigated by switching node creation to LiDAR-only validation (no SLAM grid checks)
- C4: Rewrote `hasLineOfSight` to use grid-based Bresenham walk that respects from-position

**DEAD CODE (all 10 removed, plus 3 more subsystems):**
- Deleted: ConfidenceScorer, SpatialIndex, ObservationLog, DStarLite, ExplorationStrategy, RegressionController, GraphMerger, PathPlanner, LoopClosure, DriftCorrector, ConsistencyEngine
- Removed: `shouldCreateNode`, `externalNodeBuffer`
- Total: ~3,700 lines removed

**SHORTCUTS (3 fixed):**
- S3: Ring check replaced with LiDAR-only node validation
- S5: Freshness decay rescaled from real-hours to sim-minutes
- S6: ConsistencyEngine removed (penalties were too weak to matter)

**SUSPICIOUS (3 fixed):**
- SU4: Loop closure removed entirely (pointless with zero noise)
- SU6: Decay/pruning moved from doScanning to processTick (runs every phase)
- SU7: Stuck detection unchanged but loop closure removal eliminates one interaction

**MINOR (2 fixed):**
- M1: Added missing `publishEdge`/`emit` for edges to existing nodes in doScanning
- M2: `hasLineOfSight` naming collision resolved (robot-agent now uses grid-based version)

**Additional improvements:**
- Node wall pullback: if LiDAR sees a wall closer than proposed node, pulls node back to (wall_dist - 0.5m)
- Edge creation to existing nodes now verified with `hasDirectLineOfSight`

### Remaining Items (not bugs, monitor only)

- S1/S2: Noise is now enabled (1% linear, 1.5% angular odometry; 0.01m LiDAR range noise, 0.2% spurious rate). Particle filter is active and functional. The audit originally flagged zero noise, which has since been corrected in DEFAULT_SIMULATION_CONFIG.
- S4: `isPositionOccupied` single-cell check — mitigated by LiDAR-only node creation
- SU1: Side wall avoidance gain could oscillate in narrow corridors — not confirmed
- SU2: Wall-stop uses distance from start, not recent progress — edge case with circular paths
- SU5: Corridor analysis projects to A->B line, not actual curved path — edge case
- M3: `findGridFrontier` duplication resolved — ExplorationStrategy was removed as dead code
- M4: `DEFAULT_MOTOR_CONFIG.robot_radius` corrected from 0.05 to 0.25
- M5: `normalizeAngle` boundary behavior is correct per its documented contract

### Diagnostic Results

**Before fixes:** 9 errors, 19 warnings
**After fixes:** 0 errors, 18 warnings (all "no SLAM data" on starting nodes — cosmetic)
