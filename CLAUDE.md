# SLAM Explorer — Multi-Robot Warehouse Mapping

## Project Overview

Multi-robot exploration simulation where robots use LiDAR SLAM to map unknown warehouses, building a topological graph (nodes = places, edges = connections) as they explore. Robots do NOT share information except through the Gun.js protocol (CRDT observation-level merge).

## Implementation Workflow

This workflow applies to any task that changes code (new features, bug fixes, refactoring). Skip it for questions, exploration, or backlog review — handle those directly.

### Phase 1: Plan & Scope (in main thread)

1. Break the task into clear, actionable subtasks
2. Identify which files / modules are likely to be affected
3. Define acceptance criteria — what does "done" look like?
4. Identify risks, dependencies, and edge cases
5. Describe WHAT, not HOW

### Phase 2: Design (for anything non-trivial; in main thread)

1. Describe the design in chat — Problem / Approach / Alternatives / Risks
2. Get user signoff before code lands. Don't write design memos to disk unless explicitly asked.

### Phase 3: Implement (delegate to `robochickens-coder`)

1. Pass the agent: task description, files in scope, acceptance
   criteria, any relevant project constraints
2. The agent commits in small chunks with clear messages

### Phase 4: Test (delegate to `robochickens-tester`)

1. Pass the agent: what changed, what acceptance criteria need
   verification, which focus areas apply
2. The agent runs verification matching the project's stage and
   reports pass/fail with evidence

### Phase 5: Review (delegate to `robochickens-reviewer`)

1. Pass the agent: nothing — it pulls the diff itself
2. The agent surfaces issues by severity. Critical issues block
   merge; design concerns route back to `robochickens-coder`

### Phase 6: Iterate

1. If review surfaces issues, route feedback back to `robochickens-coder`
2. Re-run testing and review after fixes
3. Continue until tests pass and review approves

### Phase 7: Commit / Merge / PR (in main thread)

1. Commit / merge in small chunks with clear messages
2. For day+ items or anything risky, open a PR with
   Summary / Risk / Test plan / Rollback
3. Update this `CLAUDE.md` if the change altered tech stack, key
   files, workflow, or domain vocabulary

## Key Architecture

### Exploration Cycle (4 phases, 5 logical steps)

1. **ARRIVING** — Robot reaches target node (or creates starting node on tick 1)
2. **SCANNING** — 5-tick pause, then feature detection + node/edge creation
3. **CHOOSING** — Pick next edge via exploration priority (manual path > unexplored > untraveled > unmapped neighbors > random)
4. **TRAVELING** — Drive to destination, collect corridor scans for retroactive junction detection

### Critical Files

| File | Purpose |
|------|---------|
| `src/robot/robot-agent.ts` | Main exploration loop, node creation, LOS checks |
| `src/robot/network/network-graph.ts` | Topological graph, exploration state, pathfinding |
| `src/robot/feature-detection/feature-detector.ts` | LiDAR scan analysis, corridor direction detection |
| `src/robot/navigation/motor-controller.ts` | Wall avoidance, steering, corridor centering |
| `src/robot/slam/particle-filter.ts` | 30-particle SLAM, scan matching at node arrivals only |
| `src/robot/slam/inverse-sensor-model.ts` | Occupancy grid updates with wall thickening |
| `src/world/world-server.ts` | Simulation server, ground truth, visualization |
| `public/index.html` | Browser UI, topology graph, map editing |

### Key Rules (hard-won, do NOT violate)

- **Node creation is atomic with edge creation** — if edge fails, remove the node. No orphans.
- **Edge LOS uses direct single ray** (`hasDirectLineOfSight`) — no +/-20 degree search
- **Node placement uses +/-20 degree search** (`checkLineOfSight`) — find best wall clearance, verified with `hasDirectLineOfSight` after adjustment
- **Node wall pullback** — if LiDAR sees a wall closer than the proposed node, pull node back to (wall_dist - 0.5m). Minimum 1.0m from robot.
- **LiDAR-only node validation** — no SLAM grid checks for node creation. Grid has gaps and thickening artifacts. LiDAR is ground truth.
- **Grid-based LOS (`hasLineOfSight`)** — uses Bresenham walk on SLAM grid. Used for edge checks between arbitrary positions (not just from robot).
- **First tick uses exact SLAM origin (0,0,0)** — no particle filter noise, no grid population
- **Scan matching DISABLED during travel** — only runs at node arrivals (`refinePoseAtNode`)
- **Blocked edges are kept, not deleted** — they represent real topology that may reopen
- **Traveled/explored entities are never pruned**
- **Robots do NOT share info except through Gun protocol**
- **6 edges max per scan, 15 degree angular occlusion filter**
- **Freshness decay uses sim-minutes** (60,000ms), not real hours

### Removed Subsystems (dead code, cleaned 2026-05-25)

These were designed but never wired in, or solved problems that don't exist with zero noise:
- ConfidenceScorer, SpatialIndex, ObservationLogManager, DStarLite
- ExplorationStrategy, RegressionController, GraphMerger, PathPlanner
- LoopClosureDetector, DriftCorrector, ConsistencyEngine

### Known Issues

- 0 errors across all 10 maps (as of 2026-05-25)
- 18 warnings: starting nodes lack SLAM data (tick 1 skips processSensorData)
- Corner pathing (waypoint nodes at corners) not yet implemented

### Diagnostics

Run `node run-diagnostics.js` to headlessly test all 10 maps at 200 ticks. Checks: nodes OOB, nodes in walls, starting node drift, edges through walls, orphaned nodes, robot stuck, explored nodes without SLAM data, long edges, 0-edge nodes, duplicate edges.
