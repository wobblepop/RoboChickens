# SLAM Explorer — Changelog

## 2026-05-25: Major Rebuild

### New Robot Exploration Algorithm
- Replaced old SCANNING/PLANNING/TRAVELING 3-phase model with 7-step exploration cycle
- Robots only create nodes at features (turns, junctions, dead ends, side openings) — not in straight corridors
- Edges created only via LiDAR line-of-sight from the robot's current position (minimum range in cone, not average)
- Mid-edge scanning at 50% of travel distance discovers nodes and edges from corridor midpoints
- Exploration priority: unexplored adjacent nodes > untraveled edges > nodes with most unmapped edges > random
- Manual targeting uses BFS through the graph, hop-by-hop with full scan cycle at each node

### Missing Subsystems Implemented
- **Observation Log** — hot/warm/cold tiering with compaction and archival
- **CRDT Merge** — observation-level set-union semantics (replaced Object.assign property overwrites)
- **Spatial Index** — quadtree for O(log n + k) spatial queries
- **Confidence Scorer** — 5-factor multiplicative model with all tiers
- **Consistency Engine** — 5 topological checks (degree, distance, angular, crossing, region)
- **Regression Controller** — cascade management with quarantine and rollback
- **Merge Protocol** — candidate detection, Umeyama alignment, 6-phase graph merger
- **D* Lite Pathfinding** — incremental replanner with multi-factor cost function
- **Gun.js Network Adapter** — real P2P communication layer
- **Drift Corrector** — map correlation, loop closure, neighbor propagation

### Edge/Node Lifecycle
- Traveled edges that become invisible are marked "blocked" — never deleted
- Explored nodes are never pruned — they represent confirmed locations
- Only speculative (never-confirmed) entities can be pruned
- Freshness decay respects stability classes (static features never decay)
- Edge revalidation: blocked edges reopen if the path becomes visible again

### Bug Fixes
- Degenerate particle filter defaults (was 1 particle, zero noise — now 30 particles, realistic noise)
- Bearing change formula (was always PI — now computed from actual heading change)
- Loop closure heading (now tests 4 rotations instead of using current heading)
- Node selection in subscriber (picks closest, not arbitrary first)
- Confidence uses diminishing returns curve instead of flat +0.05
- Edge corridor map lookup (ID mismatch between edge manager and graph)
- Robot wall clipping (radius increased from 0.05m to 0.25m)
- Robot trail desync (both position and trail now use SLAM-to-world transform)
- Topology graph jumping (now uses fixed map bounds instead of auto-fitting)
- Starting node crash (particle filter guards against empty particle array)

### Frontend
- Topology graph uses fixed world coordinates (no more jumping on node addition)
- Explored nodes shown as filled circles, unexplored as hollow rings
- Traveled edges shown as solid lines, untraveled as dashed
- Blocked edges shown as red dashed lines
- Node popup: SLAM map at top (fixed), details scrollable below
- Edge popup: corridor SLAM data displayed for traveled edges
- Map editing: wall/obstacle/erase changes update the server's ground truth
- Legends start minimized, click to expand
- Tier colors use canonical boundaries (Dead/Candidate/Provisional/Committed/Anchored)

### Test Suite
- 77 tests across 5 files: spatial index, observation log, confidence scorer, consistency engine, network graph
