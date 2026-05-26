# Robot Behavior Modes — Design Document

Each robot has a selectable behavior mode that controls its exploration strategy during the CHOOSING phase. The mode is changeable at runtime via a per-robot dropdown in the UI. All modes use the same ARRIVING, SCANNING, and TRAVELING phases — only the target selection logic changes.

## Modes

### 1. Explore New Areas (default)
**Purpose:** Maximize coverage of unknown territory.
**Strategy:** Current behavior — BFS toward unexplored nodes, then untraveled edges, then grid frontiers.
**When to use:** Initial deployment. Robots spread out and map unknown areas.
**Priority:**
1. Path toward nearest unexplored node (BFS through graph)
2. Nearest untraveled edge from current node
3. Neighbor with most unmapped edges
4. Grid frontier (free cell adjacent to unknown cell)
5. Random edge (fully explored fallback)

**Test criteria:**
- Coverage % should increase monotonically
- Robots should not revisit explored areas unless no frontier remains
- Multiple robots should spread out, not cluster

### 2. Recheck Map
**Purpose:** Revisit explored nodes and verify the graph is still accurate.
**Strategy:** Prioritize nodes with the OLDEST `last_explored_at` timestamp. Travel to them, do a full scan, revalidate edges. Corrects drift accumulated since last visit.
**When to use:** After initial exploration. Detects changes (moved obstacles, opened/closed doors).
**Priority:**
1. Explored node with oldest `last_explored_at` (most stale)
2. Prefer nodes with high edge count (junctions — more likely to have changed)
3. Avoid nodes explored in the last N ticks (configurable cooldown)

**Test criteria:**
- Node positions should converge toward actual positions over multiple revisits
- Stale edges should get revalidated (blocked if changed, refreshed if still open)
- Position error metric should decrease after rechecking

### 3. Consolidate Pathing
**Purpose:** Strengthen the graph by traveling untraveled edges and verifying speculative connections.
**Strategy:** Prioritize untraveled edges. For each, travel the full edge and verify it's passable. Mark as traveled or blocked based on outcome.
**When to use:** After exploration when the graph has many untraveled (speculative) edges.
**Priority:**
1. Untraveled edges sorted by confidence (lowest first — most speculative)
2. Prefer edges connecting explored nodes (both endpoints known)
3. Avoid edges already marked blocked

**Test criteria:**
- Untraveled edge count should decrease
- Traveled edge count should increase
- Blocked edges should be correctly identified (not false positives)
- Graph connectivity should improve

### 4. Check Stale Areas
**Purpose:** Focus on areas where confidence has decayed below a threshold.
**Strategy:** Find nodes/edges in the Candidate or Provisional confidence tier and revisit them. Different from Recheck Map in that it targets LOW CONFIDENCE entities, not just old ones.
**When to use:** Long-running simulation where freshness decay has weakened parts of the graph.
**Priority:**
1. Nodes in Candidate tier (confidence < 0.35)
2. Nodes in Provisional tier (confidence < 0.65)
3. Prefer nodes with blocked edges (potential for reopening)

**Test criteria:**
- Low-confidence node count should decrease after a sweep
- Confidence values should increase for revisited nodes
- Pruned (dead-tier) entities should be minimal after consolidation

### 5. Check Main Pathways
**Purpose:** Verify the most-used routes are still passable.
**Strategy:** Identify high-traffic edges (traveled by multiple robots, or traveled multiple times) and re-traverse them. Like a patrol route.
**When to use:** Ongoing operations where key corridors must stay verified.
**Priority:**
1. Traveled edges sorted by traffic count (highest first)
2. Focus on edges connecting explored junction nodes (3+ exits)
3. Skip dead-end branches unless they've changed

**Test criteria:**
- Main corridor edges should stay at high confidence
- Blocked main pathways should be detected quickly
- Robot should complete patrol loops, not get stuck in corners

### 6. Find Boxes
**Purpose:** Detect and catalog rectangular objects (obstacles, shelving, pallets) in the environment.
**Strategy:** Look for rectangular occupancy patterns in the SLAM grid. Travel to areas with detected rectangles, scan from multiple angles to refine the shape estimate. Create special "box" annotations on nodes.
**When to use:** Warehouse inventory mapping — after the layout is known, identify what's in the aisles.
**Priority:**
1. Areas with unclassified rectangular occupancy clusters
2. Scan from at least 2 perpendicular angles for shape confirmation
3. Catalog: position, approximate dimensions, orientation

**Test criteria:**
- Known obstacles ('O' cells in ground truth) should be detected
- Position estimates should be within 0.5m of actual centers
- False positives (walls misidentified as boxes) should be rare

### 7. Idle / Hold Position
**Purpose:** Robot stays at current node. Does not move.
**Strategy:** Stop motor. Continue collecting LiDAR data and updating SLAM grid at current position. Useful as a baseline or when the user wants to manually control via click-to-navigate.
**When to use:** Manual control mode, or pausing a robot while others work.

**Test criteria:**
- Robot should not move
- SLAM grid should still update from stationary scans
- Manual click-to-navigate should still work

### 8. Return to Start
**Purpose:** Navigate back to the robot's starting node via the shortest known path.
**Strategy:** BFS from current node to the starting node (first node created). Follow the graph path, doing full scan cycles at each intermediate node.
**When to use:** Recall robots to base, or test path quality back to origin.
**Priority:**
1. Shortest path (hop count) to starting node
2. Avoid blocked edges (reroute if needed)
3. Full ARRIVING cycle at each hop (corrects drift on the way back)

**Test criteria:**
- Robot should reach within 0.5m of starting position
- Position error at end should be small (drift corrected by revisiting nodes)
- If path is blocked, robot should find alternative route or report failure

## Implementation Architecture

Each mode is a function with signature:
```
(currentNodeId: string, graph: NetworkGraph, pose: Pose2D, scan: LidarScan) => { edge: NetworkEdge; reason: string } | null
```

The `doChoosing` method selects which function to call based on the robot's current mode. If the mode function returns null, fall back to the default exploration behavior.

### UI Integration
- Per-robot dropdown in the robot panel (sidebar)
- WebSocket message: `{ type: "setMode", robotId: string, mode: string }`
- Server forwards to robot agent
- Robot stores `currentMode` and uses it in `doChoosing`
- Mode change takes effect at next CHOOSING phase (doesn't interrupt travel)

### Mode State
Each mode may need per-robot state:
- **Recheck Map:** set of recently rechecked node IDs + cooldown timer
- **Consolidate Pathing:** set of attempted edge IDs (avoid retrying failed edges)
- **Check Main Pathways:** patrol route (ordered list of edges to traverse)
- **Find Boxes:** detected box catalog + scan angles completed

This state lives in the robot agent, not the graph.
