# Pathing Improvements — Design Document

## Network-Level Improvements

### 1. Multi-Robot Exploration Coordination
**Problem:** Robots independently choose the same unexplored areas, wasting effort.
**Approach:** When choosing targets, weight against nodes/edges that other robots are currently traveling toward. Use the Gun protocol to share "I'm heading to node X" claims. Each robot deprioritizes targets within N meters of another robot's announced destination.
**Complexity:** Medium — requires publishing intent, not just state.

### 2. Avoid Exploration Loops
**Problem:** Robot oscillates between two nodes, revisiting the same area repeatedly.
**Approach:** Track the last N visited nodes (recency window). Penalize edges leading to recently visited nodes. The penalty decays with time — a node visited 50 ticks ago is fine, one visited 5 ticks ago is heavily penalized.
**Complexity:** Low — add a recency penalty to edge scoring.

### 3. Frontier Balancing
**Problem:** Robots cluster at one frontier and ignore others.
**Approach:** Assign each robot to the nearest unassigned frontier. Use a simple greedy assignment: sort all (robot, frontier) pairs by distance, assign greedily, skip already-assigned frontiers.
**Complexity:** Medium — requires global view across robots.

### 4. Edge Confidence-Weighted Pathfinding
**Problem:** BFS shortest path ignores edge quality. A 3-hop path through Anchored edges is better than a 2-hop path through Candidate edges.
**Approach:** Replace BFS with Dijkstra where edge cost = 1 / confidence. High-confidence edges are "cheaper" to traverse. Blocked edges have infinite cost.
**Complexity:** Low — swap BFS for Dijkstra in findPath.

### 5. Subgraph Reconnection
**Problem:** When an edge is blocked, the graph can split into disconnected components. Robots in one component can't reach nodes in the other.
**Approach:** After blocking an edge, check for disconnection. If detected, prioritize exploring toward the disconnected component (find the closest unexplored frontier in the direction of the disconnected nodes).
**Complexity:** Medium — requires disconnection detection + directional frontier search.

## Local-Level Improvements

### 6. Corner Waypoints
**Problem:** Robot clips wall corners when navigating between nodes. The straight-line path from A to B passes through a wall corner.
**Approach:** When creating edges around corners, add intermediate waypoint nodes at the corner apex. The robot follows A → corner → B instead of A → B directly. Detect corners as pairs of adjacent edges with >45 degree bearing change.
**Complexity:** Medium — requires corner detection and edge splitting.

### 7. Corridor Centering
**Problem:** Motor controller's side-wall avoidance creates oscillation in narrow corridors.
**Approach:** Replace reactive wall-push with predictive centering. Average the left and right wall distances over the last 5 ticks. Steer toward the average center, not the instantaneous center. Dampen the correction rate.
**Complexity:** Low — modify motor controller averaging window.

### 8. Look-Ahead Speed Control
**Problem:** Robot drives fast toward a target then has to brake hard at the last moment.
**Approach:** Compute the distance to the target AND the minimum clearance over the next N meters of the path. Scale speed by min(target_distance, corridor_clearance). Start braking when distance < 2 * stopping_distance.
**Complexity:** Low — modify speed computation in driveToward.

### 9. Smooth Path Following
**Problem:** Robot turns sharply at each node, creating jerky motion.
**Approach:** When the robot has a multi-hop path (manual or exploration), compute a smooth spline through the waypoints. Follow the spline instead of hopping node-to-node. Use cubic Bezier curves with control points at 1/3 and 2/3 of each edge.
**Complexity:** High — requires spline math and continuous path tracking.

### 10. Dynamic Obstacle Avoidance
**Problem:** Static path planning doesn't handle newly appeared obstacles (other robots, moved boxes).
**Approach:** During TRAVELING, continuously check forward clearance against expected clearance for the current position on the path. If clearance drops below threshold unexpectedly, trigger a local replan (find the nearest clear waypoint and path around the obstacle).
**Complexity:** Medium — requires expected-vs-actual clearance comparison.

## Priority Ranking

| # | Improvement | Impact | Effort | Priority |
|---|-------------|--------|--------|----------|
| 2 | Avoid exploration loops | High | Low | 1 |
| 7 | Corridor centering | High | Low | 2 |
| 8 | Look-ahead speed control | Medium | Low | 3 |
| 4 | Confidence-weighted pathfinding | Medium | Low | 4 |
| 6 | Corner waypoints | High | Medium | 5 |
| 1 | Multi-robot coordination | High | Medium | 6 |
| 10 | Dynamic obstacle avoidance | Medium | Medium | 7 |
| 5 | Subgraph reconnection | Low | Medium | 8 |
| 3 | Frontier balancing | Medium | Medium | 9 |
| 9 | Smooth path following | Low | High | 10 |
