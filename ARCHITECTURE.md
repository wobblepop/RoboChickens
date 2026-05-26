# SLAM Explorer — System Architecture

**Last updated:** 2026-05-25

## Overview

A real-time, multi-robot warehouse exploration and mapping system. Robots autonomously explore unknown facilities using LiDAR SLAM, building a shared topological graph of the environment. Each robot operates independently — information is shared only through the Gun.js protocol.

The system has two layers: a **world simulation** (ground truth, physics, sensors) that only the server knows about, and **robot agents** that receive only noisy sensor data and must build their maps from scratch.

## Robot Exploration Algorithm (7-Step Cycle)

Each robot follows this cycle continuously:

1. **ARRIVE** at a target node (starting position is the first node)
   - Mark node as explored, capture SLAM data
   - Run drift correction against stored node map (if revisiting)
   - Propagate corrections to connected unexplored nodes

2. **SCAN** — identify new unvisited nodes in every direction
   - 360-degree LiDAR feature detection (junctions, corners, dead ends, side openings)
   - Suppress node creation in straight corridors (only at turns/features)
   - Avoid placing nodes at max LiDAR range

3. **CONNECT** — identify untraveled edges via LiDAR visibility check
   - For all nodes within scan radius, check line-of-sight (minimum range in a narrow cone)
   - Create untraveled edges (dashed lines) for verified visibility connections
   - Revalidate existing edges: mark blocked if no longer visible, reopen if visible again

4. **CHOOSE** an edge to travel, using priority:
   - Manual path queue (if user clicked a target — BFS through graph, hop by hop)
   - Adjacent nodes that haven't been explored
   - Edges that haven't been traveled
   - Adjacent nodes with the most unmapped edges
   - Random edge (last resort)

5. **TRAVEL** to 50% of the edge length
   - SLAM continues mapping the corridor during travel
   - Side-passage detection creates junction nodes at perpendicular openings
   - Loop closure checked every 20 ticks

6. **MID-SCAN** — repeat steps 2-3 at the midpoint
   - Discover nodes and edges visible from the corridor midpoint
   - Edges only radiate from the nearest existing node (never between two remote nodes)

7. **COMPLETE** — travel to the destination node, restart from step 1

## System Layers

```
+------------------------------------------------------------------+
|                    WORLD SIMULATION (server only)                 |
|  Ground truth map | Physics engine | LiDAR + Odometry simulation |
+------------------------------------------------------------------+
|                    ROBOT AGENT (per robot, no ground truth)       |
|  SLAM Engine (particle filter + occupancy grid + scan matching)  |
|  Feature Detection | Treadmill Edge/Node Maps                   |
|  Network Graph | Graph Publisher/Subscriber (Gun.js)             |
|  Drift Corrector | Loop Closure Detector                        |
+------------------------------------------------------------------+
|                    VALIDATION + SCORING                           |
|  Confidence Scorer (5-factor multiplicative)                     |
|  Consistency Engine (5 topological checks)                       |
|  Regression Controller (quarantine + rollback)                   |
+------------------------------------------------------------------+
|                    MERGE PROTOCOL                                 |
|  Candidate Detector | Graph Aligner | Graph Merger               |
+------------------------------------------------------------------+
|                    NAVIGATION                                     |
|  A* Path Planner | D* Lite Replanner | Motor Controller          |
|  Exploration Strategy | Manual Target (BFS hop-by-hop)           |
+------------------------------------------------------------------+
```

## Key Data Types

### NetworkNode
- Position (SLAM frame), type (junction/doorway/dead_end/corner/room_entry)
- **explored** (robot has arrived and scanned), **explored_by** (which robots)
- Confidence, tier (Dead/Candidate/Provisional/Committed/Anchored)
- Stability class (Static/SemiStatic/Dynamic), observation log, version

### NetworkEdge
- Source/target nodes, distance, bearing change, bidirectional
- **traveled** (robot has physically traversed), **traveled_by** (which robots)
- Edge type, traversability (clearance, slope, status: open/blocked/restricted)
- Confidence, tier, stability class

### Key Rules
- Traveled edges are never pruned — they become "blocked" if no longer visible
- Explored nodes are never pruned — they represent confirmed locations
- Only speculative (untraveled, never-confirmed) edges/nodes can be pruned
- Edges only connect from the robot's current position to visible nodes

## SLAM Pipeline

1. **Particle Filter** (30 particles, GMapping-style)
   - Motion model with Gaussian odometry noise (1% linear, 2% angular)
   - Particle weighting by scan-to-map agreement (no per-tick pose snapping)
   - Low-variance systematic resampling
   - Single shared occupancy grid updated by best particle
   - First tick: exact origin placement (no particle noise on starting node)

2. **Occupancy Grid** — log-odds Float32Array, 0.1m resolution, 60m grid
3. **Inverse Sensor Model** — Bresenham ray tracing per LiDAR beam
4. **Scan Matching** — runs ONLY at node arrivals (not during travel)
   - Prevents tunnel-axis drift in straight corridors
   - Balanced objective: min(α * topology_mismatch + β * displacement²)
   - Both terms normalized to comparable scales (scan score / window²)
   - Tight window (±0.3m, ±5°) with strong odometry prior (weight=8.0)
   - Max correction capped at 0.5m from odometry prediction

## Confidence Model

Multiplicative 5-factor model: `C = C_source^0.20 * C_obs^0.15 * C_consist^0.25 * C_corrob^0.25 * C_fresh^0.15`

| Factor | Description |
|--------|-------------|
| Source | Weighted mean of source-type credibility (sensor=0.85, peer=0.70, plan=0.60, manual=0.50) |
| Observation | Diminishing returns: 1 - e^(-0.25 * count) |
| Consistency | From topological checks (degree, distance, angular, crossing, region) |
| Corroboration | 1 source=0.4, 2 agreeing=0.65, 3+=0.85, disagreeing=min*0.6 |
| Freshness | Static=1.0 always, SemiStatic=e^(-0.005*h), Dynamic=e^(-0.05*h) |

### Tiers
| Score | Tier | Behavior |
|-------|------|----------|
| 0.00-0.10 | Dead | Pruned (unless traveled/explored) |
| 0.10-0.35 | Candidate | Visible, not pathable |
| 0.35-0.65 | Provisional | Pathable at 2x cost |
| 0.65-0.85 | Committed | Full member |
| 0.85-1.00 | Anchored | Reference point |

## Drift Correction

Three mechanisms:
1. **Map correlation at nodes** — on revisit, cross-correlate fresh SLAM grid with stored node map over a ±2m search window to find the (dx,dy) offset minimizing squared error
2. **Loop closure during travel** — every 20 ticks, scan-match against stored reference scans at nearby nodes; apply 30% partial correction
3. **Neighbor propagation** — after correcting a node, shift connected unexplored nodes proportionally (explored nodes are anchored)

## Multi-Robot Merge Protocol

6-phase sequence:
1. **Candidate Detection** — broadcast node summaries, match by geometry+features+topology
2. **Evaluation** — per-dimension floors (geometry>0.6, features>0.4, topology>0.5), composite>0.70
3. **Alignment** — Umeyama similarity transform (1 anchor=translation, 2=similarity, 3+=residual-qualified)
4. **Merge** — union observation logs, remap edges, transform coordinates
5. **Peer Connection** — establish Gun.js sync
6. **Post-Merge Validation** — consistency checks on merged subgraph

## Environment Dynamics (Trimming)

When the environment changes (walls moved, paths blocked):
- **Freshness decay** — dynamic features lose confidence exponentially
- **Edge revalidation** — on node revisit, edges no longer visible become "blocked" (not deleted)
- **Dead pruning** — only speculative entities are removed; traveled edges and explored nodes persist as blocked/archived
- **Map editing** — browser UI can place/remove walls; changes update the ground truth immediately

## File Structure

```
slam-explorer/
  src/
    shared/          Types, math, protocol, Gun.js adapters
    world/           Server: physics, sensors, ground truth, visualization
    robot/
      slam/          Particle filter, occupancy grid, scan matching
      network/       Graph storage, publisher, subscriber, loop closure
      navigation/    Path planning (A*, D* Lite), motor control, exploration
      treadmill/     Node/edge local maps (SLAM data per entity)
      merge/         Multi-robot merge protocol (detect, align, merge)
      feature-detection/  LiDAR feature extraction
      (root)         Robot agent, observation log, confidence, consistency,
                     regression, drift correction, spatial index
  public/            Browser visualization (single index.html)
  warehouse-maps/    ASCII map files
  tests/             Jest test suite
```

## Default Configuration

| Parameter | Value |
|-----------|-------|
| LiDAR rays | 360, full 360°, 8m range |
| Odometry noise | 1% linear, 2% angular |
| Robot radius | 0.25m |
| SLAM particles | 30 |
| SLAM grid | 0.1m resolution, 60m |
| Node separation | 2.5m minimum |
| Tick interval | 100ms |
| Max speed | 3.0 m/s linear, 10.0 rad/s angular |
