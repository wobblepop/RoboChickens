# SLAM Explorer — System Architecture

**Last updated:** 2026-05-26

## Overview

A real-time, multi-robot warehouse exploration and mapping system. Robots autonomously explore unknown facilities using LiDAR SLAM, building a shared topological graph of the environment. Each robot operates independently — information is shared only through the Gun.js protocol.

The system has two layers: a **world simulation** (ground truth, physics, sensors) that only the server knows about, and **robot agents** that receive only noisy sensor data and must build their maps from scratch.

## Robot Exploration Algorithm (4-Phase Cycle)

Each robot follows this cycle continuously:

1. **ARRIVING** — Robot reaches target node (or creates starting node on tick 1)
   - Mark node as explored, capture SLAM data
   - Run scan matching refinement at node (`refinePoseAtNode`)
   - Analyze corridor scan log for retroactive junction detection
   - Create/update node local map

2. **SCANNING** — 5-tick pause, then feature detection + node/edge creation
   - 360-degree LiDAR feature detection (junctions, corners, dead ends, side openings)
   - Suppress node creation in straight corridors (only at turns/features)
   - Create untraveled edges to visible nodes via LiDAR line-of-sight
   - Revalidate existing edges: mark blocked if no longer visible, reopen if visible again
   - Periodic freshness decay and dead-entity pruning

3. **CHOOSING** — Pick next edge via exploration priority
   - Manual path queue (if user clicked a target — BFS through graph, hop by hop)
   - Behavior mode strategy (8 selectable modes)
   - Fallback: unexplored adjacent nodes > untraveled edges > neighbors with most unmapped edges > grid frontier > random

4. **TRAVELING** — Drive to destination, collect corridor scans
   - Reactive navigation with wall avoidance and corridor centering
   - Side-passage detection creates junction nodes at perpendicular openings
   - Corridor scan log captures LiDAR data for retroactive junction detection on arrival
   - Stuck detection and recovery (wall-stop, timeout, reversal)

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
|  Motor Controller | Behavior Modes                               |
+------------------------------------------------------------------+
|                    MERGE PROTOCOL                                 |
|  Candidate Detector | Graph Aligner (Umeyama similarity)         |
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
   - Motion model with Gaussian odometry noise
   - Particle weighting by scan-to-map agreement (no per-tick pose snapping)
   - Low-variance systematic resampling
   - Single shared occupancy grid updated by best particle
   - First tick: exact origin placement (no particle noise on starting node)

2. **Occupancy Grid** — log-odds Float32Array, 0.1m resolution, 80m grid
3. **Inverse Sensor Model** — Bresenham ray tracing per LiDAR beam, wall thickening on 4-connected neighbors
4. **Scan Matching** — runs ONLY at node arrivals (`refinePoseAtNode`), not during travel
   - Prevents tunnel-axis drift in straight corridors
   - Balanced objective: min(α * topology_mismatch + β * displacement²)
   - Both terms normalized to comparable scales (scan score / window²)
   - Tight window (±0.3m, ±5°) with strong odometry prior (weight=8.0)
   - Max correction capped at 0.5m from odometry prediction

## Confidence Model

Confidence values are assigned at creation and updated through observation:
- Nodes: initial confidence from feature detection quality, increased via `strengthenNode` (diminishing returns curve: `1 - exp(-0.25 * count)`)
- Edges: initial confidence based on traveled/untraveled status
- Freshness decay: exponential decay based on stability class (static features never decay, semi-static/dynamic features lose confidence over sim-minutes)

### Tiers
| Score | Tier | Behavior |
|-------|------|----------|
| 0.00-0.10 | Dead | Pruned (unless traveled/explored) |
| 0.10-0.35 | Candidate | Visible, not pathable |
| 0.35-0.65 | Provisional | Pathable at 2x cost |
| 0.65-0.85 | Committed | Full member |
| 0.85-1.00 | Anchored | Reference point |

## Multi-Robot Merge Protocol

Simplified 3-step alignment:
1. **Candidate Detection** — Generate node summaries, match by geometry + feature + topology similarity
2. **Evaluation** — Per-dimension floors (geometry>0.6, features>0.4, topology>0.5), composite>0.70
3. **Alignment** — Umeyama similarity transform (1 anchor=translation, 2=similarity, 3+=residual-qualified)

After alignment, the subscriber transforms incoming external node positions into the local SLAM frame. The world server can also establish alignment using known starting positions (pure translation offset).

## Environment Dynamics

When the environment changes (walls moved, paths blocked):
- **Freshness decay** — dynamic features lose confidence exponentially (sim-minutes timescale)
- **Edge revalidation** — on node revisit, edges no longer visible become "blocked" (not deleted)
- **Dead pruning** — only speculative entities are removed; traveled edges and explored nodes persist as blocked/archived
- **Map editing** — browser UI can place/remove walls; changes update the ground truth immediately

## File Structure

```
slam-explorer/
  src/
    shared/          Types, math, geometry, protocol, events, UUID,
                     Gun.js adapters (in-process + real network)
    world/           Server: ground truth map, physics engine,
                     LiDAR/odometry simulation, robot connection,
                     visualization WebSocket
    robot/
      slam/          Particle filter, occupancy grid, scan matching,
                     inverse sensor model
      network/       Topological graph, graph publisher/subscriber
      navigation/    Motor controller (reactive wall avoidance)
      treadmill/     Node/edge local maps (SLAM data per entity)
      merge/         Candidate detector, Umeyama alignment
      feature-detection/  LiDAR feature extraction
      behavior/      8 behavior mode strategy functions
      (root)         Robot agent (main exploration loop)
  public/            Browser visualization (single index.html)
  warehouse-maps/    ASCII map files
```

## Default Configuration

| Parameter | Value |
|-----------|-------|
| LiDAR rays | 360, full 360°, 8m range |
| Odometry noise | 1% linear, 1.5% angular |
| Robot radius | 0.25m |
| SLAM particles | 30 |
| SLAM grid | 0.1m resolution, 80m |
| Node separation | 2.5m minimum |
| Tick interval | 100ms |
| Max speed | 3.0 m/s linear, 3.0 rad/s angular |
