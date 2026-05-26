# Stitched SLAM Map Viewer — Design Document

## Overview

Each robot maintains a treadmill occupancy grid centered on its current position. Node maps capture SLAM grid snapshots at each explored node. Edge maps capture corridor strips along traveled edges. The stitched map viewer combines these fragments into a coherent, high-fidelity map for a given context.

## Two Viewing Modes

### Mode A: Radius View
**Trigger:** Click a robot or node in the UI.
**What it shows:** A combined SLAM map within a configurable radius (default 10m) around the selected point.
**Source data:**
- The node map at the selected node (primary)
- Node maps of all neighbors within the radius
- Edge corridor maps connecting those neighbors
**Stitching:** Overlay all maps in SLAM coordinates. Where maps overlap, use the highest-confidence cell value (most positive for walls, most negative for free space). Unknown cells stay unknown.

### Mode B: Route View
**Trigger:** Select two nodes in the UI (click start, shift-click destination).
**What it shows:** A combined SLAM map along the shortest path from A to B, showing the full corridor the robot would traverse.
**Source data:**
- Compute the shortest path A → N1 → N2 → ... → B via findPath
- Collect node maps at A, N1, N2, ..., B
- Collect edge corridor maps for each hop
**Stitching:** Same as radius view but linear. Render the map as a scrollable strip or fit-to-window overview.

## Stitching Algorithm

```
function stitchMaps(centerX, centerY, radius, nodeMaps, edgeMaps):
  // 1. Determine bounding box in SLAM coords
  minX = centerX - radius
  maxX = centerX + radius
  minY = centerY - radius
  maxY = centerY + radius

  // 2. Create output grid at same resolution as SLAM grid (0.1m)
  outputWidth = ceil((maxX - minX) / 0.1)
  outputHeight = ceil((maxY - minY) / 0.1)
  output = new Float32Array(outputWidth * outputHeight)  // init to 0 (unknown)

  // 3. For each source map, copy cells into output
  for each map in [nodeMaps + edgeMaps]:
    for each cell (gx, gy) in map.grid:
      worldX = map.origin_x + (gx + 0.5) * map.resolution
      worldY = map.origin_y + (gy + 0.5) * map.resolution

      // Check if within output bounds
      outGx = floor((worldX - minX) / 0.1)
      outGy = floor((worldY - minY) / 0.1)
      if outGx < 0 or outGx >= outputWidth: continue
      if outGy < 0 or outGy >= outputHeight: continue

      // Merge: keep the value with the highest absolute magnitude
      // (strongest evidence, whether occupied or free)
      outIdx = outGy * outputWidth + outGx
      if abs(map.cells[gy * map.width + gx]) > abs(output[outIdx]):
        output[outIdx] = map.cells[gy * map.width + gx]

  return { output, minX, minY, outputWidth, outputHeight, resolution: 0.1 }
```

## Data Flow

### Server-Side
1. Client sends: `{ type: "stitchMap", mode: "radius" | "route", nodeId?: string, fromId?: string, toId?: string, radius?: number }`
2. Server collects relevant node maps and edge maps from the robot's treadmill managers
3. Server runs stitching algorithm
4. Server sends back: `{ type: "stitchedMap", data: { cells: number[], width: number, height: number, originX: number, originY: number, resolution: number, nodes: VizNode[], edges: VizEdge[] } }`

### Client-Side
1. Render the stitched grid as a canvas overlay or in a popup panel
2. Use the same color scheme as the occupancy grid: green = free, red = occupied, gray = unknown
3. Overlay the topological graph (nodes + edges) on top
4. Highlight the route path (for route view)

## Implementation Plan

### Phase 1: Server-side stitching
1. Add `getNodeMap(nodeId)` and `getEdgeMap(edgeId)` accessors to robot-agent
2. Implement `stitchRadiusMap(robotId, nodeId, radius)` in world-server
3. Implement `stitchRouteMap(robotId, fromId, toId)` in world-server
4. Add WebSocket handler for stitchMap requests

### Phase 2: Client-side rendering
1. Add a popup panel (reuse the node-info popup style)
2. Render the stitched grid on a canvas element
3. Color cells by log-odds: negative = green, positive = red, zero = dark gray
4. Overlay node circles and edge lines
5. Add zoom/pan controls

### Phase 3: Route highlighting
1. When in route mode, highlight the path edges in a different color
2. Show node names along the route
3. Display total route distance and estimated travel time

## Data Requirements

**Node maps** (from NodeMapManager): stored per explored node, contain a square SLAM grid snapshot (typically 3m x 3m around the node position). Include origin, resolution, width, height.

**Edge corridor maps** (from EdgeMapManager): stored per traveled edge, contain a rectangular strip of SLAM grid along the travel path. Width is corridor width, length is edge distance.

Both are in SLAM coordinates — the stitching algorithm works entirely in SLAM frame. For display, the output is transformed to world coordinates via slamToWorld.

## Estimated Complexity

| Component | Lines | Effort |
|-----------|-------|--------|
| Server stitching logic | ~150 | Medium |
| WebSocket protocol | ~50 | Low |
| Client canvas rendering | ~200 | Medium |
| Route highlighting | ~80 | Low |
| **Total** | **~480** | **Medium** |
