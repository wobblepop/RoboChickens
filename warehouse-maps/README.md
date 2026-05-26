# Warehouse Map Test Suite

Test maps for multi-robot warehouse mapping algorithms, progressing from trivial to maximum complexity.

## File Format

Each `.txt` file contains metadata lines (prefixed with `#`) followed by the grid. All rows in a grid have equal width. The grid is a rectangular array where each character represents one cell.

### Cell Legend

| Symbol | Name | Traversable | Description |
|--------|------|:-----------:|-------------|
| `#` | Wall | No | Permanent barrier (wall, shelf, rack) |
| `.` | Floor | Yes | Standard open floor |
| `R` | Start | Yes | Single-robot start position |
| `1`-`4` | Start N | Yes | Numbered robot start position |
| `O` | Obstacle | No | Pallet, equipment, or stored goods |
| `P` | Pillar | No | Structural column (point obstacle) |
| `D` | Doorway | Yes | Narrow passage / chokepoint marker |
| `~` | Rough | Yes | Rough terrain (higher movement cost) |

### Parsing Notes

- Lines starting with `#` are metadata/comments; skip when parsing the grid
- The first non-comment line is grid row 0
- Coordinate system: `(x, y)` where x is column (left=0) and y is row (top=0)
- All traversable cells are connected (verified by BFS from each start position)
- Grid border is always solid wall

---

## Map Summary

| Map | Size | Robots | Floor | Obstacles | Primary Test |
|-----|------|:------:|------:|----------:|--------------|
| 01 Zigzag | 25x9 | 1 | 88 | 0 | Linear path coverage |
| 02 T-Junction | 25x15 | 1 | 168 | 0 | Branch decisions |
| 03 Loop | 21x13 | 1 | 104 | 0 | Loop closure |
| 04 Dead Ends | 31x19 | 1 | 87 | 0 | Backtracking |
| 05 Two Rooms | 35x15 | 2 | 321 | 0 | Basic coordination |
| 06 Aisle Grid | 37x23 | 2 | 287 | 0 | Systematic exploration |
| 07 Obstacles | 37x23 | 2 | 677 | 58 | Obstacle avoidance |
| 08 Chokepoints | 45x25 | 3 | 644 | 63 | Bottleneck routing |
| 09 Symmetric | 39x21 | 2 | 527 | 0 | Perceptual aliasing |
| 10 Gauntlet | 55x35 | 4 | 1185 | 29 | Everything combined |

---

## Map Details

### Map 01: Zigzag Corridor (Difficulty 1/10)

**File:** `map-01-zigzag.txt` | **Size:** 25x9 | **Robots:** 1

A single snake-shaped path with no branches. The robot must traverse four horizontal corridors connected by vertical turns. This is the baseline: if the algorithm can't handle this, nothing else will work.

**What it tests:**
- Can the robot move forward and turn?
- Does it correctly mark visited cells?
- Does exploration terminate when all reachable space is covered?

**Expected behavior:** Robot follows the only available path from start to dead end. 100% coverage guaranteed by any functional exploration algorithm.

---

### Map 02: T-Junction with Rooms (Difficulty 2/10)

**File:** `map-02-t-junction.txt` | **Size:** 25x15 | **Robots:** 1

A vertical approach corridor opens to a T-intersection. The left branch leads to a 10x7 open room; the right branch leads to a 10x7 open room. The robot must choose a branch, explore it, backtrack, then explore the other.

**What it tests:**
- Frontier selection at a branch point
- Backtracking to unexplored frontiers
- Room (open area) exploration after corridor navigation

**Key observation:** The robot's branch choice doesn't affect final coverage, but affects efficiency. A good algorithm should complete both rooms without redundant backtracking.

---

### Map 03: Simple Loop (Difficulty 2/10)

**File:** `map-03-loop.txt` | **Size:** 21x13 | **Robots:** 1

A rectangular corridor loop (2 cells wide). The robot starts at bottom-left and can go clockwise or counterclockwise. Both paths reconnect.

**What it tests:**
- **Loop closure detection** — the critical SLAM challenge
- Does the algorithm recognize it has returned to a previously visited area?
- Does it avoid "double-counting" the loop as two separate corridors?

**Key observation:** Without loop closure, the robot may believe it's in a spiral rather than a loop, causing map distortion. The 2-cell corridor width provides some margin but the algorithm must still detect the closure.

---

### Map 04: Dead-End Branches (Difficulty 3/10)

**File:** `map-04-dead-ends.txt` | **Size:** 31x19 | **Robots:** 1

A main horizontal corridor with 5 dead-end branches of different shapes:
- **Branch A** (up-left): Short stub with a horizontal cap
- **Branch B** (up-center): Medium corridor with a side pocket room
- **Branch C** (down-center): Long vertical dead end with horizontal cap
- **Branch D** (up-right): L-shaped corridor
- **Branch E** (down-right): T-shaped branch

**What it tests:**
- Efficient dead-end detection (recognizing when to stop exploring a branch)
- Frontier prioritization (which branch to explore next?)
- Backtracking to the main corridor after each dead end
- Handling branches of different topological complexity (stubs, L-shapes, T-shapes)

**Key observation:** The optimal strategy explores each branch exactly once. A poor frontier selector might partially enter a branch, backtrack to another, then return — wasting movement.

---

### Map 05: Two Robots, Connected Rooms (Difficulty 3/10)

**File:** `map-05-two-robots.txt` | **Size:** 35x15 | **Robots:** 2

Two 14x13 rooms connected by a single-cell-wide corridor at row 7. Each room contains four 2x3 shelving blocks creating aisle-like navigation. Robot 1 starts in the left room, Robot 2 in the right room.

**What it tests:**
- **First multi-robot test** — can two robots operate simultaneously?
- Territory division (each robot should primarily map its own room)
- Corridor contention (what happens when both robots try to cross the corridor?)
- Map merging (can independently built maps be combined when robots meet?)

**Key observation:** The narrow corridor is a natural territory boundary. Ideally, each robot fully maps its own room, then one crosses to verify/merge. If both try to cross simultaneously, the algorithm must handle contention.

---

### Map 06: Warehouse Aisle Grid (Difficulty 4/10)

**File:** `map-06-aisle-grid.txt` | **Size:** 37x23 | **Robots:** 2

A regular warehouse layout: 7 vertical aisles intersecting 5 horizontal cross-aisles, forming a grid of 24 shelf blocks. Robot 1 starts at bottom-left, Robot 2 at top-right (diagonal opposites).

**What it tests:**
- Systematic exploration of highly regular structure
- Multi-robot work division in a grid topology
- Redundancy avoidance (robots should not re-explore each other's mapped areas)
- The grid creates many equivalent paths — does the algorithm choose efficiently?

**Key observation:** The grid has high connectivity (many loops), so loop closure is tested repeatedly. The regular structure also tests whether the algorithm exploits symmetry or treats each aisle independently. Optimal coverage divides the grid roughly in half.

---

### Map 07: Open Floor with Obstacles (Difficulty 5/10)

**File:** `map-07-obstacles.txt` | **Size:** 37x23 | **Robots:** 2

A large open warehouse floor (34x20 interior) with:
- **18 pillars (P):** Arranged in a 6x3 grid, spacing every 6 columns and 5 rows
- **18 pallet clusters (O):** 2-cell-wide obstacles in 3 rows of 3 clusters each
- **Wall-mounted equipment (O):** Along top, bottom, left, and right walls

**What it tests:**
- Open-space coverage (very different from corridor-based exploration)
- Obstacle detection and path planning around objects
- Distinguishing between structural columns, temporary obstacles, and walls
- Efficient coverage patterns in open areas (lawn-mower, frontier-based, etc.)

**Key observation:** Open spaces are harder to map efficiently than corridors because there are many possible frontiers at every step. The obstacle layout creates implicit "lanes" but the algorithm shouldn't rely on them. Wall-mounted equipment creates impassable zones along edges.

---

### Map 08: Chokepoint Zones (Difficulty 6/10)

**File:** `map-08-chokepoints.txt` | **Size:** 45x25 | **Robots:** 3

Five distinct zones connected only through narrow doorway corridors:
- **Zone A** (top-left): 13x10, furniture obstacles
- **Zone B** (top-center): 13x10, furniture obstacles
- **Zone C** (bottom-left): 13x10, furniture obstacles
- **Zone D** (bottom-center): 13x10, furniture obstacles
- **Zone E** (right wing): 7x23, tall narrow zone, furniture obstacles

Connections: A↔B, A↔C, B↔D, C↔D, B↔E, D↔E. Each connection passes through a narrow corridor with a doorway marker (D).

**What it tests:**
- **Multi-robot congestion at bottlenecks** — 3 robots, 6 doorways
- Zone-level exploration planning (which robot should explore which zone?)
- Doorway queuing (when two robots need the same passage)
- Furniture avoidance within zones while navigating toward exits

**Key observation:** With 3 robots and 5 zones, at least two robots must share access to some zones. The topology creates potential deadlocks if robots block each other in narrow corridors. The algorithm should prioritize doorway access to minimize waiting.

---

### Map 09: Symmetric Warehouse (Difficulty 7/10)

**File:** `map-09-symmetric.txt` | **Size:** 39x21 | **Robots:** 2

A warehouse with perfect mirror symmetry around a central vertical spine (column 19). Each half contains 6 rooms arranged in a 3x2 grid, connected by vertical and horizontal corridors. The halves connect through 3 bridge points on the central spine.

**What it tests:**
- **Perceptual aliasing** — the defining SLAM challenge
- The left-top room looks identical to the right-top room from the inside
- Can the algorithm correctly determine which half it's in?
- Map merging when robots in symmetric positions produce similar-looking maps

**Key observation:** This is a stress test for localization. If the SLAM algorithm relies only on local geometry (wall distances, room shape), it will confuse left and right rooms. Successful localization requires either: (a) tracking path history from a known start, (b) detecting the asymmetric robot start positions, or (c) global feature matching via the central spine. Robots starting at symmetric positions (1 at left, 2 at right) make this maximally challenging.

---

### Map 10: The Gauntlet (Difficulty 10/10)

**File:** `map-10-gauntlet.txt` | **Size:** 55x35 | **Robots:** 4

A full warehouse complex with 6 distinct zones:

| Zone | Location | Type | Key Features |
|------|----------|------|-------------|
| **A: Loading Dock** | Bottom strip (rows 29-33) | Open area | 5 pillars, wide open space, 2 robots start here |
| **B: Aisle Zone** | Top-left (rows 1-17) | Regular grid | 5 vertical aisles, 5 cross-aisles, classic warehouse |
| **C: Open Staging** | Top-right (rows 1-13) | Open + obstacles | 18 pallet obstacles, 6 pillars, open floor |
| **D: Offices** | Right-mid (rows 15-27) | Rooms + doors | 3 rooms connected by doorways (D), restricted access |
| **E: Narrow Maze** | Bottom-left (rows 21-27) | Maze corridors | Winding paths through partial internal walls |
| **F: Rough Terrain** | Center (rows 21-27) | Slow zone (~) | Higher traversal cost, connects to multiple zones |

**Zone connectivity:**
```
B (Aisles) ←→ C (Staging)     via corridor at row 5
B (Aisles) ←→ E (Maze)        via corridor at col 6
B (Aisles) ←→ F (Rough)       via corridor at col 18
E (Maze)   ←→ A (Loading)     via corridor at col 6
F (Rough)  ←→ A (Loading)     via corridor at col 20
F (Rough)  ←→ D (Offices)     via corridor at row 25
D (Offices)←→ A (Loading)     via corridor at col 40
C (Staging)←→ D (Offices)     via doorways at cols 33, 47
```

**What it tests:**
- **All previous challenges simultaneously**
- 4-robot coordination across 6 heterogeneous zones
- Transitioning between different navigation modes (corridors → open space → maze → rough terrain)
- Inter-zone path planning through connection corridors
- Handling terrain cost variation (~ cells vs . cells)
- Complex doorway routing with multiple restricted-access rooms
- Exploration prioritization across fundamentally different zone types

**Key observation:** No single exploration strategy works for all zones. Aisle Zone B rewards systematic row-by-row sweeping. Staging Area C needs obstacle-aware open-space coverage. The Maze E requires wall-following. Rough Terrain F introduces movement cost trade-offs. The algorithm must adapt its strategy per zone or use a general approach robust enough for all. With 4 robots starting at the corners, the initial territory assignment significantly impacts total exploration time.

---

## Suggested Testing Progression

### Phase 1: Single-Robot Fundamentals (Maps 01-04)

Run these sequentially. Each builds on the previous:

1. **Map 01** — Verify basic movement and coverage tracking work at all
2. **Map 02** — Verify branching and backtracking
3. **Map 03** — Verify loop closure (first real SLAM challenge)
4. **Map 04** — Verify efficient dead-end handling

**Pass criteria:** 100% coverage of all traversable cells. Track total steps vs. optimal (minimum) steps for efficiency measurement.

### Phase 2: Multi-Robot Basics (Maps 05-06)

Introduce the second robot:

5. **Map 05** — Two robots in separate spaces (low contention)
6. **Map 06** — Two robots in shared grid (high coordination needed)

**Pass criteria:** 100% coverage. Measure: does adding the second robot actually reduce total time? If not, coordination is failing.

### Phase 3: Environmental Complexity (Maps 07-09)

Keep 2-3 robots, increase map complexity:

7. **Map 07** — Open spaces with obstacles (non-corridor mapping)
8. **Map 08** — Chokepoints with 3 robots (contention stress test)
9. **Map 09** — Symmetric layout (localization stress test)

**Pass criteria:** 100% coverage. Measure: coverage rate over time (should be roughly linear, not plateauing), deadlock frequency at chokepoints, localization error in the symmetric map.

### Phase 4: Integration (Map 10)

The final test:

10. **Map 10** — Everything at once, 4 robots

**Pass criteria:** 100% coverage. Measure all metrics from previous phases simultaneously. The algorithm either handles the full complexity or it doesn't.

---

## Metrics to Track

For each test run, record:

| Metric | What it measures | How to compute |
|--------|-----------------|----------------|
| **Coverage %** | Exploration completeness | Visited cells / total traversable cells |
| **Total steps** | Aggregate movement cost | Sum of all robot movements |
| **Exploration ratio** | Movement efficiency | Total steps / total traversable cells (lower = better, 1.0 = perfect) |
| **Coverage rate** | Speed of exploration | Coverage % vs. time step (plot as curve) |
| **Redundant visits** | Wasted movement | Cells visited by >1 robot / total cells visited |
| **Deadlocks** | Coordination failure | Count of situations where robots block each other |
| **Loop closure count** | SLAM accuracy | Correct loop closures / total opportunities |
| **Localization error** | Position accuracy | Estimated position - true position (Map 09 especially) |

---

## Regenerating Maps

The maps were generated by `generate_maps.py` in this directory. The script includes connectivity validation (BFS from every start position) and border integrity checks. Run it to regenerate:

```
py generate_maps.py
```
