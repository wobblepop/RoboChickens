"""
Warehouse Map Generator for Multi-Robot Mapping Algorithm Testing
Generates 10 progressively complex warehouse maps as text grid files.

Legend:
  #  Wall (impassable)
  .  Open floor (traversable)
  R  Robot start (single-robot maps)
  1-4  Numbered robot starts (multi-robot maps)
  O  Obstacle - pallet, equipment (impassable)
  P  Pillar (impassable point obstacle)
  D  Doorway (traversable, marks a chokepoint)
  ~  Rough terrain (traversable, higher movement cost)
"""
import os
from collections import deque

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class Grid:
    def __init__(self, width, height, fill='#'):
        self.w = width
        self.h = height
        self.data = [[fill] * width for _ in range(height)]

    def set(self, x, y, c):
        if 0 <= x < self.w and 0 <= y < self.h:
            self.data[y][x] = c

    def get(self, x, y):
        if 0 <= x < self.w and 0 <= y < self.h:
            return self.data[y][x]
        return None

    def rect(self, x1, y1, x2, y2, c='.'):
        for y in range(max(0, y1), min(self.h, y2 + 1)):
            for x in range(max(0, x1), min(self.w, x2 + 1)):
                self.data[y][x] = c

    def hline(self, x1, x2, y, c='.'):
        for x in range(max(0, x1), min(self.w, x2 + 1)):
            if 0 <= y < self.h:
                self.data[y][x] = c

    def vline(self, x, y1, y2, c='.'):
        for y in range(max(0, y1), min(self.h, y2 + 1)):
            if 0 <= x < self.w:
                self.data[y][x] = c

    def lines(self):
        return [''.join(row) for row in self.data]

    def verify(self):
        widths = set(len(row) for row in self.data)
        assert len(widths) == 1, f"Inconsistent row widths: {widths}"
        assert all(self.data[0][x] == '#' for x in range(self.w)), "Top border broken"
        assert all(self.data[-1][x] == '#' for x in range(self.w)), "Bottom border broken"
        assert all(self.data[y][0] == '#' for y in range(self.h)), "Left border broken"
        assert all(self.data[y][-1] == '#' for y in range(self.h)), "Right border broken"

    def find_starts(self):
        starts = {}
        for y in range(self.h):
            for x in range(self.w):
                c = self.data[y][x]
                if c == 'R' or c in '1234':
                    starts[c] = (x, y)
        return starts

    def reachable_from(self, sx, sy):
        passable = set('.R1234D~')
        visited = set()
        queue = deque([(sx, sy)])
        visited.add((sx, sy))
        while queue:
            x, y = queue.popleft()
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.w and 0 <= ny < self.h and (nx, ny) not in visited:
                    if self.data[ny][nx] in passable:
                        visited.add((nx, ny))
                        queue.append((nx, ny))
        return visited

    def check_connectivity(self):
        starts = self.find_starts()
        if not starts:
            return False, "No start positions found"
        passable = set('.R1234D~')
        all_passable = set()
        for y in range(self.h):
            for x in range(self.w):
                if self.data[y][x] in passable:
                    all_passable.add((x, y))
        first_start = list(starts.values())[0]
        reachable = self.reachable_from(*first_start)
        unreachable = all_passable - reachable
        if unreachable:
            return False, f"{len(unreachable)} unreachable cells (e.g. {list(unreachable)[:5]})"
        return True, f"All {len(all_passable)} passable cells reachable"

    def count_cells(self):
        counts = {}
        for y in range(self.h):
            for x in range(self.w):
                c = self.data[y][x]
                counts[c] = counts.get(c, 0) + 1
        return counts


def save(filename, grid, meta):
    grid.verify()
    ok, msg = grid.check_connectivity()
    status = "OK" if ok else "FAIL"
    path = os.path.join(BASE_DIR, filename)
    with open(path, 'w', newline='\n') as f:
        for k, v in meta.items():
            f.write(f"# {k}: {v}\n")
        f.write("#\n")
        for line in grid.lines():
            f.write(line + '\n')
    starts = grid.find_starts()
    start_str = ', '.join(f"{k}=({v[0]},{v[1]})" for k, v in sorted(starts.items()))
    counts = grid.count_cells()
    floor = counts.get('.', 0) + counts.get('~', 0) + counts.get('D', 0)
    for s in 'R1234':
        floor += counts.get(s, 0)
    obstacles = counts.get('O', 0) + counts.get('P', 0)
    print(f"  [{status}] {filename:35s} {grid.w:2d}x{grid.h:<2d}  "
          f"floor={floor:4d}  obstacles={obstacles:2d}  starts={start_str}  ({msg})")
    if not ok:
        print(f"         WARNING: {msg}")


# ═══════════════════════════════════════════════════════════════
# MAP 01: Zigzag Corridor
# ═══════════════════════════════════════════════════════════════
def map_01():
    g = Grid(25, 9)
    g.hline(1, 22, 1)
    g.vline(22, 1, 3)
    g.hline(2, 22, 3)
    g.vline(2, 3, 5)
    g.hline(2, 22, 5)
    g.vline(22, 5, 7)
    g.hline(2, 22, 7)
    g.set(1, 1, 'R')
    save("map-01-zigzag.txt", g, {
        "Name": "Zigzag Corridor",
        "Size": "25x9",
        "Robots": "1 (R)",
        "Difficulty": "1 - Trivial",
        "Tests": "Linear exploration, turning, complete single-path coverage",
        "Description": "Snake-shaped single path. No branches or decisions. Baseline test."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 02: T-Junction with Rooms
# ═══════════════════════════════════════════════════════════════
def map_02():
    g = Grid(25, 15)
    g.vline(12, 8, 13)
    g.hline(1, 23, 8)
    g.rect(1, 1, 10, 7)
    g.rect(14, 1, 23, 7)
    g.set(12, 13, 'R')
    save("map-02-t-junction.txt", g, {
        "Name": "T-Junction with Rooms",
        "Size": "25x15",
        "Robots": "1 (R)",
        "Difficulty": "2 - Easy",
        "Tests": "Branch decision, room exploration, backtracking to unexplored branch",
        "Description": "Vertical corridor opens to T-intersection. Each branch leads to a room."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 03: Simple Loop
# ═══════════════════════════════════════════════════════════════
def map_03():
    g = Grid(21, 13)
    g.rect(1, 1, 19, 2)
    g.rect(1, 10, 19, 11)
    g.rect(1, 1, 2, 11)
    g.rect(18, 1, 19, 11)
    g.set(1, 11, 'R')
    save("map-03-loop.txt", g, {
        "Name": "Simple Loop",
        "Size": "21x13",
        "Robots": "1 (R)",
        "Difficulty": "2 - Easy",
        "Tests": "Loop closure detection, bidirectional path choice",
        "Description": "Rectangular corridor loop. Both directions lead back to start."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 04: Dead-End Branches
# ═══════════════════════════════════════════════════════════════
def map_04():
    g = Grid(31, 19)
    g.hline(1, 29, 9)

    # Branch A: short stub (up-left)
    g.vline(4, 5, 9)
    g.hline(4, 7, 5)

    # Branch B: medium with side pocket (up-center)
    g.vline(11, 2, 9)
    g.hline(11, 15, 2)
    g.rect(14, 2, 15, 5)

    # Branch C: long dead end (down-center)
    g.vline(17, 9, 17)
    g.hline(15, 19, 17)

    # Branch D: L-shaped (up-right)
    g.vline(23, 3, 9)
    g.hline(23, 28, 3)
    g.vline(28, 1, 3)

    # Branch E: T-shaped (down-right)
    g.vline(28, 9, 14)
    g.hline(25, 29, 14)

    g.set(1, 9, 'R')
    save("map-04-dead-ends.txt", g, {
        "Name": "Dead-End Branches",
        "Size": "31x19",
        "Robots": "1 (R)",
        "Difficulty": "3 - Moderate",
        "Tests": "Backtracking efficiency, exploration completeness, dead-end detection",
        "Description": "Main corridor with 5 dead-end branches of varying depth and shape."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 05: Two Robots, Connected Rooms
# ═══════════════════════════════════════════════════════════════
def map_05():
    g = Grid(35, 15)
    g.rect(1, 1, 14, 13)
    g.rect(20, 1, 33, 13)
    g.hline(14, 20, 7)

    # Shelving in left room (4 blocks)
    g.rect(4, 3, 5, 5, '#')
    g.rect(4, 9, 5, 11, '#')
    g.rect(9, 3, 10, 5, '#')
    g.rect(9, 9, 10, 11, '#')

    # Shelving in right room (4 blocks)
    g.rect(23, 3, 24, 5, '#')
    g.rect(23, 9, 24, 11, '#')
    g.rect(28, 3, 29, 5, '#')
    g.rect(28, 9, 29, 11, '#')

    g.set(2, 7, '1')
    g.set(32, 7, '2')
    save("map-05-two-robots.txt", g, {
        "Name": "Two Robots, Connected Rooms",
        "Size": "35x15",
        "Robots": "2 (1, 2)",
        "Difficulty": "3 - Moderate",
        "Tests": "Basic multi-robot coordination, territory division, corridor contention",
        "Description": "Two rooms with shelving, connected by narrow corridor. One robot per room."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 06: Warehouse Aisle Grid
# ═══════════════════════════════════════════════════════════════
def map_06():
    g = Grid(37, 23)
    for cy in [1, 7, 13, 19, 21]:
        g.hline(1, 35, cy)
    for cx in [1, 7, 13, 19, 25, 31, 35]:
        g.vline(cx, 1, 21)
    g.set(1, 21, '1')
    g.set(35, 1, '2')
    save("map-06-aisle-grid.txt", g, {
        "Name": "Warehouse Aisle Grid",
        "Size": "37x23",
        "Robots": "2 (1, 2)",
        "Difficulty": "4 - Moderate",
        "Tests": "Systematic grid exploration, multi-robot work division in regular structure",
        "Description": "Classic warehouse with parallel aisles and cross-aisles forming a grid."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 07: Open Floor with Obstacles
# ═══════════════════════════════════════════════════════════════
def map_07():
    g = Grid(37, 23)
    g.rect(1, 1, 35, 21)

    # Pillar grid
    for px in range(5, 35, 6):
        for py in range(4, 20, 5):
            g.set(px, py, 'P')

    # Pallet clusters (2-cell wide)
    pallets = [
        (8, 6), (9, 6), (20, 6), (21, 6), (32, 6), (33, 6),
        (8, 11), (9, 11), (20, 11), (21, 11), (32, 11), (33, 11),
        (8, 16), (9, 16), (20, 16), (21, 16), (32, 16), (33, 16),
    ]
    for px, py in pallets:
        g.set(px, py, 'O')

    # Wall-mounted equipment
    g.hline(14, 18, 1, 'O')
    g.hline(14, 18, 21, 'O')
    g.vline(1, 8, 12, 'O')
    g.vline(35, 8, 12, 'O')

    g.set(2, 2, '1')
    g.set(34, 20, '2')
    save("map-07-obstacles.txt", g, {
        "Name": "Open Floor with Obstacles",
        "Size": "37x23",
        "Robots": "2 (1, 2)",
        "Difficulty": "5 - Challenging",
        "Tests": "Obstacle detection and avoidance, open-space coverage, path planning",
        "Description": "Open warehouse with pillars (P), pallets (O), and wall-mounted equipment."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 08: Chokepoint Zones
# ═══════════════════════════════════════════════════════════════
def map_08():
    g = Grid(45, 25)
    # Five zones
    g.rect(1, 1, 13, 10)      # Zone A (top-left)
    g.rect(19, 1, 31, 10)     # Zone B (top-center)
    g.rect(1, 14, 13, 23)     # Zone C (bot-left)
    g.rect(19, 14, 31, 23)    # Zone D (bot-center)
    g.rect(37, 1, 43, 23)     # Zone E (right wing)

    # Narrow doorway corridors
    g.hline(13, 19, 5);  g.set(16, 5, 'D')    # A-B
    g.vline(7, 10, 14);  g.set(7, 12, 'D')    # A-C
    g.vline(25, 10, 14); g.set(25, 12, 'D')   # B-D
    g.hline(13, 19, 18); g.set(16, 18, 'D')   # C-D
    g.hline(31, 37, 5);  g.set(34, 5, 'D')    # B-E
    g.hline(31, 37, 18); g.set(34, 18, 'D')   # D-E

    # Furniture in zones
    g.rect(3, 3, 5, 5, 'O')
    g.rect(8, 6, 10, 8, 'O')
    g.rect(21, 3, 23, 5, 'O')
    g.rect(3, 17, 5, 19, 'O')
    g.rect(21, 17, 23, 19, 'O')
    g.rect(39, 4, 41, 6, 'O')
    g.rect(39, 17, 41, 19, 'O')

    g.set(2, 2, '1')
    g.set(42, 22, '2')
    g.set(2, 22, '3')
    save("map-08-chokepoints.txt", g, {
        "Name": "Chokepoint Zones",
        "Size": "45x25",
        "Robots": "3 (1, 2, 3)",
        "Difficulty": "6 - Hard",
        "Tests": "Bottleneck congestion, multi-robot doorway routing, zone prioritization",
        "Description": "Five zones connected only by narrow doorways (D). Furniture obstacles (O) inside zones."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 09: Symmetric Warehouse
# ═══════════════════════════════════════════════════════════════
def map_09():
    g = Grid(39, 21)

    # Center spine
    g.vline(19, 1, 19)

    # Left half: 6 rooms in 3x2 grid
    g.rect(1, 1, 8, 5)
    g.rect(1, 9, 8, 13)
    g.rect(1, 15, 8, 19)
    g.rect(11, 1, 18, 5)
    g.rect(11, 9, 18, 13)
    g.rect(11, 15, 18, 19)
    g.vline(5, 5, 9);   g.vline(5, 13, 15)
    g.vline(14, 5, 9);  g.vline(14, 13, 15)
    g.hline(8, 11, 3);  g.hline(8, 11, 11);  g.hline(8, 11, 17)
    g.hline(18, 19, 3); g.hline(18, 19, 11);  g.hline(18, 19, 17)

    # Right half: mirror of left
    g.rect(30, 1, 37, 5)
    g.rect(30, 9, 37, 13)
    g.rect(30, 15, 37, 19)
    g.rect(20, 1, 27, 5)
    g.rect(20, 9, 27, 13)
    g.rect(20, 15, 27, 19)
    g.vline(33, 5, 9);   g.vline(33, 13, 15)
    g.vline(24, 5, 9);   g.vline(24, 13, 15)
    g.hline(27, 30, 3);  g.hline(27, 30, 11);  g.hline(27, 30, 17)
    g.hline(19, 20, 3);  g.hline(19, 20, 11);  g.hline(19, 20, 17)

    g.set(4, 3, '1')
    g.set(34, 3, '2')
    save("map-09-symmetric.txt", g, {
        "Name": "Symmetric Warehouse",
        "Size": "39x21",
        "Robots": "2 (1, 2)",
        "Difficulty": "7 - Hard",
        "Tests": "Perceptual aliasing, SLAM localization in symmetric environment, map merging",
        "Description": "Mirror-symmetric layout. Both halves are structurally identical, challenging localization."
    })


# ═══════════════════════════════════════════════════════════════
# MAP 10: The Gauntlet (Full Complex)
# ═══════════════════════════════════════════════════════════════
def map_10():
    g = Grid(55, 35)

    # ── SECTION A: Loading Dock (bottom, rows 29-33) ──
    g.rect(1, 29, 53, 33)
    for px in [8, 18, 28, 38, 48]:
        g.set(px, 31, 'P')

    # ── SECTION B: Aisle Zone (top-left, rows 1-17) ──
    for cy in [1, 5, 9, 13, 17]:
        g.hline(1, 24, cy)
    for cx in [1, 6, 12, 18, 24]:
        g.vline(cx, 1, 17)

    # ── SECTION C: Open Staging (top-right, rows 1-13) ──
    g.rect(28, 1, 53, 13)
    staging_obs = [
        (31, 3), (32, 3), (37, 3), (38, 3), (45, 3), (46, 3),
        (31, 7), (32, 7), (37, 7), (38, 7), (45, 7), (46, 7),
        (31, 11), (32, 11), (37, 11), (38, 11), (45, 11), (46, 11),
    ]
    for sx, sy in staging_obs:
        g.set(sx, sy, 'O')
    for px in [34, 42, 50]:
        for py in [5, 9]:
            g.set(px, py, 'P')

    # ── SECTION D: Office Rooms (right-mid, rows 15-27) ──
    g.rect(28, 15, 38, 19)    # Office 1
    g.rect(42, 15, 53, 19)    # Office 2
    g.rect(28, 23, 53, 27)    # Large conference room

    # Office interconnections (narrow doorways)
    g.hline(38, 42, 17); g.set(40, 17, 'D')   # Office1-Office2
    g.vline(33, 13, 15); g.set(33, 14, 'D')   # Staging-Office1
    g.vline(47, 13, 15); g.set(47, 14, 'D')   # Staging-Office2
    g.vline(33, 19, 23); g.set(33, 21, 'D')   # Office1-Conference
    g.vline(47, 19, 23); g.set(47, 21, 'D')   # Office2-Conference

    # ── SECTION E: Narrow Maze (bottom-left, rows 21-27) ──
    g.rect(1, 21, 12, 27)
    # Internal maze walls (partial, with gaps for winding path)
    g.vline(3, 22, 25, '#')
    g.vline(6, 22, 24, '#')
    g.vline(9, 22, 25, '#')
    g.hline(3, 6, 24)    # gap through left+center walls
    g.hline(6, 9, 22)    # gap through center+right walls
    g.hline(9, 11, 26)   # gap near right wall

    # ── SECTION F: Rough Terrain (center, rows 21-27) ──
    g.rect(14, 21, 25, 27, '~')

    # ── CONNECTIONS between sections ──
    g.hline(24, 28, 5)    # B aisles → C staging
    g.vline(6, 17, 21)    # B aisles → E maze
    g.vline(18, 17, 21)   # B aisles → F rough terrain
    g.vline(6, 27, 29)    # E maze → loading dock
    g.vline(20, 27, 29)   # F rough terrain → loading dock
    g.vline(40, 27, 29)   # D conference → loading dock
    g.hline(25, 28, 25)   # F rough terrain → D conference

    g.set(1, 33, '1')     # loading dock, far left
    g.set(53, 1, '2')     # staging, top-right
    g.set(1, 1, '3')      # aisles, top-left
    g.set(53, 33, '4')    # loading dock, far right

    save("map-10-gauntlet.txt", g, {
        "Name": "The Gauntlet",
        "Size": "55x35",
        "Robots": "4 (1, 2, 3, 4)",
        "Difficulty": "10 - Maximum",
        "Tests": "All features: aisles, open space, obstacles, rooms, doorways, maze, rough terrain",
        "Description": "Full warehouse with 6 distinct zones testing every mapping challenge simultaneously."
    })


if __name__ == '__main__':
    print("Generating warehouse test maps...\n")
    map_01()
    map_02()
    map_03()
    map_04()
    map_05()
    map_06()
    map_07()
    map_08()
    map_09()
    map_10()
    print(f"\nAll maps saved to: {BASE_DIR}")
