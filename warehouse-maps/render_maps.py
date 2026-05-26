"""
Warehouse Map Renderer
Converts ASCII grid maps to architectural-style PNG floor plan images.
"""
from PIL import Image, ImageDraw, ImageFont
import os
import glob

CELL = 32

FLOOR = (248, 246, 241)
WALL = (58, 58, 66)
WALL_EDGE = (40, 40, 48)
RACK = (205, 203, 198)
RACK_LINE = (170, 168, 163)
OBS = (192, 172, 140)
OBS_EDGE = (148, 128, 96)
PILLAR = (88, 88, 96)
PILLAR_EDGE = (58, 58, 66)
DOOR_BG = (238, 234, 226)
DOOR_MARK = (175, 50, 50)
ROUGH = (215, 225, 202)
ROUGH_HATCH = (188, 198, 175)
GRID_CLR = (240, 238, 233)
SHADOW = (228, 226, 222)

ROBOTS = {
    'R': (41, 128, 185),
    '1': (41, 128, 185),
    '2': (231, 76, 60),
    '3': (46, 174, 96),
    '4': (243, 156, 18),
}

PASSABLE = set('.R1234OPD~')


def parse_map(fp):
    meta, grid = {}, []
    valid = set('#.R1234OPD~')
    with open(fp) as f:
        for ln in f:
            ln = ln.rstrip('\n\r')
            if not ln:
                continue
            if len(ln) > 3 and all(c in valid for c in ln):
                grid.append(ln)
            elif ln.startswith('# ') and ':' in ln:
                k, _, v = ln[2:].partition(':')
                meta[k.strip()] = v.strip()
    return grid, meta


def at(grid, x, y):
    if 0 <= y < len(grid) and 0 <= x < len(grid[0]):
        return grid[y][x]
    return None


def is_interior(grid, x, y):
    if at(grid, x, y) != '#':
        return False
    if x <= 0 or y <= 0 or x >= len(grid[0]) - 1 or y >= len(grid) - 1:
        return False
    return all(at(grid, x + dx, y + dy) == '#'
               for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)])


def render(grid, meta, out):
    H, W = len(grid), len(grid[0])
    mt, mb, ms = 60, 78, 36
    iw = W * CELL + 2 * ms
    ih = H * CELL + mt + mb

    img = Image.new('RGB', (iw, ih), (255, 255, 255))
    d = ImageDraw.Draw(img)
    ox, oy = ms, mt

    # Drop shadow behind map
    d.rectangle([ox + 4, oy + 4, ox + W * CELL + 4, oy + H * CELL + 4], fill=SHADOW)

    # ── Base fill ──
    for cy in range(H):
        for cx in range(W):
            px, py = ox + cx * CELL, oy + cy * CELL
            c = grid[cy][cx]
            if c == '#':
                fill = RACK if is_interior(grid, cx, cy) else WALL
                d.rectangle([px, py, px + CELL - 1, py + CELL - 1], fill=fill)
            elif c == '~':
                d.rectangle([px, py, px + CELL - 1, py + CELL - 1], fill=ROUGH)
            elif c == 'D':
                d.rectangle([px, py, px + CELL - 1, py + CELL - 1], fill=DOOR_BG)
            else:
                d.rectangle([px, py, px + CELL - 1, py + CELL - 1], fill=FLOOR)

    # ── Rack shelf detail ──
    for cy in range(H):
        for cx in range(W):
            if not is_interior(grid, cx, cy):
                continue
            px, py = ox + cx * CELL, oy + cy * CELL
            for i in range(1, 5):
                ly = py + i * CELL // 5
                d.line([px + 2, ly, px + CELL - 3, ly], fill=RACK_LINE, width=1)
            mid = px + CELL // 2
            d.line([mid, py + 2, mid, py + CELL - 3], fill=RACK_LINE, width=1)

    # ── Subtle floor grid ──
    for cy in range(H):
        for cx in range(W):
            if grid[cy][cx] in '.R1234':
                px, py = ox + cx * CELL, oy + cy * CELL
                d.line([px, py, px + CELL - 1, py], fill=GRID_CLR)
                d.line([px, py, px, py + CELL - 1], fill=GRID_CLR)

    # ── Wall boundary lines ──
    for cy in range(H):
        for cx in range(W):
            if grid[cy][cx] != '#':
                continue
            px, py = ox + cx * CELL, oy + cy * CELL
            edges = [
                (1, 0, px + CELL, py, px + CELL, py + CELL),
                (-1, 0, px, py, px, py + CELL),
                (0, 1, px, py + CELL, px + CELL, py + CELL),
                (0, -1, px, py, px + CELL, py),
            ]
            for dx, dy, x1, y1, x2, y2 in edges:
                nb = at(grid, cx + dx, cy + dy)
                if nb and nb in PASSABLE:
                    d.line([(x1, y1), (x2, y2)], fill=WALL_EDGE, width=2)

    # ── Wall inner shadow (depth effect) ──
    for cy in range(H):
        for cx in range(W):
            if grid[cy][cx] != '#':
                continue
            px, py = ox + cx * CELL, oy + cy * CELL
            nb_r = at(grid, cx + 1, cy)
            nb_b = at(grid, cx, cy + 1)
            if nb_r and nb_r in PASSABLE:
                shadow_clr = (35, 35, 42, 40)
                for si in range(1, 4):
                    sc = tuple(max(0, c - 8 * (4 - si)) for c in FLOOR)
                    nx = px + CELL + si
                    if cx + 1 < W:
                        d.line([(nx, py), (nx, py + CELL - 1)], fill=sc, width=1)
            if nb_b and nb_b in PASSABLE:
                for si in range(1, 4):
                    sc = tuple(max(0, c - 8 * (4 - si)) for c in FLOOR)
                    ny = py + CELL + si
                    if cy + 1 < H:
                        d.line([(px, ny), (px + CELL - 1, ny)], fill=sc, width=1)

    # ── Obstacles, pillars, doors, rough ──
    for cy in range(H):
        for cx in range(W):
            px, py = ox + cx * CELL, oy + cy * CELL
            c = grid[cy][cx]
            half = CELL // 2

            if c == 'O':
                mg = 4
                d.rectangle([px + mg + 2, py + mg + 2,
                             px + CELL - mg, py + CELL - mg], fill=SHADOW)
                d.rectangle([px + mg, py + mg,
                             px + CELL - 1 - mg, py + CELL - 1 - mg],
                            fill=OBS, outline=OBS_EDGE, width=1)
                d.line([(px + mg + 2, py + mg + 2),
                        (px + CELL - mg - 3, py + CELL - mg - 3)], fill=OBS_EDGE)
                d.line([(px + CELL - mg - 3, py + mg + 2),
                        (px + mg + 2, py + CELL - mg - 3)], fill=OBS_EDGE)

            elif c == 'P':
                mg = 6
                d.ellipse([px + mg + 2, py + mg + 2,
                           px + CELL - mg + 1, py + CELL - mg + 1], fill=SHADOW)
                d.ellipse([px + mg, py + mg,
                           px + CELL - 1 - mg, py + CELL - 1 - mg],
                          fill=PILLAR, outline=PILLAR_EDGE, width=2)

            elif c == 'D':
                d.polygon([
                    (px + half, py + 4),
                    (px + CELL - 4, py + half),
                    (px + half, py + CELL - 4),
                    (px + 4, py + half),
                ], outline=DOOR_MARK, width=2)

            elif c == '~':
                for i in range(3, CELL, 5):
                    d.line([(px + i, py), (px, py + i)], fill=ROUGH_HATCH, width=1)
                    if i > 0:
                        d.line([(px + CELL - 1, py + i),
                                (px + i, py + CELL - 1)], fill=ROUGH_HATCH, width=1)

    # ── Robot markers ──
    try:
        rfnt = ImageFont.truetype("arial.ttf", 14)
    except Exception:
        rfnt = ImageFont.load_default()

    for cy in range(H):
        for cx in range(W):
            c = grid[cy][cx]
            if c not in ROBOTS:
                continue
            px, py = ox + cx * CELL, oy + cy * CELL
            color = ROBOTS[c]
            cx2, cy2 = px + CELL // 2, py + CELL // 2
            r = CELL // 2 - 3

            for gi in range(5, 0, -1):
                gc = tuple(min(255, v + 30 * gi) for v in color)
                d.ellipse([cx2 - r - gi, cy2 - r - gi,
                           cx2 + r + gi, cy2 + r + gi], fill=gc)
            d.ellipse([cx2 - r, cy2 - r, cx2 + r, cy2 + r],
                      fill=color, outline='white', width=2)
            bb = d.textbbox((0, 0), c, font=rfnt)
            tw, th = bb[2] - bb[0], bb[3] - bb[1]
            d.text((cx2 - tw // 2, cy2 - th // 2 - 1), c,
                   fill='white', font=rfnt)

    # ── Title ──
    try:
        tfnt = ImageFont.truetype("arial.ttf", 20)
        sfnt = ImageFont.truetype("arial.ttf", 12)
    except Exception:
        tfnt = sfnt = ImageFont.load_default()

    d.text((ms, 8), meta.get('Name', ''), fill=(40, 40, 48), font=tfnt)
    info_parts = []
    for k in ['Size', 'Robots', 'Difficulty']:
        if k in meta:
            info_parts.append(meta[k])
    d.text((ms, 32), ' | '.join(info_parts), fill=(130, 130, 138), font=sfnt)

    # ── Legend ──
    try:
        lfnt = ImageFont.truetype("arial.ttf", 11)
    except Exception:
        lfnt = ImageFont.load_default()

    ly_pos = oy + H * CELL + 16
    lx = ms
    flat = ''.join(grid)

    items = [(WALL, "Wall"), (FLOOR, "Floor"), (RACK, "Shelving")]
    if 'O' in flat:
        items.append((OBS, "Obstacle"))
    if 'P' in flat:
        items.append((PILLAR, "Pillar"))
    if 'D' in flat:
        items.append((DOOR_BG, "Doorway"))
    if '~' in flat:
        items.append((ROUGH, "Rough Terrain"))

    for clr, lbl in items:
        d.rectangle([lx, ly_pos, lx + 14, ly_pos + 14],
                    fill=clr, outline=(140, 140, 148))
        d.text((lx + 18, ly_pos + 1), lbl, fill=(90, 90, 98), font=lfnt)
        bb = d.textbbox((0, 0), lbl, font=lfnt)
        lx += 18 + bb[2] - bb[0] + 16

    for rc in ['R', '1', '2', '3', '4']:
        if rc in flat:
            nm = "Robot" if rc == 'R' else f"Robot {rc}"
            d.ellipse([lx, ly_pos + 2, lx + 11, ly_pos + 13], fill=ROBOTS[rc])
            d.text((lx + 15, ly_pos + 1), nm, fill=(90, 90, 98), font=lfnt)
            bb = d.textbbox((0, 0), nm, font=lfnt)
            lx += 15 + bb[2] - bb[0] + 16

    # Outer border
    d.rectangle([ox - 1, oy - 1, ox + W * CELL, oy + H * CELL],
                outline=WALL_EDGE, width=2)

    img.save(out, 'PNG', optimize=True)
    print(f"  {os.path.basename(out):35s} {iw}x{ih}")


def main():
    base = os.path.dirname(os.path.abspath(__file__))
    odir = os.path.join(base, 'rendered')
    os.makedirs(odir, exist_ok=True)

    maps = sorted(glob.glob(os.path.join(base, 'map-*.txt')))
    print(f"Rendering {len(maps)} warehouse maps...\n")

    for mf in maps:
        grid, meta = parse_map(mf)
        if not grid:
            continue
        bn = os.path.splitext(os.path.basename(mf))[0]
        render(grid, meta, os.path.join(odir, bn + '.png'))

    print(f"\nSaved to: {odir}")


if __name__ == '__main__':
    main()
