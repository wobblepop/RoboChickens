"""Generate the negotiation-duck-hunt chicken as a BMP file.
Sprite data from frauddyscotty.com/projects/negotiation-duck-hunt.html"""
import struct

# Palette from the game
PAL = {
    'black': (24, 24, 24),
    'white': (248, 248, 248),
    'cream': (240, 224, 176),
    'gray': (200, 200, 200),
    'orange': (232, 104, 32),
    'red': (216, 32, 32),
    'yellow': (248, 216, 48),
}
BG = (135, 206, 235)  # sky blue background

# Sprite data: each entry is (color_name, [[x, y, w, h], ...])
BODY = [
    ('white', [[4,5,5,1],[3,6,7,1],[3,7,7,1],[4,8,5,1],[5,9,3,1]]),
    ('cream', [[5,7,3,1],[4,8,4,1],[5,9,3,1]]),
    ('white', [[10,3,2,1],[9,4,4,1],[9,5,4,1],[9,6,3,1]]),
    ('red', [[10,2,2,1],[11,1,1,1]]),
    ('orange', [[13,4,2,1],[13,5,1,1]]),
    ('red', [[12,6,1,1],[13,6,1,1]]),
    ('black', [[12,4,1,1]]),
    ('white', [[2,4,2,1],[1,3,2,1],[1,2,1,1]]),
]
WING_UP = [
    ('gray', [[6,0,2,1],[5,1,4,1],[5,2,4,1],[6,3,3,1],[6,4,3,1]]),
]

# Combine body + wing-up frame
all_parts = WING_UP + BODY

# Determine grid size
max_x = 0
max_y = 0
for color_name, rects in all_parts:
    for r in rects:
        max_x = max(max_x, r[0] + r[2])
        max_y = max(max_y, r[1] + r[3])

grid_w = max_x + 2  # +2 for the outline padding
grid_h = max_y + 2

# Render to pixel grid (with outline like the game does)
pixels = [[BG for _ in range(grid_w)] for _ in range(grid_h)]

# First pass: draw black outlines (each rect expanded by 1 pixel on all sides)
for color_name, rects in all_parts:
    for r in rects:
        x, y, w, h = r
        for py in range(max(0, y-1), min(grid_h, y+h+1)):
            for px in range(max(0, x-1), min(grid_w, x+w+1)):
                pixels[py][px] = PAL['black']

# Second pass: draw colored fill
for color_name, rects in all_parts:
    color = PAL[color_name]
    for r in rects:
        x, y, w, h = r
        for py in range(y, y+h):
            for px in range(x, x+w):
                if 0 <= py < grid_h and 0 <= px < grid_w:
                    pixels[py][px] = color

# Scale up for visibility
scale = 6
out_w = grid_w * scale
out_h = grid_h * scale

# BMP format
row_bytes = out_w * 3
row_padding = (4 - (row_bytes % 4)) % 4
padded_row_size = row_bytes + row_padding
pixel_data_size = padded_row_size * out_h
file_size = 54 + pixel_data_size

with open("chicken.bmp", "wb") as f:
    # File header (14 bytes)
    f.write(b'BM')
    f.write(struct.pack('<I', file_size))
    f.write(struct.pack('<HH', 0, 0))
    f.write(struct.pack('<I', 54))

    # DIB header (40 bytes)
    f.write(struct.pack('<I', 40))
    f.write(struct.pack('<i', out_w))
    f.write(struct.pack('<i', out_h))
    f.write(struct.pack('<HH', 1, 24))
    f.write(struct.pack('<I', 0))
    f.write(struct.pack('<I', pixel_data_size))
    f.write(struct.pack('<i', 2835))
    f.write(struct.pack('<i', 2835))
    f.write(struct.pack('<I', 0))
    f.write(struct.pack('<I', 0))

    # Pixel data (bottom-up, BGR)
    for y in range(out_h - 1, -1, -1):
        grid_y = y // scale
        row_data = bytearray()
        for x in range(out_w):
            grid_x = x // scale
            r, g, b = pixels[grid_y][grid_x]
            row_data.extend([b, g, r])
        row_data.extend(b'\x00' * row_padding)
        f.write(row_data)

print(f"Created chicken.bmp ({out_w}x{out_h} pixels, {grid_w}x{grid_h} sprite grid, {file_size} bytes)")
