#!/usr/bin/env python3
"""
gen_test_pattern.py  —  Generate a synthetic road image with lane markings.
Used for smoke-testing the Verilator harness without a real road image.

Usage:
    python3 gen_test_pattern.py [width=640] [height=480] [output=test_pattern.png]
"""

import sys
import math

try:
    from PIL import Image, ImageDraw
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

def make_pattern_pil(W, H, out):
    img = Image.new("RGB", (W, H), color=(80, 80, 80))   # grey tarmac
    draw = ImageDraw.Draw(img)

    # Road surface gradient (lighter near horizon)
    for y in range(H):
        shade = int(60 + 60 * y / H)
        draw.line([(0, y), (W, y)], fill=(shade, shade, shade))

    # Left lane marking (white solid line)
    # Line from bottom-left area to vanishing point (centre top)
    vp_x, vp_y = W // 2, H // 2   # vanishing point
    lx_bot = W // 5
    draw.line([(lx_bot, H - 1), (vp_x - 10, vp_y)], fill=(220, 220, 220), width=4)

    # Right lane marking
    rx_bot = 4 * W // 5
    draw.line([(rx_bot, H - 1), (vp_x + 10, vp_y)], fill=(220, 220, 220), width=4)

    # Dashed centre line
    steps = 12
    for i in range(steps):
        t0 = i / steps
        t1 = (i + 0.4) / steps
        x0 = int(vp_x + (W // 2 - vp_x) * t0)
        y0 = int(vp_y + (H - vp_y) * t0)
        x1 = int(vp_x + (W // 2 - vp_x) * t1)
        y1 = int(vp_y + (H - vp_y) * t1)
        draw.line([(x0, y0), (x1, y1)], fill=(200, 200, 200), width=2)

    img.save(out)
    print(f"Saved {out}  ({W}×{H})")

def make_pattern_raw(W, H, out):
    """Fallback: write a PPM file then rename to .png (no libs needed)."""
    import struct
    pixels = []
    vp_x, vp_y = W // 2, H // 2

    for y in range(H):
        row = []
        shade = int(60 + 60 * y / H)
        for x in range(W):
            r = g = b = shade

            # Left lane — simple distance-to-line check
            # Line from (W//5, H-1) to (vp_x-10, vp_y)
            def near_line(px, py, x0, y0, x1, y1, thresh=3):
                dx, dy = x1 - x0, y1 - y0
                denom = math.hypot(dx, dy)
                if denom == 0: return False
                dist = abs(dy * px - dx * py + x1 * y0 - y1 * x0) / denom
                # Also check within bounding box
                t = ((px - x0) * dx + (py - y0) * dy) / (denom * denom)
                return dist < thresh and 0 <= t <= 1

            if near_line(x, y, W//5, H-1, vp_x-10, vp_y):
                r = g = b = 220
            elif near_line(x, y, 4*W//5, H-1, vp_x+10, vp_y):
                r = g = b = 220

            row.extend([r, g, b])
        pixels.append(row)

    # Write PNG using zlib-compressed raw bytes
    import zlib, struct
    def png_chunk(name, data):
        c = zlib.crc32(name + data) & 0xffffffff
        return struct.pack('>I', len(data)) + name + data + struct.pack('>I', c)

    raw = b''
    for row in pixels:
        raw += b'\x00' + bytes(row)  # filter byte per row

    compressed = zlib.compress(raw, 9)
    ihdr = struct.pack('>IIBBBBB', W, H, 8, 2, 0, 0, 0)
    png = (b'\x89PNG\r\n\x1a\n'
           + png_chunk(b'IHDR', ihdr)
           + png_chunk(b'IDAT', compressed)
           + png_chunk(b'IEND', b''))
    with open(out, 'wb') as f:
        f.write(png)
    print(f"Saved {out}  ({W}×{H}) [fallback mode]")

if __name__ == "__main__":
    W   = int(sys.argv[1]) if len(sys.argv) > 1 else 640
    H   = int(sys.argv[2]) if len(sys.argv) > 2 else 480
    out = sys.argv[3]       if len(sys.argv) > 3 else "test_pattern.png"

    if HAS_PIL:
        make_pattern_pil(W, H, out)
    else:
        print("PIL not found — using fallback PNG writer (no anti-aliasing).")
        make_pattern_raw(W, H, out)
