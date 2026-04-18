#!/usr/bin/env python3
"""
gen_trig_lut.py — Generate fixed-point sin/cos LUT for hough_transform.sv
Outputs a SystemVerilog snippet ready to paste into the module.

Usage:
    python3 gen_trig_lut.py [scale=1024] [bins=180]
"""

import sys, math

SCALE = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
BINS  = int(sys.argv[2]) if len(sys.argv) > 2 else 180

cos_vals = [int(round(SCALE * math.cos(math.radians(i)))) for i in range(BINS)]
sin_vals = [int(round(SCALE * math.sin(math.radians(i)))) for i in range(BINS)]

# Bit-width needed (signed, SCALE can be negative for cos)
import math as _m
bits = int(_m.ceil(_m.log2(SCALE + 1))) + 1  # +1 for sign

print(f"// Auto-generated trig LUT — scale={SCALE}, bins={BINS}")
print(f"// Paste inside hough_transform module (before always blocks)\n")
print(f"localparam int signed COS_LUT [{BINS}] = '{{")
chunks = [cos_vals[i:i+12] for i in range(0, BINS, 12)]
for j, chunk in enumerate(chunks):
    sep = "" if j == len(chunks)-1 else ","
    print("    " + ", ".join(f"{v:5d}" for v in chunk) + ("," if j < len(chunks)-1 else ""))
print(f"}};\n")

print(f"localparam int signed SIN_LUT [{BINS}] = '{{")
chunks = [sin_vals[i:i+12] for i in range(0, BINS, 12)]
for j, chunk in enumerate(chunks):
    print("    " + ", ".join(f"{v:5d}" for v in chunk) + ("," if j < len(chunks)-1 else ""))
print(f"}};")

print(f"\n// Usage in always block:")
print(f"//   rho = (x * COS_LUT[theta_idx] + y * SIN_LUT[theta_idx]) >>> 10;")
print(f"//   (shift by $clog2({SCALE}) = {int(math.log2(SCALE))} bits)")
