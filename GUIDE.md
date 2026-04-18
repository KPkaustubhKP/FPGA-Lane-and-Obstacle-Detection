# Verilator Verification — Step-by-Step Guide
**FPGA Lane Detection | Kaustubh Pandey · Krishanu Dey**

---

## What this does

Feeds a real PNG/JPG image from your laptop through the **exact RTL** (`lane_detection_top.sv`) cycle-accurately using Verilator. You get:

| Output file | Contents |
|---|---|
| `<image>_edges.png` | Binary edge map from Sobel + threshold stage |
| `<image>_lanes.png` | Your image with detected lane lines drawn on it |
| `lane_results.csv`  | ρ and θ values for left and right lanes |
| `lane_detection.vcd` | Full waveform dump (only when `VCD=1`) |

---

## Step 1 — Install Verilator

### Ubuntu / Debian (recommended)
```bash
sudo apt update
sudo apt install -y verilator build-essential curl

# Verify version (need ≥ 4.210 for --timing flag)
verilator --version
```

> If your distro ships an old Verilator (< 4.210), build from source:
> ```bash
> sudo apt install -y git autoconf flex bison libfl2 libfl-dev help2man
> git clone https://github.com/verilator/verilator
> cd verilator
> autoconf
> ./configure
> make -j$(nproc)
> sudo make install
> ```

### macOS (Homebrew)
```bash
brew install verilator
```

### Windows
Use WSL2 (Ubuntu) and follow the Ubuntu steps above.

---

## Step 2 — Arrange your files

Create a working directory and place the files exactly as shown:

```
lane_verify/
├── Makefile                  ← provided
├── cpp/
│   └── sim_main.cpp          ← provided
├── scripts/
│   ├── gen_test_pattern.py   ← provided
│   └── gen_trig_lut.py       ← provided
└── sv/                       ← YOU create this folder
    ├── rgb_to_gray.sv
    ├── gaussian_blur.sv
    ├── sobel_edge_detector.sv
    ├── lane_roi_filter.sv
    ├── hough_transform.sv
    └── lane_detection_top.sv
```

```bash
mkdir -p lane_verify/sv
cp *.sv lane_verify/sv/
```

---

## Step 3 — Apply Verilator compatibility patches

Read `sv_patches/VERILATOR_COMPAT.md` carefully. The four most critical changes:

### 3a. Add missing top-level ports (lane_detection_top.sv)

The C++ harness reads these ports directly. Confirm they exist in `lane_detection_top.sv`:

```systemverilog
output logic        edge_valid,
output logic [9:0]  edge_x,      // adjust width = $clog2(IMG_WIDTH)
output logic [8:0]  edge_y,      // adjust width = $clog2(IMG_HEIGHT)
output logic        lane_valid,
output logic [10:0] rho_left,    // RHO_BINS=1024 → 10-bit, biased
output logic [7:0]  theta_left,  // 0-179
output logic [10:0] rho_right,
output logic [7:0]  theta_right,
```

If any are missing, wire them out from the internal submodule signals:

```systemverilog
// Inside lane_detection_top — connect internal wires to outputs
assign edge_valid   = roi_edge_valid;
assign edge_x       = roi_x_out;
assign edge_y       = roi_y_out;
assign lane_valid   = ht_lane_valid;
assign rho_left     = ht_rho_left;
assign theta_left   = ht_theta_left;
assign rho_right    = ht_rho_right;
assign theta_right  = ht_theta_right;
```

### 3b. Fix for-loop variable scope (hough_transform.sv)

```systemverilog
// CHANGE THIS:
integer i, j;
always_ff @(posedge clk) begin
    for (i = 0; i < THETA_BINS; i++) ...

// TO THIS:
always_ff @(posedge clk) begin
    for (int i = 0; i < THETA_BINS; i++) ...
```

### 3c. Fix 2D array declaration (hough_transform.sv)

```systemverilog
// CHANGE THIS:
logic [7:0] accum [0:RHO_BINS-1][0:THETA_BINS-1];

// TO THIS:
logic [7:0] accum [RHO_BINS][THETA_BINS];
```

### 3d. Add `default_nettype none to all RTL files

At the top of each `.sv` file (after any `timescale):
```systemverilog
`default_nettype none
```
At the very bottom (after `endmodule`):
```systemverilog
`default_nettype wire
```

---

## Step 4 — Run lint check first

Before compiling, catch all SV issues:

```bash
cd lane_verify
make lint
```

Fix every `-Wall` warning. Verilator treats width mismatches and implicit nets as real bugs.

---

## Step 5 — Build

```bash
cd lane_verify
make
```

First build downloads `stb_image.h` (image loader, ~1000-line header, no deps), then Verilates your RTL and compiles the harness. Takes ~30 seconds.

Expected output:
```
Verilating lane_detection_top...
make[1]: Entering directory '.../obj_dir'
...
Build OK → obj_dir/Vlane_detection_top
```

---

## Step 6 — Run with your image

```bash
# Basic run (640×480 image recommended)
make run IMG=/path/to/your/road_image.png

# With custom Sobel threshold (try lower values if no lanes detected)
make run IMG=/path/to/road.png THRESH=48

# Enable waveform dump (GTKWave)
make run IMG=/path/to/road.png VCD=1
```

### Image requirements
| Requirement | Recommendation |
|---|---|
| Format | PNG, JPG, BMP (any colour) |
| Resolution | 640×480 preferred (auto-resized otherwise) |
| Content | Straight or mildly curved road with visible lane markings |
| Camera angle | Forward-facing dashcam view works best |

If your image is a different resolution, override the DUT parameters:
```bash
make run IMG=road_1280x720.png IMG_WIDTH=1280 IMG_HEIGHT=720 ROI_ROW_TOP=430
```

---

## Step 7 — Interpret results

```
╔══════════════════════════════════════════════╗
║            Lane Detection Results             ║
╚══════════════════════════════════════════════╝
  ✓ Left  lane : ρ = -312 px,  θ =  47°
  ✓ Right lane : ρ =  289 px,  θ = 134°
  (lane_valid asserted 18432 cycles after end-of-frame vsync)
```

- **ρ** — perpendicular distance from image centre to lane line (pixels)
- **θ** — angle of normal to lane line (degrees, 0=horizontal normal)
- Left lane θ should be **< 90°**, right lane **≥ 90°** — the module classifies them this way

Open the output images:
```bash
eog road.png_edges.png   # edge map
eog road.png_lanes.png   # lane overlay
```

---

## Step 8 — View waveforms (optional)

```bash
sudo apt install gtkwave
gtkwave lane_detection.vcd
```

Add these signal groups in GTKWave:
- `clk`, `rst_n`, `vsync`, `hsync`, `pixel_valid`
- `edge_valid`, `edge_x`, `edge_y`
- `lane_valid`, `rho_left`, `theta_left`, `rho_right`, `theta_right`

---

## Troubleshooting

### `lane_valid` never asserts

1. **Lower `HT_THRESHOLD`** — `make run IMG=x.png HT_THRESHOLD=20`
2. **Check edge map** — if `_edges.png` is all black, the Sobel threshold is too high. Try `THRESH=32`.
3. **Check ROI** — if edges are outside ROI, lower `ROI_ROW_TOP`: `make run IMG=x.png ROI_ROW_TOP=240`
4. **Pipeline not flushed** — very rarely the line buffers need more warm-up. The harness adds `2×W` extra cycles which should be sufficient.

### Verilator compile error: `Unsupported: ... --timing`

Your Verilator is too old. Either:
- Remove `--timing` from `VFLAGS` in Makefile (needed only if RTL uses `#delays`)
- Upgrade Verilator to ≥ 4.210

### `error: 'Vlane_detection_top' has no member named 'edge_valid'`

The port is missing from your `lane_detection_top.sv`. See Step 3a above.

### Width mismatch warnings

Run `make lint` and fix all `-Wall` warnings before building. Common fix:
```systemverilog
// Cast to matching width
assign edge_x = 10'(internal_x_signal);
```

### Simulation is very slow (Hough wait takes minutes)

The Hough FSM scan is `RHO_BINS × THETA_BINS = 184,320` cycles. At ~50 MHz simulation speed on a laptop this is ~4 seconds. It is normal. The harness breaks early once `lane_valid` asserts.

---

## Running multiple images in batch

```bash
for img in ~/dashcam_frames/*.png; do
    make run IMG="$img" 2>&1 | tail -5
    echo "---"
done
```

Results for each image are appended to `lane_results.csv` (edit `sim_main.cpp` to append instead of overwrite if running batch).

---

## Quick-reference command cheatsheet

```bash
# One-time setup
sudo apt install verilator build-essential curl
mkdir lane_verify/sv && cp *.sv lane_verify/sv/

# Build
cd lane_verify && make

# Lint only
make lint

# Run
make run IMG=~/Pictures/road.jpg

# Run with all params explicit
make run IMG=road.png IMG_WIDTH=640 IMG_HEIGHT=480 \
         EDGE_THRESH=48 ROI_ROW_TOP=260 HT_THRESHOLD=30 VCD=0

# Smoke test (no real image needed)
make smoke

# View waveforms
gtkwave lane_detection.vcd

# Clean
make clean
```
