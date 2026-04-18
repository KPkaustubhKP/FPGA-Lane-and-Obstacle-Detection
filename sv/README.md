# FPGA Lane Detection — SystemVerilog IP
**Authors:** Kaustubh Pandey (230959054) · Krishanu Dey (230959026)

---

## File Map

| File | Module | Latency |
|------|--------|---------|
| `rgb_to_gray.sv` | RGB → 8-bit Grayscale | 2 cycles |
| `gaussian_blur.sv` | 3×3 Gaussian Blur (line-buffer) | 2×W + 2 cycles |
| `sobel_edge_detector.sv` | Sobel Gx/Gy + magnitude + threshold | 2×W + 4 cycles |
| `lane_roi_filter.sv` | Trapezoidal ROI mask + coordinate counter | 1 cycle |
| `hough_transform.sv` | Hough Accumulator + peak detector | per-frame |
| `lane_detection_top.sv` | Top-level pipeline integration | sum of above |
| `tb_lane_detection.sv` | 5-task self-checking testbench | — |
| `vivado_project.tcl` | Vivado auto-project script | — |

---

## Pipeline Architecture

```
[Camera / AXI-S]
      │  24-bit RGB
      ▼
 rgb_to_gray          Gray = (77R + 150G + 29B) >> 8
      │  8-bit gray
      ▼
 gaussian_blur         3×3 kernel, line-buffer windowing
      │  8-bit blurred
      ▼
 sobel_edge_detector   |Gx|+|Gy|, threshold → edge map
      │  edge flag + (x,y)
      ▼
 lane_roi_filter        Trapezoidal ROI, coordinate counter
      │  masked edge + coords
      ▼
 hough_transform       ρ=x·cosθ+y·sinθ, BRAM accumulator,
      │                peak detect on vsync
      ▼
 (ρ_left,θ_left, ρ_right,θ_right)
```

---

## Key Design Decisions

### Fixed-Point Arithmetic
- All computation is integer — no floating-point units needed.
- Grayscale: coefficients {77, 150, 29} sum to 256 → right-shift 8 normalises exactly.
- Trig LUT: sin/cos × 1024 (Q1.10 fixed-point) → right-shift 10 after multiply.

### Line Buffers
- `gaussian_blur` and `sobel_edge_detector` each hold 2 row-deep shift-register FIFOs.
- Vivado maps these to BRAM (SRL32/RAMB18) automatically for `IMG_WIDTH ≥ 64`.

### Hough Accumulator
- BRAM array: `RHO_BINS × THETA_BINS` 8-bit saturating counters.
- After every frame's vsync, FSM scans accumulator → finds two peak bins
  (left lane θ < 90°, right lane θ ≥ 90°).
- Accumulator is cleared after each frame scan.

---

## Simulation (Icarus / ModelSim)

```bash
# Icarus Verilog
iverilog -g2012 -o sim.out \
  rgb_to_gray.sv gaussian_blur.sv sobel_edge_detector.sv \
  lane_roi_filter.sv hough_transform.sv lane_detection_top.sv \
  tb_lane_detection.sv
vvp sim.out

# View waves
gtkwave tb_lane_detection.vcd
```

---

## Vivado Synthesis

```bash
# Auto-create project (edit part name in TCL first)
vivado -mode batch -source vivado_project.tcl
```

Or manually:
1. Create new RTL project, language = SystemVerilog.
2. Add all `.sv` files; set top = `lane_detection_top`.
3. Add `tb_lane_detection.sv` to Simulation sources; set sim top = `tb_lane_detection`.
4. Run Synthesis → Implementation → Generate Bitstream.

**Recommended constraints (XDC):**
```tcl
create_clock -period 10.000 -name clk [get_ports clk]   ;# 100 MHz
set_input_delay  -clock clk 2.000 [all_inputs]
set_output_delay -clock clk 2.000 [all_outputs]
```

---

## Parameters (top-level)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `IMG_WIDTH` | 640 | Frame width in pixels |
| `IMG_HEIGHT` | 480 | Frame height in pixels |
| `EDGE_THRESH` | 64 | Sobel magnitude threshold (0–255) |
| `ROI_ROW_TOP` | 288 | ROI top row (0 = image top) |
| `ROI_ROW_BOT` | 479 | ROI bottom row |
| `THETA_BINS` | 180 | Hough angular resolution |
| `RHO_BINS` | 1024 | Hough ρ range |
| `HT_THRESHOLD` | 50 | Min votes for a lane line |

---

## References
1. Gao et al., "Development of Lane Detection System Based on FPGA," FISITA 2013.
2. Rout & Nesam, "Optimizing RGB to Grayscale, Gaussian Blur and Sobel-Filter on FPGAs," IEEE AI-IoT 2024.
3. Malmir & Shalchian, "Dual-stage lane detection based on Hough transform," Microprocessors & Microsystems 2019.
4. El Hajjouji et al., "FPGA Based Real-Time Lane Detection and Tracking," IEEE ICEIT 2016.
