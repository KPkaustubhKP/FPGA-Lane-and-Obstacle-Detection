// =============================================================================
//  sv_patches/VERILATOR_COMPAT.md
//
//  Verilator is a lint-strict, 2-state simulator. Below are the most common
//  issues found in RTL written for ModelSim/VCS that break under Verilator,
//  and the exact fixes to apply to each file in this project.
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 1 — lane_detection_top.sv
//  Add output ports that sim_main.cpp reads.
//  The harness expects these signals at the top level:
//
//    output logic        edge_valid,     // from sobel/roi stage
//    output logic [9:0]  edge_x,         // pixel column of valid edge
//    output logic [8:0]  edge_y,         // pixel row    of valid edge
//    output logic        lane_valid,     // Hough FSM done
//    output logic [10:0] rho_left,       // accumulator bin (biased)
//    output logic [7:0]  theta_left,     // degree bin 0-179
//    output logic [10:0] rho_right,
//    output logic [7:0]  theta_right,
//
//  If these already exist, verify bit-widths match the params:
//    edge_x width  = $clog2(IMG_WIDTH)
//    edge_y width  = $clog2(IMG_HEIGHT)
//    rho   width   = $clog2(RHO_BINS)   (must hold 0..RHO_BINS-1)
//    theta width   = $clog2(THETA_BINS)
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 2 — hough_transform.sv  (most likely to need work)
//
//  ISSUE A: 2D packed arrays used as BRAM
//  Verilator supports 2D arrays but warns on implicit width truncation.
//  Replace:
//    logic [7:0] accum [0:RHO_BINS-1][0:THETA_BINS-1];
//  With explicit dimensions using localparams:
//    localparam int RB = RHO_BINS;
//    localparam int TB = THETA_BINS;
//    logic [7:0] accum [RB][TB];         // OK in Verilator ≥ 4.x
//
//  ISSUE B: $signed() in always_ff
//  Replace any use of $signed() applied to a parameter difference with a
//  signed localparam:
//    localparam signed [11:0] RHO_OFFSET = RHO_BINS / 2;
//    ...
//    rho_val = $signed(rho_raw) - RHO_OFFSET;   // fine
//
//  ISSUE C: for-loop variable declared outside always block
//  Verilator requires loop variables to be declared inside the for():
//    // BAD (ModelSim accepts this, Verilator -Wall warns):
//    integer i;
//    always_ff @(posedge clk) for (i = 0; ...) ...
//
//    // GOOD:
//    always_ff @(posedge clk) for (int i = 0; ...) ...
//
//  ISSUE D: Unsized integer literals in parameter context
//    // BAD:
//    parameter HT_THRESHOLD = 50;   // becomes 32-bit but comparison width mismatch
//    // GOOD:
//    parameter int unsigned HT_THRESHOLD = 50;
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 3 — gaussian_blur.sv / sobel_edge_detector.sv
//
//  ISSUE: Line buffers declared as dynamic/variable-size arrays.
//  Verilator requires all array dimensions to be constants at elaboration.
//
//  Replace:
//    logic [7:0] line_buf_0 [0:IMG_WIDTH-1];
//    logic [7:0] line_buf_1 [0:IMG_WIDTH-1];
//  With:
//    logic [7:0] line_buf_0 [IMG_WIDTH];  // unpacked, fine in Verilator
//    logic [7:0] line_buf_1 [IMG_WIDTH];
//
//  ISSUE: Implicit net types.
//  Add at the top of each file (after `timescale if present):
//    `default_nettype none
//  This makes Verilator flag any undeclared wire (good practice anyway).
//
//  ISSUE: $display with %0d format in always_ff
//  Verilator supports $display but it's evaluated at simulation runtime,
//  not synthesis. Wrap in `ifdef SIMULATION:
//    `ifdef SIMULATION
//      $display("edge at (%0d,%0d)", x_out, y_out);
//    `endif
//  Add -DSIMULATION to CFLAGS in the Makefile (already included).
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 4 — lane_roi_filter.sv
//
//  ISSUE: Comparison of unsigned to negative literal
//    // BAD: pixel_y >= ROI_ROW_TOP  where ROI_ROW_TOP can be 0 → unsigned
//    // GOOD: already fine if both sides are logic (unsigned). Just make sure
//    //       your ROI params are declared as:
//    parameter int unsigned ROI_ROW_TOP = 288;
//    parameter int unsigned ROI_ROW_BOT = 479;
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 5 — ALL FILES  (global, apply once)
//
//  Remove or guard `timescale directives.
//  Verilator ignores `timescale but warns if inconsistent across files.
//  Safe approach: remove `timescale from all RTL files and only keep it
//  in testbenches (which Verilator does not compile).
//
//  Also: ensure every file uses:
//    `default_nettype none    // at top
//    `default_nettype wire    // at bottom (after endmodule)
//  This prevents accidental implicit nets that silently simulate as X.
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
//  PATCH 6 — hough_transform.sv  (trig LUT)
//
//  If your LUT is initialised with $readmemh / $readmemb, Verilator supports
//  this but the .mem file must be in the working directory when you run the
//  binary.  If the LUT is inline (case statement or parameter array), no change
//  is needed.
//
//  Example of a Verilator-safe inline ROM:
//
//    localparam int COS_LUT [0:179] = '{
//        1024, 1023, 1022, ...  // cos(θ) × 1024, θ = 0..179°
//    };
//    localparam int SIN_LUT [0:179] = '{
//        0, 17, 35, ...
//    };
//
//  A Python snippet to generate these values is in scripts/gen_trig_lut.py
// ─────────────────────────────────────────────────────────────────────────────
