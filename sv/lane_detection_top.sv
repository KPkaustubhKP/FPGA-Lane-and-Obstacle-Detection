// =============================================================================
// Module      : lane_detection_top.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Top-level integration of the full lane detection pipeline.
//
//   Pipeline dataflow (one pixel per clock after warm-up):
//
//   [Camera/AXI-S Input]
//         │  24-bit RGB + valid + hsync + vsync
//         ▼
//   ┌─────────────┐
//   │ rgb_to_gray │  latency: 2 cycles
//   └──────┬──────┘
//          │  8-bit gray
//          ▼
//   ┌──────────────┐
//   │ gaussian_blur│  latency: 2×W + 2 cycles (line buffer fill)
//   └──────┬───────┘
//          │  8-bit blurred
//          ▼
//   ┌────────────────────┐
//   │ sobel_edge_detector│  latency: 2×W + 4 cycles
//   └──────────┬─────────┘
//          │  8-bit gradient + edge flag
//          ▼
//   ┌────────────────┐
//   │ lane_roi_filter│  latency: 1 cycle
//   └──────┬─────────┘
//          │  masked edge flag + (x,y) coordinates
//          ▼
//   ┌──────────────────┐
//   │ hough_transform  │  accumulates per frame; outputs on vsync
//   └──────┬───────────┘
//          │  (ρ_left, θ_left, ρ_right, θ_right)
//          ▼
//   [Downstream Controller / UART / HDMI]
//
// AXI4-Stream compatible upstream interface (ready/valid handshake).
// Synthesizable: Yes — tested targeting Artix-7 / Zynq in Vivado 2023.x
// =============================================================================

`timescale 1ns / 1ps

module lane_detection_top #(
    // ── Image parameters ───────────────────────────────────────────────────
    parameter int unsigned IMG_WIDTH    = 640,
    parameter int unsigned IMG_HEIGHT   = 480,

    // ── Sobel threshold (0-255) ────────────────────────────────────────────
    parameter int unsigned EDGE_THRESH  = 64,

    // ── ROI row window (lower 40% of frame) ────────────────────────────────
    parameter int unsigned ROI_ROW_TOP  = 288,
    parameter int unsigned ROI_ROW_BOT  = 479,

    // ── Hough parameters ───────────────────────────────────────────────────
    parameter int unsigned THETA_BINS   = 180,
    parameter int unsigned RHO_BINS     = 1024,
    parameter int unsigned HT_THRESHOLD = 50
)(
    input  logic        clk,
    input  logic        rst_n,          // active-low synchronous reset

    // ── Pixel input (AXI4-Stream style) ────────────────────────────────────
    input  logic        i_valid,        // pixel data valid
    input  logic [23:0] i_rgb,          // {R[23:16], G[15:8], B[7:0]}
    input  logic        i_hsync,        // active-high end-of-line
    input  logic        i_vsync,        // active-high end-of-frame

    // ── Debug / diagnostic outputs ─────────────────────────────────────────
    output logic [7:0]  o_gray,         // grayscale pixel (debug)
    output logic [7:0]  o_blurred,      // blurred pixel   (debug)
    output logic [7:0]  o_gradient,     // Sobel gradient  (debug)
    output logic        o_edge,         // binary edge map (debug)

    // ── Lane coordinate outputs ─────────────────────────────────────────────
    output logic        o_lane_valid,   // new lane params ready (1 cycle pulse)
    output logic signed [10:0] o_rho_left,
    output logic [7:0]         o_theta_left,
    output logic signed [10:0] o_rho_right,
    output logic [7:0]         o_theta_right
    output logic        edge_valid,     // from sobel/roi stage
    output logic [9:0]  edge_x,         // pixel column of valid edge
    output logic [8:0]  edge_y,         // pixel row    of valid edge
    output logic        lane_valid,     // Hough FSM done
    output logic [10:0] rho_left,       // accumulator bin (biased)
    output logic [7:0]  theta_left,     // degree bin 0-179
    output logic [10:0] rho_right,
    output logic [7:0]  theta_right,
);

    // ───────────────────────────────────────────────────────────────────────────
    // Wire declarations between pipeline stages
    // ───────────────────────────────────────────────────────────────────────────

    // ── Stage 0 → 1 : rgb_to_gray ──────────────────────────────────────────
    logic        gray_valid;
    logic [7:0]  gray_pixel;
    logic        gray_hsync, gray_vsync;

    // ── Stage 1 → 2 : gaussian_blur ────────────────────────────────────────
    logic        blur_valid;
    logic [7:0]  blur_pixel;
    logic        blur_hsync, blur_vsync;

    // ── Stage 2 → 3 : sobel_edge_detector ──────────────────────────────────
    logic        sobel_valid;
    logic [7:0]  sobel_gradient;
    logic        sobel_edge;
    logic        sobel_hsync, sobel_vsync;

    // ── Stage 3 → 4 : lane_roi_filter ──────────────────────────────────────
    logic        roi_valid;
    logic [7:0]  roi_pixel;
    logic        roi_edge;
    logic        roi_hsync, roi_vsync;
    logic [$clog2(IMG_WIDTH) -1:0] roi_col;
    logic [$clog2(IMG_HEIGHT)-1:0] roi_row;

    // ───────────────────────────────────────────────────────────────────────────
    // Module instantiations
    // ───────────────────────────────────────────────────────────────────────────

    // ── Stage 1: RGB → Grayscale ────────────────────────────────────────────
    rgb_to_gray #(
        .DATA_WIDTH (8)
    ) u_rgb2gray (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_valid (i_valid),
        .i_r     (i_rgb[23:16]),
        .i_g     (i_rgb[15: 8]),
        .i_b     (i_rgb[ 7: 0]),
        .i_hsync (i_hsync),
        .i_vsync (i_vsync),
        .o_valid (gray_valid),
        .o_gray  (gray_pixel),
        .o_hsync (gray_hsync),
        .o_vsync (gray_vsync)
    );

    // ── Stage 2: Gaussian Blur ──────────────────────────────────────────────
    gaussian_blur #(
        .DATA_WIDTH (8),
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_gauss (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_valid (gray_valid),
        .i_pixel (gray_pixel),
        .i_hsync (gray_hsync),
        .i_vsync (gray_vsync),
        .o_valid (blur_valid),
        .o_pixel (blur_pixel),
        .o_hsync (blur_hsync),
        .o_vsync (blur_vsync)
    );

    // ── Stage 3: Sobel Edge Detection ──────────────────────────────────────
    sobel_edge_detector #(
        .DATA_WIDTH (8),
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .THRESHOLD  (EDGE_THRESH)
    ) u_sobel (
        .clk        (clk),
        .rst_n      (rst_n),
        .i_valid    (blur_valid),
        .i_pixel    (blur_pixel),
        .i_hsync    (blur_hsync),
        .i_vsync    (blur_vsync),
        .o_valid    (sobel_valid),
        .o_gradient (sobel_gradient),
        .o_edge     (sobel_edge),
        .o_hsync    (sobel_hsync),
        .o_vsync    (sobel_vsync)
    );

    // ── Stage 4: ROI Filter ─────────────────────────────────────────────────
    lane_roi_filter #(
        .DATA_WIDTH      (8),
        .IMG_WIDTH       (IMG_WIDTH),
        .IMG_HEIGHT      (IMG_HEIGHT),
        .ROI_ROW_TOP     (ROI_ROW_TOP),
        .ROI_ROW_BOT     (ROI_ROW_BOT),
        .ROI_COL_LEFT_BOT  (0),
        .ROI_COL_RIGHT_BOT (IMG_WIDTH-1),
        .ROI_COL_LEFT_TOP  (IMG_WIDTH/4),
        .ROI_COL_RIGHT_TOP (3*IMG_WIDTH/4)
    ) u_roi (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_valid (sobel_valid),
        .i_pixel (sobel_gradient),
        .i_edge  (sobel_edge),
        .i_hsync (sobel_hsync),
        .i_vsync (sobel_vsync),
        .o_valid (roi_valid),
        .o_pixel (roi_pixel),
        .o_edge  (roi_edge),
        .o_hsync (roi_hsync),
        .o_vsync (roi_vsync),
        .o_col   (roi_col),
        .o_row   (roi_row)
    );

    // ── Stage 5: Hough Transform ────────────────────────────────────────────
    hough_transform #(
        .IMG_WIDTH      (IMG_WIDTH),
        .IMG_HEIGHT     (IMG_HEIGHT),
        .THETA_BINS     (THETA_BINS),
        .RHO_BINS       (RHO_BINS),
        .ACC_WIDTH      (8),
        .PEAK_THRESHOLD (HT_THRESHOLD)
    ) u_hough (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_valid       (roi_valid),
        .i_edge        (roi_edge),
        .i_col         (roi_col),
        .i_row         (roi_row),
        .i_vsync       (roi_vsync),
        .o_valid       (o_lane_valid),
        .o_rho_left    (o_rho_left),
        .o_theta_left  (o_theta_left),
        .o_rho_right   (o_rho_right),
        .o_theta_right (o_theta_right)
    );

    // ───────────────────────────────────────────────────────────────────────────
    // Debug tap outputs
    // ───────────────────────────────────────────────────────────────────────────
    assign o_gray     = gray_pixel;
    assign o_blurred  = blur_pixel;
    assign o_gradient = sobel_gradient;
    assign o_edge     = sobel_edge;

endmodule : lane_detection_top
