// =============================================================================
// Module      : lane_roi_filter.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Region-of-Interest (ROI) filter for lane detection.
//               • Counts pixel coordinates (col, row) based on sync signals.
//               • Passes edge pixels ONLY when they fall inside the defined
//                 ROI window (lower portion of frame = road region).
//               • Pixels outside the ROI are masked to 0 (background).
//               • Also applies a trapezoidal / triangular ROI mask so that
//                 the sky and bonnet regions are always excluded.
//               Latency: 1 clock cycle (register stage on output).
// Synthesizable: Yes
// =============================================================================

`timescale 1ns / 1ps

module lane_roi_filter #(
    parameter int unsigned DATA_WIDTH  = 8,
    parameter int unsigned IMG_WIDTH   = 640,
    parameter int unsigned IMG_HEIGHT  = 480,

    // ROI bounds (rows, 0-indexed from top)
    // Only pixels with row_index in [ROI_ROW_TOP … ROI_ROW_BOT] are kept.
    // Default: lower 40% of frame.
    parameter int unsigned ROI_ROW_TOP = 288,   // 480 × 0.60
    parameter int unsigned ROI_ROW_BOT = 479,

    // ROI column limits at the BOTTOM of the trapezoid
    parameter int unsigned ROI_COL_LEFT_BOT  = 0,
    parameter int unsigned ROI_COL_RIGHT_BOT = 639,

    // ROI column limits at the TOP of the trapezoid (narrower for perspective)
    parameter int unsigned ROI_COL_LEFT_TOP  = 160,   // 640 × 0.25
    parameter int unsigned ROI_COL_RIGHT_TOP = 480    // 640 × 0.75
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── Upstream ────────────────────────────────────────────────────────────
    input  logic                    i_valid,
    input  logic [DATA_WIDTH-1:0]   i_pixel,    // binarised edge map (from Sobel)
    input  logic                    i_edge,     // edge flag from Sobel
    input  logic                    i_hsync,    // active-high end-of-line pulse
    input  logic                    i_vsync,    // active-high end-of-frame pulse

    // ── Downstream ──────────────────────────────────────────────────────────
    output logic                    o_valid,
    output logic [DATA_WIDTH-1:0]   o_pixel,    // masked pixel
    output logic                    o_edge,     // masked edge flag
    output logic                    o_hsync,
    output logic                    o_vsync,

    // ── Pixel coordinate outputs (used by downstream Hough Transform) ───────
    output logic [$clog2(IMG_WIDTH)-1:0]  o_col,
    output logic [$clog2(IMG_HEIGHT)-1:0] o_row
);

    // ─── Internal coordinate counters ─────────────────────────────────────────
    logic [$clog2(IMG_WIDTH) -1:0] col_cnt;
    logic [$clog2(IMG_HEIGHT)-1:0] row_cnt;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            col_cnt <= '0;
            row_cnt <= '0;
        end else if (i_valid) begin
            if (i_hsync) begin
                col_cnt <= '0;
                row_cnt <= (i_vsync) ? '0 : row_cnt + 1'b1;
            end else begin
                col_cnt <= (col_cnt == IMG_WIDTH - 1) ? '0 : col_cnt + 1'b1;
            end
        end
    end

    // ─── Trapezoidal ROI column-limit interpolation ────────────────────────────
    // At row = ROI_ROW_TOP the valid band is [COL_LEFT_TOP … COL_RIGHT_TOP].
    // At row = ROI_ROW_BOT the valid band is [COL_LEFT_BOT … COL_RIGHT_BOT].
    // Linear interpolation between the two (integer arithmetic):
    //
    //   col_left(row)  = COL_LEFT_TOP  + (row - ROI_ROW_TOP)
    //                    × (COL_LEFT_BOT  − COL_LEFT_TOP ) / ROI_HEIGHT
    //
    // To avoid division we pre-compute a fractional increment using a running
    // accumulator (Bresenham-style), but for simplicity here we use a divider
    // that Vivado can optimise away (constant denominator → shift-logic).

    localparam int unsigned ROI_HEIGHT = ROI_ROW_BOT - ROI_ROW_TOP;

    // Signed deltas for left and right edges
    localparam int signed DL = $signed(ROI_COL_LEFT_BOT)  - $signed(ROI_COL_LEFT_TOP);
    localparam int signed DR = $signed(ROI_COL_RIGHT_BOT) - $signed(ROI_COL_RIGHT_TOP);

    logic signed [10:0] col_left_limit, col_right_limit;
    logic signed [10:0] row_offset;

    always_comb begin
        row_offset       = $signed({1'b0, row_cnt}) - $signed({1'b0, ROI_ROW_TOP[9:0]});
        col_left_limit   = $signed(ROI_COL_LEFT_TOP[9:0])
                           + (DL[9:0] * row_offset) / $signed(ROI_HEIGHT[9:0]);
        col_right_limit  = $signed(ROI_COL_RIGHT_TOP[9:0])
                           + (DR[9:0] * row_offset) / $signed(ROI_HEIGHT[9:0]);
    end

    // ─── ROI mask decision ─────────────────────────────────────────────────────
    logic in_roi;

    always_comb begin
        in_roi = 1'b0;
        if (   (row_cnt >= ROI_ROW_TOP)
            && (row_cnt <= ROI_ROW_BOT)
            && ($signed({1'b0, col_cnt}) >= col_left_limit)
            && ($signed({1'b0, col_cnt}) <= col_right_limit))
            in_roi = 1'b1;
    end

    // ─── Output register ───────────────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            o_valid  <= 1'b0;
            o_pixel  <= '0;
            o_edge   <= 1'b0;
            o_hsync  <= 1'b0;
            o_vsync  <= 1'b0;
            o_col    <= '0;
            o_row    <= '0;
        end else begin
            o_valid  <= i_valid;
            o_pixel  <= (i_valid && in_roi) ? i_pixel : '0;
            o_edge   <= (i_valid && in_roi) ? i_edge  : 1'b0;
            o_hsync  <= i_hsync;
            o_vsync  <= i_vsync;
            o_col    <= col_cnt;
            o_row    <= row_cnt;
        end
    end

endmodule : lane_roi_filter
