// =============================================================================
// Module      : rgb_to_gray.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Converts 24-bit RGB pixel to 8-bit grayscale using the
//               ITU-R BT.601 weighted approximation:
//                   Gray ≈ (77·R + 150·G + 29·B) >> 8
//               Coefficients sum to 256, so the >> 8 is exact.
//               All arithmetic is integer-only (no floating-point).
//               Latency: 2 clock cycles (fully pipelined).
// Synthesizable: Yes — Vivado / Quartus compatible
// =============================================================================

`timescale 1ns / 1ps

module rgb_to_gray #(
    parameter DATA_WIDTH = 8    // bits per colour channel (R, G, or B)
)(
    input  logic                    clk,
    input  logic                    rst_n,          // active-low synchronous reset

    // ── Upstream (source) handshake ─────────────────────────────────────────
    input  logic                    i_valid,        // upstream pixel valid
    input  logic [DATA_WIDTH-1:0]   i_r,            // red channel
    input  logic [DATA_WIDTH-1:0]   i_g,            // green channel
    input  logic [DATA_WIDTH-1:0]   i_b,            // blue channel
    input  logic                    i_hsync,        // horizontal sync (pass-through)
    input  logic                    i_vsync,        // vertical sync   (pass-through)

    // ── Downstream (sink) ───────────────────────────────────────────────────
    output logic                    o_valid,        // downstream pixel valid
    output logic [DATA_WIDTH-1:0]   o_gray,         // 8-bit grayscale result
    output logic                    o_hsync,
    output logic                    o_vsync
);

    // ─────────────────────────────────────────────────────────────────────────
    // Coefficient table  (fixed-point, denominator = 256)
    //   0.299 × 256 = 76.544  → 77
    //   0.587 × 256 = 150.272 → 150
    //   0.114 × 256 = 29.184  → 29
    //   sum = 256  ✓ (no rounding error in the weight sum)
    // ─────────────────────────────────────────────────────────────────────────
    localparam int unsigned KR = 8'd77;
    localparam int unsigned KG = 8'd150;
    localparam int unsigned KB = 8'd29;

    // ── Stage 1: multiply each channel by its coefficient ────────────────────
    // Each product is at most 255 × 150 = 38 250  → fits in 16 bits
    logic [15:0] prod_r_s1, prod_g_s1, prod_b_s1;
    logic        valid_s1, hsync_s1, vsync_s1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            prod_r_s1 <= '0;
            prod_g_s1 <= '0;
            prod_b_s1 <= '0;
            valid_s1  <= 1'b0;
            hsync_s1  <= 1'b0;
            vsync_s1  <= 1'b0;
        end else begin
            prod_r_s1 <= KR * i_r;   // 8×8 → 16-bit unsigned multiply
            prod_g_s1 <= KG * i_g;
            prod_b_s1 <= KB * i_b;
            valid_s1  <= i_valid;
            hsync_s1  <= i_hsync;
            vsync_s1  <= i_vsync;
        end
    end

    // ── Stage 2: sum products then divide by 256 (arithmetic right-shift 8) ──
    // Maximum sum: 77×255 + 150×255 + 29×255 = 256×255 = 65 280 → fits in 16 bits
    logic [15:0] sum_s2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            o_gray  <= '0;
            o_valid <= 1'b0;
            o_hsync <= 1'b0;
            o_vsync <= 1'b0;
        end else begin
            sum_s2  <= prod_r_s1 + prod_g_s1 + prod_b_s1;
            o_gray  <= sum_s2[15:8];  // equivalent to >> 8; upper byte is the result
            o_valid <= valid_s1;
            o_hsync <= hsync_s1;
            o_vsync <= vsync_s1;
        end
    end

endmodule : rgb_to_gray
