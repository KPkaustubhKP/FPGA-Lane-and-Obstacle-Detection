// =============================================================================
// Module      : sobel_edge_detector.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Sobel operator for gradient-based edge detection.
//               Computes Gx and Gy from a 3×3 pixel window and produces
//               an 8-bit gradient magnitude estimate:
//                   |Gx| + |Gy|   (Manhattan approximation; avoids sqrt)
//               A configurable threshold then binarises the magnitude into
//               an edge map (1 = edge, 0 = background).
//
//               Sobel kernels:
//                   Gx               Gy
//                 -1  0  +1       -1 -2 -1
//                 -2  0  +2        0  0  0
//                 -1  0  +1       +1 +2 +1
//
//               Line-buffer architecture mirrors gaussian_blur.sv.
//               Latency: 2 × IMG_WIDTH + 4 clock cycles.
// Synthesizable: Yes
// =============================================================================

`timescale 1ns / 1ps

module sobel_edge_detector #(
    parameter int unsigned DATA_WIDTH  = 8,     // pixel bit width
    parameter int unsigned IMG_WIDTH   = 640,
    parameter int unsigned IMG_HEIGHT  = 480,
    parameter int unsigned THRESHOLD   = 64     // edge threshold (0-255)
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── Upstream ────────────────────────────────────────────────────────────
    input  logic                    i_valid,
    input  logic [DATA_WIDTH-1:0]   i_pixel,    // blurred grayscale pixel
    input  logic                    i_hsync,
    input  logic                    i_vsync,

    // ── Downstream ──────────────────────────────────────────────────────────
    output logic                    o_valid,
    output logic [DATA_WIDTH-1:0]   o_gradient, // magnitude |Gx|+|Gy| (clamped to 8-bit)
    output logic                    o_edge,     // 1 if magnitude > THRESHOLD
    output logic                    o_hsync,
    output logic                    o_vsync
);

    typedef logic [DATA_WIDTH-1:0] pixel_t;

    // ─── Line buffers ──────────────────────────────────────────────────────────
    pixel_t lb0 [0:IMG_WIDTH-1];
    pixel_t lb1 [0:IMG_WIDTH-1];

    logic [$clog2(IMG_WIDTH)-1:0] wr_ptr;

    // ─── 3×3 sliding window ────────────────────────────────────────────────────
    pixel_t win [0:2][0:2];   // win[row][col], row0 = oldest

    // ─── Line buffer + window fill ────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    win[r][c] <= '0;
        end else if (i_valid) begin
            // Update line buffers
            lb0[wr_ptr] <= lb1[wr_ptr];
            lb1[wr_ptr] <= i_pixel;

            // Slide window
            win[0][0] <= win[0][1]; win[0][1] <= win[0][2]; win[0][2] <= lb0[wr_ptr];
            win[1][0] <= win[1][1]; win[1][1] <= win[1][2]; win[1][2] <= lb1[wr_ptr];
            win[2][0] <= win[2][1]; win[2][1] <= win[2][2]; win[2][2] <= i_pixel;

            wr_ptr <= (wr_ptr == IMG_WIDTH - 1) ? '0 : wr_ptr + 1'b1;
        end
    end

    // ─── Stage 1: Gx and Gy computation ──────────────────────────────────────
    // Sobel kernels (signed arithmetic on unsigned pixels):
    //   Gx = (win[0][2] - win[0][0]) + 2*(win[1][2] - win[1][0]) + (win[2][2] - win[2][0])
    //   Gy = (win[2][0] + 2*win[2][1] + win[2][2]) - (win[0][0] + 2*win[0][1] + win[0][2])
    // Range of each: -4×255 … +4×255 = -1020 … +1020 → 11-bit signed

    logic signed [10:0] gx_s1, gy_s1;
    logic                valid_s1, hsync_s1, vsync_s1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            gx_s1    <= '0; gy_s1 <= '0;
            valid_s1 <= 1'b0; hsync_s1 <= 1'b0; vsync_s1 <= 1'b0;
        end else begin
            // Gx:  right column − left column (weighted)
            // Gx = (P02 - P00) + 2*(P12 - P10) + (P22 - P20)
            gx_s1 <= (  $signed({1'b0, win[0][2]}) - $signed({1'b0, win[0][0]})
                      + (($signed({1'b0, win[1][2]}) - $signed({1'b0, win[1][0]})) <<< 1)
                      +  $signed({1'b0, win[2][2]}) - $signed({1'b0, win[2][0]}));

            // Gy:  bottom row − top row (weighted)
            // Gy = (P20 - P00) + 2*(P21 - P01) + (P22 - P02)
            gy_s1 <= (  $signed({1'b0, win[2][0]}) - $signed({1'b0, win[0][0]})
                      + (($signed({1'b0, win[2][1]}) - $signed({1'b0, win[0][1]})) <<< 1)
                      +  $signed({1'b0, win[2][2]}) - $signed({1'b0, win[0][2]}));

            valid_s1 <= i_valid;
            hsync_s1 <= i_hsync;
            vsync_s1 <= i_vsync;
        end
    end

    // ─── Stage 2: absolute values ─────────────────────────────────────────────
    logic [9:0] abs_gx_s2, abs_gy_s2;
    logic        valid_s2, hsync_s2, vsync_s2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            abs_gx_s2 <= '0; abs_gy_s2 <= '0;
            valid_s2  <= 1'b0; hsync_s2 <= 1'b0; vsync_s2 <= 1'b0;
        end else begin
            abs_gx_s2 <= gx_s1[10] ? 10'(-gx_s1) : 10'(gx_s1);
            abs_gy_s2 <= gy_s1[10] ? 10'(-gy_s1) : 10'(gy_s1);
            valid_s2  <= valid_s1;
            hsync_s2  <= hsync_s1;
            vsync_s2  <= vsync_s1;
        end
    end

    // ─── Stage 3: Manhattan magnitude + threshold ─────────────────────────────
    logic [10:0] magnitude_s3;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            o_gradient <= '0;
            o_edge     <= 1'b0;
            o_valid    <= 1'b0;
            o_hsync    <= 1'b0;
            o_vsync    <= 1'b0;
        end else begin
            magnitude_s3 <= {1'b0, abs_gx_s2} + {1'b0, abs_gy_s2};  // max = 2040

            // Clamp to 8-bit: if > 255 → saturate at 255
            o_gradient <= (magnitude_s3 > 11'd255) ? 8'hFF : DATA_WIDTH'(magnitude_s3);
            o_edge     <= (magnitude_s3 > DATA_WIDTH'(THRESHOLD));
            o_valid    <= valid_s2;
            o_hsync    <= hsync_s2;
            o_vsync    <= vsync_s2;
        end
    end

endmodule : sobel_edge_detector
