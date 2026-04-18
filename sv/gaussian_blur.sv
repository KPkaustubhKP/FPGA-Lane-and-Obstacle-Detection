// =============================================================================
// Module      : gaussian_blur.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : 3×3 Gaussian blur filter implemented with streaming line buffers.
//               Kernel (sum = 16, so normalise by >> 4):
//                   | 1  2  1 |
//                   | 2  4  2 |
//                   | 1  2  1 |
//               Architecture:
//                 • Two line-buffers (shift-register FIFOs, depth = IMG_WIDTH)
//                   hold rows N-1 and N-2 while row N streams in.
//                 • A 3×3 sliding window register collects 9 pixels.
//                 • Parallel MAC tree computes the weighted sum in one cycle.
//                 • Output is valid after the pipeline is filled
//                   (2 full rows + 2 extra pixels of start-up latency).
// Synthesizable: Yes — infers BRAM line buffers in Vivado (SRL/RAM style).
// Latency      : 2 × IMG_WIDTH + 2 clock cycles
// =============================================================================

`timescale 1ns / 1ps

module gaussian_blur #(
    parameter int unsigned DATA_WIDTH = 8,      // pixel bit width
    parameter int unsigned IMG_WIDTH  = 640,    // image columns
    parameter int unsigned IMG_HEIGHT = 480     // image rows
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── Upstream ────────────────────────────────────────────────────────────
    input  logic                    i_valid,
    input  logic [DATA_WIDTH-1:0]   i_pixel,
    input  logic                    i_hsync,
    input  logic                    i_vsync,

    // ── Downstream ──────────────────────────────────────────────────────────
    output logic                    o_valid,
    output logic [DATA_WIDTH-1:0]   o_pixel,
    output logic                    o_hsync,
    output logic                    o_vsync
);

    // ─── Gaussian 3×3 coefficients (integer, denominator = 16) ──────────────
    //   | 1  2  1 |
    //   | 2  4  2 |
    //   | 1  2  1 |

    // ─── Line-buffer type ─────────────────────────────────────────────────────
    // Each line buffer stores IMG_WIDTH pixels and acts as a FIFO.
    // Vivado maps these to BRAM when IMG_WIDTH is large.
    typedef logic [DATA_WIDTH-1:0] pixel_t;

    // ─── Line buffers (two rows) ──────────────────────────────────────────────
    pixel_t lb0 [0:IMG_WIDTH-1];   // oldest row (row n-2)
    pixel_t lb1 [0:IMG_WIDTH-1];   // middle row (row n-1)

    // Write pointer into the circular line buffers
    logic [$clog2(IMG_WIDTH)-1:0] wr_ptr;

    // ─── 3×3 sliding window (row2=newest, row0=oldest) ───────────────────────
    // Each row holds three consecutive pixels: [col-2], [col-1], [col]
    pixel_t win [0:2][0:2];   // win[row][col]

    // ─── Sync pipeline ────────────────────────────────────────────────────────
    logic [1:0] valid_pipe;
    logic [1:0] hsync_pipe;
    logic [1:0] vsync_pipe;

    // ─── Write-side: shift new pixel into both line buffers ───────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (i_valid) begin
            // Shift: lb0 ← lb1 ← current pixel (oldest drops out)
            lb0[wr_ptr] <= lb1[wr_ptr];
            lb1[wr_ptr] <= i_pixel;
            wr_ptr <= (wr_ptr == IMG_WIDTH - 1) ? '0 : wr_ptr + 1'b1;
        end
    end

    // ─── Read-side: update 3×3 window ─────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    win[r][c] <= '0;
        end else if (i_valid) begin
            // Row 0 — oldest (lb0, already written with lb1's previous value)
            win[0][0] <= win[0][1];
            win[0][1] <= win[0][2];
            win[0][2] <= lb0[wr_ptr];   // read BEFORE wr_ptr advances

            // Row 1 — middle (lb1)
            win[1][0] <= win[1][1];
            win[1][1] <= win[1][2];
            win[1][2] <= lb1[wr_ptr];

            // Row 2 — current (incoming pixel)
            win[2][0] <= win[2][1];
            win[2][1] <= win[2][2];
            win[2][2] <= i_pixel;
        end
    end

    // ─── MAC: compute weighted sum (18-bit max: 255×16 = 4080 → fits in 12 bits) ─
    // Kernel weights:
    //   row0: 1  2  1
    //   row1: 2  4  2
    //   row2: 1  2  1
    // Sum = 16 → normalise by >> 4

    logic [11:0] weighted_sum;
    logic        mac_valid;
    logic        mac_hsync, mac_vsync;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            weighted_sum <= '0;
            mac_valid    <= 1'b0;
            mac_hsync    <= 1'b0;
            mac_vsync    <= 1'b0;
        end else begin
            // Parallel MAC (one clock latency)
            weighted_sum <=   (12'(win[0][0])           // ×1
                             + {11'(win[0][1]), 1'b0}   // ×2  (left-shift 1)
                             +  12'(win[0][2])           // ×1
                             + {11'(win[1][0]), 1'b0}   // ×2
                             + {10'(win[1][1]), 2'b0}   // ×4  (left-shift 2)
                             + {11'(win[1][2]), 1'b0}   // ×2
                             +  12'(win[2][0])           // ×1
                             + {11'(win[2][1]), 1'b0}   // ×2
                             +  12'(win[2][2]));         // ×1
            mac_valid <= i_valid;
            mac_hsync <= i_hsync;
            mac_vsync <= i_vsync;
        end
    end

    // ─── Output: normalise by dividing by 16 (>> 4) ──────────────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            o_pixel <= '0;
            o_valid <= 1'b0;
            o_hsync <= 1'b0;
            o_vsync <= 1'b0;
        end else begin
            o_pixel <= DATA_WIDTH'(weighted_sum >> 4);
            o_valid <= mac_valid;
            o_hsync <= mac_hsync;
            o_vsync <= mac_vsync;
        end
    end

endmodule : gaussian_blur
