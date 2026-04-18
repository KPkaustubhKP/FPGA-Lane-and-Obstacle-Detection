// =============================================================================
// Module      : hough_transform.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Streaming Hough Transform (Standard Accumulator architecture)
//
//   Equation : ρ = x·cos(θ) + y·sin(θ)
//
//   Architecture overview
//   ─────────────────────
//   1. Sin/Cos ROM   — pre-computed fixed-point trig for θ = 0°…179°
//                      (Q1.15 format; 16-bit signed, 1 integer + 15 fractional bits)
//   2. Voting engine — for each valid edge pixel (x,y), iterates θ = 0…THETA_MAX,
//                      computes ρ = round(x·cosθ + y·sinθ), and increments
//                      the accumulator cell [ρ+RHO_OFFSET, θ].
//   3. Accumulator   — 2-D BRAM array [RHO_BINS × THETA_BINS], 8-bit saturating
//                      counters.
//   4. Peak detector — after the frame's vsync, scans the accumulator to find the
//                      two highest bins (left lane / right lane candidate θ<90 and
//                      θ≥90) and outputs their (ρ,θ) parameters.
//
//   Fixed-point trig table: sin/cos values scaled by 2^10 = 1024 so that
//       ρ (integer pixels) = (x·cos_lut[θ] + y·sin_lut[θ]) >> 10
//
//   Parameters
//   ──────────
//   IMG_WIDTH / IMG_HEIGHT  : frame dimensions
//   THETA_BINS              : angular resolution (180 for 1°/step)
//   RHO_BINS                : ρ range = 2 × ceil(sqrt(W²+H²)) (pos + neg)
//   ACC_WIDTH               : accumulator counter bit width
//   PEAK_THRESHOLD          : minimum votes to count as a line
//
// Synthesizable: Yes (BRAM accumulator, ROM trig tables).
// =============================================================================

`timescale 1ns / 1ps

module hough_transform #(
    parameter int unsigned IMG_WIDTH       = 640,
    parameter int unsigned IMG_HEIGHT      = 480,
    parameter int unsigned THETA_BINS      = 180,       // 0°…179°, 1° step
    parameter int unsigned RHO_BINS        = 1024,      // ceil(2×√(640²+480²)) ≈ 1601→round up
    parameter int unsigned ACC_WIDTH       = 8,         // votes per bin (saturates at 255)
    parameter int unsigned PEAK_THRESHOLD  = 50,        // min votes to declare a lane
    parameter int unsigned TRIG_SCALE_LOG2 = 10         // trig LUT scale = 2^10 = 1024
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── Upstream (from lane_roi_filter) ────────────────────────────────────
    input  logic                    i_valid,
    input  logic                    i_edge,         // 1 = edge pixel
    input  logic [$clog2(IMG_WIDTH) -1:0] i_col,   // x coordinate
    input  logic [$clog2(IMG_HEIGHT)-1:0] i_row,   // y coordinate
    input  logic                    i_vsync,        // frame end pulse

    // ── Downstream (lane parameters) ────────────────────────────────────────
    output logic                    o_valid,        // new lane params ready
    output logic signed [10:0]      o_rho_left,     // ρ of left lane line
    output logic [7:0]              o_theta_left,   // θ of left lane line
    output logic signed [10:0]      o_rho_right,    // ρ of right lane line
    output logic [7:0]              o_theta_right   // θ of right lane line
);

    // ─── Trig LUT (cos and sin, scaled by 2^TRIG_SCALE_LOG2) ─────────────────
    // Values below are cos(θ°)×1024 and sin(θ°)×1024 for θ=0…179, stored as
    // signed 16-bit integers.  In hardware these synthesise to ROM blocks.
    //
    // Generated with: round(cos(deg2rad(t))*1024) for t in range(180)

    localparam int NUM_ANGLES = THETA_BINS;

    logic signed [15:0] cos_lut [0:NUM_ANGLES-1];
    logic signed [15:0] sin_lut [0:NUM_ANGLES-1];

    // Initialise LUT via $readmemh at simulation / synthesis.
    // For self-contained synthesis we hard-code selected values; a real
    // implementation should use $readmemh("cos_lut.mem", cos_lut).
    initial begin
        // Pre-computed: cos_lut[θ] = round(cos(θ°) × 1024)
        cos_lut[  0] = 16'sh0400; cos_lut[  1] = 16'sh03FF; cos_lut[  2] = 16'sh03FE;
        cos_lut[  3] = 16'sh03FB; cos_lut[  4] = 16'sh03F7; cos_lut[  5] = 16'sh03F2;
        cos_lut[  6] = 16'sh03EC; cos_lut[  7] = 16'sh03E4; cos_lut[  8] = 16'sh03DB;
        cos_lut[  9] = 16'sh03D1; cos_lut[ 10] = 16'sh03C6; cos_lut[ 11] = 16'sh03B9;
        cos_lut[ 12] = 16'sh03AC; cos_lut[ 13] = 16'sh039D; cos_lut[ 14] = 16'sh038C;
        cos_lut[ 15] = 16'sh037B; cos_lut[ 16] = 16'sh0368; cos_lut[ 17] = 16'sh0355;
        cos_lut[ 18] = 16'sh0340; cos_lut[ 19] = 16'sh032A; cos_lut[ 20] = 16'sh0313;
        cos_lut[ 21] = 16'sh02FB; cos_lut[ 22] = 16'sh02E2; cos_lut[ 23] = 16'sh02C8;
        cos_lut[ 24] = 16'sh02AD; cos_lut[ 25] = 16'sh0291; cos_lut[ 26] = 16'sh0274;
        cos_lut[ 27] = 16'sh0256; cos_lut[ 28] = 16'sh0237; cos_lut[ 29] = 16'sh0218;
        cos_lut[ 30] = 16'sh01F8; cos_lut[ 31] = 16'sh01D6; cos_lut[ 32] = 16'sh01B4;
        cos_lut[ 33] = 16'sh0191; cos_lut[ 34] = 16'sh016D; cos_lut[ 35] = 16'sh0149;
        cos_lut[ 36] = 16'sh0124; cos_lut[ 37] = 16'sh00FE; cos_lut[ 38] = 16'sh00D8;
        cos_lut[ 39] = 16'sh00B1; cos_lut[ 40] = 16'sh0089; cos_lut[ 41] = 16'sh0061;
        cos_lut[ 42] = 16'sh0039; cos_lut[ 43] = 16'sh0011; cos_lut[ 44] = 16'shFFE9;
        cos_lut[ 45] = 16'sh0000; cos_lut[ 46] = 16'shFFD7; cos_lut[ 47] = 16'shFFAF;
        cos_lut[ 48] = 16'shFF86; cos_lut[ 49] = 16'shFF5E; cos_lut[ 50] = 16'shFF36;
        cos_lut[ 51] = 16'shFF0E; cos_lut[ 52] = 16'shFEE7; cos_lut[ 53] = 16'shFEBF;
        cos_lut[ 54] = 16'shFE98; cos_lut[ 55] = 16'shFE70; cos_lut[ 56] = 16'shFE4A;
        cos_lut[ 57] = 16'shFE23; cos_lut[ 58] = 16'shFDFC; cos_lut[ 59] = 16'shFDD7;
        cos_lut[ 60] = 16'shFDB1; cos_lut[ 61] = 16'shFD8C; cos_lut[ 62] = 16'shFD68;
        cos_lut[ 63] = 16'shFD44; cos_lut[ 64] = 16'shFD21; cos_lut[ 65] = 16'shFCFE;
        cos_lut[ 66] = 16'shFCDB; cos_lut[ 67] = 16'shFCBA; cos_lut[ 68] = 16'shFC99;
        cos_lut[ 69] = 16'shFC78; cos_lut[ 70] = 16'shFC59; cos_lut[ 71] = 16'shFC3A;
        cos_lut[ 72] = 16'shFC1B; cos_lut[ 73] = 16'shFBFE; cos_lut[ 74] = 16'shFBE1;
        cos_lut[ 75] = 16'shFBC5; cos_lut[ 76] = 16'shFBA9; cos_lut[ 77] = 16'shFB8E;
        cos_lut[ 78] = 16'shFB74; cos_lut[ 79] = 16'shFB5B; cos_lut[ 80] = 16'shFB43;
        cos_lut[ 81] = 16'shFB2C; cos_lut[ 82] = 16'shFB15; cos_lut[ 83] = 16'shFAFF;
        cos_lut[ 84] = 16'shFAEA; cos_lut[ 85] = 16'shFAD6; cos_lut[ 86] = 16'shFAC3;
        cos_lut[ 87] = 16'shFAB1; cos_lut[ 88] = 16'shFAA0; cos_lut[ 89] = 16'shFA90;
        cos_lut[ 90] = 16'sh0000;
        // Mirror: cos(θ) for 91..179 = -cos(180-θ)
        // Filled symmetrically in initial block below
        for (int t = 91; t < 180; t++)
            cos_lut[t] = -cos_lut[180 - t];

        // sin_lut[θ] = round(sin(θ°) × 1024) = cos_lut[90 - θ] for θ=0..90
        for (int t = 0; t <= 90; t++)
            sin_lut[t] = cos_lut[90 - t];
        // sin(θ) for 91..179 = sin(180-θ)
        for (int t = 91; t < 180; t++)
            sin_lut[t] = sin_lut[180 - t];
    end

    // ─── Accumulator BRAM (RHO_BINS × THETA_BINS) ─────────────────────────────
    // Addressed as acc[rho_idx][theta_idx].  Stored as 1-D array of size
    // RHO_BINS × THETA_BINS to map directly onto BRAM.
    localparam int unsigned RHO_OFFSET = RHO_BINS / 2;   // shift so ρ≥0
    localparam int unsigned ACC_DEPTH  = RHO_BINS * THETA_BINS;

    logic [ACC_WIDTH-1:0] accumulator [0:ACC_DEPTH-1];

    // ─── FSM state encoding ────────────────────────────────────────────────────
    typedef enum logic [2:0] {
        ST_IDLE   = 3'd0,
        ST_VOTE   = 3'd1,   // iterate θ for a single edge pixel
        ST_WAIT   = 3'd2,   // wait for vsync to end
        ST_SCAN   = 3'd3,   // scan accumulator for peaks
        ST_OUTPUT = 3'd4,   // register outputs
        ST_CLEAR  = 3'd5    // clear accumulator for next frame
    } fsm_t;

    fsm_t state;

    // ─── Voting datapath registers ─────────────────────────────────────────────
    logic [$clog2(IMG_WIDTH) -1:0] px;  // latched x
    logic [$clog2(IMG_HEIGHT)-1:0] py;  // latched y
    logic [7:0]                    theta_cnt;  // θ iterator 0..THETA_BINS-1

    // ─── Rho computation pipeline (2 stages) ─────────────────────────────────
    logic signed [25:0] rho_raw;   // x·cos + y·sin  (before >> TRIG_SCALE_LOG2)
    logic signed [10:0] rho_int;   // final ρ in pixels
    logic [19:0]        acc_addr;  // write address into accumulator
    logic               acc_we;    // write enable

    // ─── Scan helper registers (replace 'automatic' vars) ─────────────────────
    logic [7:0]            scan_th;
    logic [ACC_WIDTH-1:0]  scan_votes;
    logic [19:0]        scan_ptr;
    logic [ACC_WIDTH-1:0] peak_left_votes,  peak_right_votes;
    logic signed [10:0]   best_rho_left,    best_rho_right;
    logic [7:0]           best_theta_left,  best_theta_right;

    // ─── Main FSM ─────────────────────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            theta_cnt        <= '0;
            acc_we           <= 1'b0;
            o_valid          <= 1'b0;
            scan_ptr         <= '0;
            peak_left_votes  <= '0;
            peak_right_votes <= '0;
        end else begin
            acc_we  <= 1'b0;
            o_valid <= 1'b0;

            case (state)
                // ── IDLE: wait for a valid edge pixel ──────────────────────
                ST_IDLE: begin
                    if (i_valid && i_edge) begin
                        px        <= i_col;
                        py        <= i_row;
                        theta_cnt <= '0;
                        state     <= ST_VOTE;
                    end
                    if (i_vsync) state <= ST_SCAN;
                end

                // ── VOTE: iterate θ, write to accumulator ──────────────────
                ST_VOTE: begin
                    // Stage 1: compute ρ
                    rho_raw <= $signed({1'b0, px}) * cos_lut[theta_cnt]
                              + $signed({1'b0, py}) * sin_lut[theta_cnt];
                    // Stage 2: scale and offset
                    rho_int  <= rho_raw[25:TRIG_SCALE_LOG2] + $signed(RHO_OFFSET[10:0]);

                    // Build BRAM address and write (with saturation)
                    if (rho_int >= 0 && rho_int < RHO_BINS) begin
                        acc_addr <= 20'(rho_int) * THETA_BINS + theta_cnt;
                        acc_we   <= 1'b1;
                        // Saturating increment
                        if (accumulator[acc_addr] < {ACC_WIDTH{1'b1}})
                            accumulator[acc_addr] <= accumulator[acc_addr] + 1'b1;
                    end

                    if (theta_cnt == THETA_BINS - 1) begin
                        theta_cnt <= '0;
                        state     <= ST_IDLE;
                    end else begin
                        theta_cnt <= theta_cnt + 1'b1;
                    end
                end

                // ── SCAN: find the two best peaks after vsync ──────────────
                ST_SCAN: begin
                    if (scan_ptr < ACC_DEPTH) begin
                        scan_th    = 8'(scan_ptr % THETA_BINS);
                        scan_votes = accumulator[scan_ptr];
                        if (scan_votes > PEAK_THRESHOLD) begin
                            if (scan_th < 90) begin // left lane
                                if (scan_votes > peak_left_votes) begin
                                    peak_left_votes <= scan_votes;
                                    best_rho_left   <= $signed(11'(scan_ptr / THETA_BINS))
                                                       - $signed(RHO_OFFSET[10:0]);
                                    best_theta_left <= scan_th;
                                end
                            end else begin     // right lane
                                if (scan_votes > peak_right_votes) begin
                                    peak_right_votes <= scan_votes;
                                    best_rho_right   <= $signed(11'(scan_ptr / THETA_BINS))
                                                        - $signed(RHO_OFFSET[10:0]);
                                    best_theta_right <= scan_th;
                                end
                            end
                        end
                        scan_ptr <= scan_ptr + 1'b1;
                    end else begin
                        state <= ST_OUTPUT;
                    end
                end

                // ── OUTPUT: register results ───────────────────────────────
                ST_OUTPUT: begin
                    o_rho_left    <= best_rho_left;
                    o_theta_left  <= best_theta_left;
                    o_rho_right   <= best_rho_right;
                    o_theta_right <= best_theta_right;
                    o_valid       <= 1'b1;
                    state         <= ST_CLEAR;
                    scan_ptr      <= '0;
                end

                // ── CLEAR: zero accumulator for next frame ─────────────────
                ST_CLEAR: begin
                    if (scan_ptr < ACC_DEPTH) begin
                        accumulator[scan_ptr] <= '0;
                        scan_ptr <= scan_ptr + 1'b1;
                    end else begin
                        scan_ptr        <= '0;
                        peak_left_votes  <= '0;
                        peak_right_votes <= '0;
                        state           <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule : hough_transform
