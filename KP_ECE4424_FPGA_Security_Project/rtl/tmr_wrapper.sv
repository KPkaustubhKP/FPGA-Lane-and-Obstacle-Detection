// =============================================================================
// File        : tmr_wrapper.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Triple Modular Redundancy (TMR) wrapper. Instantiates three
//               copies of any IP module and performs bitwise majority voting.
//               Provides immediate fault masking AND an alarm signal when any
//               voter disagrees.
//
//               Architecture:
//               ┌──────┐
//               │ MAC_0 ├──────────────────────────┐
//               └──────┘                           │
//               ┌──────┐   ┌─────────────────┐    ▼
//               │ MAC_1 ├──►│  Majority Voter  ├──► DOUT (masked)
//               └──────┘   └────────┬────────┘
//               ┌──────┐            │
//               │ MAC_2 ├───────────┴──────────────► TMR_ALARM
//               └──────┘
//
//               For Artix-7 synthesis, instantiate the IP three times in the
//               parent module and connect din_0/1/2 to their outputs. Each
//               instance should be placed in a separate Pblock.
// =============================================================================

`timescale 1ns / 1ps

module tmr_wrapper #(
    parameter int DATA_WIDTH = 32
) (
    input  logic                    clk,
    input  logic                    rst_n,
    // Three redundant outputs from three IP instances
    input  logic [DATA_WIDTH-1:0]   din_0,
    input  logic [DATA_WIDTH-1:0]   din_1,
    input  logic [DATA_WIDTH-1:0]   din_2,
    // Majority-voted output
    output logic [DATA_WIDTH-1:0]   dout_voted,
    // Fault alarm: asserted on ANY mismatch between voters
    output logic                    tmr_alarm,
    // Per-module fault indicators (for diagnostic / isolation)
    output logic [2:0]              fault_flags,
    // Hamming distance between voters (0=all agree, 3=all disagree)
    output logic [1:0]              voter_disagreements
);

    // ---- Majority Vote (bitwise) -------------------------------------------
    // Out[i] = (A[i]&B[i]) | (B[i]&C[i]) | (A[i]&C[i])
    assign dout_voted = (din_0 & din_1) | (din_1 & din_2) | (din_0 & din_2);

    // ---- Fault Detection ---------------------------------------------------
    // flag[n]: module n disagrees with the majority result
    assign fault_flags[0] = (dout_voted !== din_0);
    assign fault_flags[1] = (dout_voted !== din_1);
    assign fault_flags[2] = (dout_voted !== din_2);

    // Top-level alarm: any voter disagrees
    assign tmr_alarm = |fault_flags;

    // ---- Disagreement Count (severity metric) ------------------------------
    // 0 = all agree, 1 = one disagrees (correctable), 2 = two+ disagree
    always_comb begin
        case (fault_flags)
            3'b000: voter_disagreements = 2'd0;  // All agree — healthy
            3'b001, 3'b010, 3'b100: voter_disagreements = 2'd1;  // 1 fault
            default: voter_disagreements = 2'd2; // ≥2 faults — critical
        endcase
    end

endmodule
