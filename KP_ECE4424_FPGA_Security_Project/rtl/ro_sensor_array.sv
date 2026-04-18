// =============================================================================
// File        : ro_sensor_array.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : 8-element Ring Oscillator (RO) sensor array for parametric
//               side-channel monitoring. Detects localized Vdd drops and
//               switching activity anomalies indicative of a triggered Trojan.
//
//               Architecture:
//               Each RO is a free-running oscillator whose frequency is
//               sensitive to local voltage, temperature, and switching load.
//               A triggered Trojan increases dynamic power → Vdd droop →
//               RO slows down → count deviates from calibrated baseline.
//
//               Two-phase operation:
//               1. CALIBRATION (cal_en=1): Record baseline count over a fixed
//                  window with known-clean input patterns.
//               2. MONITORING (cal_en=0): Compare live count against baseline.
//                  Deviation > ALARM_DELTA raises ro_anomaly for that RO.
//
//               For synthesis on Artix-7:
//               - Replace ro_sim_counter with actual LUT/BUFG ring chains
//               - Mark each RO with (* DONT_TOUCH = "TRUE" *)
//               - Assign each to a separate Pblock site near the 3PIP core
//               - Use FDRE primitives as frequency counters
//
//               NOTE: Simulation model uses parameterized frequency offsets
//               to mimic a triggered Trojan's Vdd droop effect.
// =============================================================================

`timescale 1ns / 1ps

module ro_sensor_array #(
    parameter int N_RO          = 8,    // Number of ring oscillators
    parameter int COUNT_WIDTH   = 16,   // Counter bit width
    parameter int WINDOW_CYCLES = 1000, // Measurement window (clk cycles)
    parameter int ALARM_DELTA   = 40,   // Count deviation threshold
    // Synthesis: target RO frequency ~200 MHz on Artix-7 (5-inverter chain)
    // Simulation: inject RO frequency offsets for Trojan modeling
    parameter int BASE_FREQ     = 200   // Nominal RO count per window (scaled)
) (
    input  logic                            clk,
    input  logic                            rst_n,
    input  logic                            cal_en,       // 1 = calibration mode
    input  logic                            sample_en,    // Enable counting window
    // Trojan injection (simulation only — models Vdd droop from Trojan activity)
    input  logic [N_RO-1:0]                trojan_active_sim,
    // Outputs
    output logic [COUNT_WIDTH-1:0]          ro_count   [N_RO],
    output logic [COUNT_WIDTH-1:0]          ro_baseline[N_RO],
    output logic [N_RO-1:0]                ro_anomaly,
    output logic                            ro_alarm,
    // Calibration done flag
    output logic                            cal_done
);

    // ---- Simulation RO model ------------------------------------------------
    // Each RO oscillates at BASE_FREQ counts per WINDOW_CYCLES.
    // When trojan_active_sim[i]=1, the count drops by ALARM_DELTA + margin.

    logic [$clog2(WINDOW_CYCLES)-1:0] window_cnt;
    logic                             window_done;
    logic                             window_running;

    // RO simulation counters (replace with LUT ring in synthesis)
    logic [COUNT_WIDTH-1:0] ro_sim_reg [N_RO];
    logic [7:0]             ro_phase   [N_RO];  // Phase accumulator for sub-clk modeling

    // Baseline registers (latched during calibration)
    logic [COUNT_WIDTH-1:0] baseline_reg [N_RO];
    logic                   cal_done_reg;

    // ---- Measurement window timer -------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_cnt     <= '0;
            window_done    <= 1'b0;
            window_running <= 1'b0;
        end else begin
            window_done <= 1'b0;
            if (sample_en && !window_running) begin
                window_cnt     <= '0;
                window_running <= 1'b1;
                // Reset all RO counters at window start
            end
            if (window_running) begin
                if (window_cnt == (WINDOW_CYCLES - 1)) begin
                    window_cnt     <= '0;
                    window_running <= 1'b0;
                    window_done    <= 1'b1;
                end else begin
                    window_cnt <= window_cnt + 1;
                end
            end
        end
    end

    // ---- Simulated RO counting ----------------------------------------------
    // Model: each RO increments at nominal rate; Trojan injects droop
    genvar g;
    generate
        for (g = 0; g < N_RO; g++) begin : ro_gen
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    ro_sim_reg[g] <= '0;
                    ro_phase[g]   <= 8'(g * 13 + 7);  // Stagger phases
                end else begin
                    if (window_running) begin
                        // Phase accumulator: increment sub-clk counter
                        // Nominal: adds ~BASE_FREQ/WINDOW_CYCLES per clk
                        // Trojan:  reduces rate by ~5% (Vdd droop model)
                        ro_phase[g] <= ro_phase[g] + 8'(trojan_active_sim[g] ? 
                                       (BASE_FREQ * 10 / WINDOW_CYCLES - 2) :
                                       (BASE_FREQ * 10 / WINDOW_CYCLES));
                        // Every 10 phase units → increment counter
                        if (ro_phase[g][3]) begin
                            ro_sim_reg[g] <= ro_sim_reg[g] + 1;
                        end
                    end else if (!sample_en) begin
                        ro_sim_reg[g] <= '0;
                    end
                end
            end
            assign ro_count[g] = ro_sim_reg[g];
        end
    endgenerate

    // ---- Baseline calibration -----------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N_RO; i++)
                baseline_reg[i] <= '0;
            cal_done_reg <= 1'b0;
        end else begin
            if (cal_en && window_done) begin
                for (int i = 0; i < N_RO; i++)
                    baseline_reg[i] <= ro_sim_reg[i];
                cal_done_reg <= 1'b1;
            end
            if (!cal_en && sample_en)
                cal_done_reg <= cal_done_reg;  // Hold after calibration
        end
    end

    // ---- Anomaly detection --------------------------------------------------
    // Alarm if |live_count - baseline| > ALARM_DELTA
    always_comb begin
        for (int i = 0; i < N_RO; i++) begin
            if (cal_done_reg && window_done) begin
                automatic int delta;
                delta = int'(ro_sim_reg[i]) - int'(baseline_reg[i]);
                if (delta < 0) delta = -delta;
                ro_anomaly[i] = (delta > ALARM_DELTA);
            end else begin
                ro_anomaly[i] = 1'b0;
            end
        end
    end

    // OR of all anomalies
    assign ro_alarm   = |ro_anomaly;
    assign cal_done   = cal_done_reg;

    // Expose baseline array
    generate
        for (g = 0; g < N_RO; g++) begin : bl_assign
            assign ro_baseline[g] = baseline_reg[g];
        end
    endgenerate

endmodule
