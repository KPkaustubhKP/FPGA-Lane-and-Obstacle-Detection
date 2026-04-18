// =============================================================================
// File        : alarm_aggregator.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Multi-modal alarm fusion engine. Combines outputs from all
//               three detection layers (TMR, CRC-16, RO array) into a single
//               confidence-weighted alarm score.
//
//               Confidence scoring:
//               ┌─────────────────┬──────────┬──────────────────────────────┐
//               │ Layer           │  Weight  │  Rationale                   │
//               ├─────────────────┼──────────┼──────────────────────────────┤
//               │ TMR Voter       │   3/6    │ High precision, exact match  │
//               │ CRC-16          │   2/6    │ Temporal drift detection      │
//               │ RO Array (≥2)   │   1/6    │ Noisy parametric channel     │
//               └─────────────────┴──────────┴──────────────────────────────┘
//
//               The system raises GLOBAL_ALARM when:
//               - TMR alarm fires (immediate) OR
//               - Confidence score ≥ CONF_THRESHOLD (weighted, multi-modal)
//
//               Fault Suppression: A debounce counter prevents false alarms
//               from transient Process Variation spikes (PV noise).
//
//               PR Trigger: Once alarm is confirmed, pr_trigger pulses for
//               one clock cycle to initiate Dynamic Partial Reconfiguration
//               via the PCAP controller.
// =============================================================================

`timescale 1ns / 1ps

module alarm_aggregator #(
    parameter int N_RO            = 8,
    parameter int DEBOUNCE_CYCLES = 4,   // Require N consecutive alarm cycles
    parameter int CONF_THRESHOLD  = 4    // Min confidence score to trigger (max=6)
) (
    input  logic        clk,
    input  logic        rst_n,
    // Layer inputs
    input  logic        tmr_alarm,
    input  logic [1:0]  tmr_severity,     // 0=OK, 1=1-fault, 2=critical
    input  logic        crc_alarm,
    input  logic [N_RO-1:0] ro_anomaly,
    input  logic        ro_alarm,
    // Outputs
    output logic        global_alarm,     // Final fused alarm
    output logic        pr_trigger,       // Pulse → start Partial Reconfiguration
    output logic [3:0]  conf_score,       // Live confidence score (0–6)
    output logic [1:0]  alarm_source,     // 0=none, 1=TMR, 2=CRC, 3=multi-modal
    output logic [7:0]  alarm_status      // Diagnostic register
);

    // ---- Confidence score computation --------------------------------------
    logic [3:0] score;
    logic [2:0] ro_count_active; // Number of ROs showing anomaly

    always_comb begin
        score = 4'd0;
        // Count active RO anomalies
        ro_count_active = '0;
        for (int i = 0; i < N_RO; i++)
            if (ro_anomaly[i]) ro_count_active++;

        // TMR contribution: weight 3 (strong functional evidence)
        if (tmr_alarm) begin
            score += (tmr_severity == 2'd2) ? 4'd3 : 4'd2;
        end
        // CRC contribution: weight 2 (temporal integrity evidence)
        if (crc_alarm) score += 4'd2;
        // RO contribution: weight 1 per 2 anomalous sensors (max 1)
        if (ro_count_active >= 2) score += 4'd1;
    end

    assign conf_score = score;

    // ---- Debounce counter --------------------------------------------------
    // Prevent false triggers from one-cycle PV spikes
    logic [2:0] debounce_cnt;
    logic       alarm_raw;

    // Raw alarm: immediate on TMR, or score-based for others
    assign alarm_raw = tmr_alarm || (score >= CONF_THRESHOLD[3:0]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_cnt <= '0;
            global_alarm <= 1'b0;
            pr_trigger   <= 1'b0;
        end else begin
            pr_trigger <= 1'b0;  // Default: no trigger

            if (alarm_raw) begin
                if (debounce_cnt < DEBOUNCE_CYCLES) begin
                    debounce_cnt <= debounce_cnt + 1;
                end else if (!global_alarm) begin
                    global_alarm <= 1'b1;
                    pr_trigger   <= 1'b1;  // One-shot pulse to PR controller
                end
            end else begin
                debounce_cnt <= '0;
                // NOTE: global_alarm is STICKY — cleared only by reset or PR ack
            end
        end
    end

    // ---- Alarm source encoding ---------------------------------------------
    always_comb begin
        if (!global_alarm)        alarm_source = 2'd0;
        else if (tmr_alarm)       alarm_source = 2'd1;  // TMR only
        else if (crc_alarm)       alarm_source = 2'd2;  // CRC only
        else                      alarm_source = 2'd3;  // Multi-modal (RO + CRC)
    end

    // ---- Diagnostic register (for chipscope / debug) -----------------------
    assign alarm_status = {ro_alarm, crc_alarm, tmr_alarm, score[3:0], 1'b0};

endmodule
