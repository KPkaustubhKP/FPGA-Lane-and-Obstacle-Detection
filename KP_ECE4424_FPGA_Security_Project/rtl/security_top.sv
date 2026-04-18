// =============================================================================
// File        : security_top.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Top-level integration of the Multi-Modal Security Architecture.
//               Wraps a 3PIP MAC core with:
//                 Layer 1 – Functional:  TMR (3× MAC instances + voter)
//                 Layer 2 – Temporal:    CRC-16 bitstream monitor
//                 Layer 3 – Parametric:  8-RO sensor array
//                 Mitigation:            PR controller (ICAP/PCAP)
//
//               Block Diagram:
//               ┌─────────────────────────────────────────────────────────┐
//               │                    security_top                         │
//               │  ┌──────────┐ ┌──────────┐ ┌──────────┐               │
//               │  │ mac_inst0│ │ mac_inst1│ │ mac_inst2│  (3PIP × 3)   │
//               │  └────┬─────┘ └────┬─────┘ └────┬─────┘               │
//               │       └────────────┼─────────────┘                     │
//               │               ┌───▼────────┐                           │
//               │               │ tmr_wrapper│ ──── tmr_alarm            │
//               │               └───┬────────┘                           │
//               │                   │ voted_out                           │
//               │        ┌──────────▼─────────┐                          │
//               │        │  crc16_monitor      │ ──── crc_alarm           │
//               │        └────────────────────┘                          │
//               │  ┌─────────────────────────┐                           │
//               │  │  ro_sensor_array (×8)   │ ──── ro_alarm             │
//               │  └─────────────────────────┘                           │
//               │       ┌──────────────────┐                             │
//               │       │ alarm_aggregator  │ ──── global_alarm          │
//               │       └────────┬─────────┘      pr_trigger            │
//               │           ┌────▼────────┐                              │
//               │           │ pr_controller│ ──── ICAP interface         │
//               │           └─────────────┘                              │
//               └─────────────────────────────────────────────────────────┘
//
// Synthesis constraints:
//   - Each mac_inst placed in separate Pblock (see security_top.xdc)
//   - ro_sensor_array placed adjacent to mac region
//   - pr_controller connected to ICAP2_7SERIES primitive
// =============================================================================

`timescale 1ns / 1ps

module security_top #(
    parameter int DATA_WIDTH  = 16,
    parameter int ACC_WIDTH   = 32,
    parameter int N_RO        = 8,
    // Inject Trojan in inst0 for testing (set to 0 for production)
    parameter bit USE_TROJAN  = 1
) (
    input  logic                    clk,
    input  logic                    rst_n,
    // MAC data interface
    input  logic                    valid_in,
    input  logic                    acc_clr,
    input  logic [DATA_WIDTH-1:0]   a,
    input  logic [DATA_WIDTH-1:0]   b,
    // Safe output (majority-voted)
    output logic [ACC_WIDTH-1:0]    acc_safe,
    output logic                    valid_out,
    // Security status
    output logic                    global_alarm,
    output logic                    pr_busy,
    output logic                    pr_done,
    output logic [3:0]              conf_score,
    output logic [7:0]              alarm_status,
    // Calibration control
    input  logic                    cal_en,
    input  logic                    ro_sample_en,
    // ICAP interface (connect to ICAP2_7SERIES in top-level wrapper)
    output logic                    icap_csn,
    output logic                    icap_rdwrn,
    output logic [31:0]             icap_din,
    input  logic                    icap_avail
);

    // =========================================================================
    // LAYER 1: TRIPLE MODULAR REDUNDANCY (Functional)
    // =========================================================================

    // MAC instance outputs
    logic [ACC_WIDTH-1:0]   mac0_out, mac1_out, mac2_out;
    logic                   mac0_vld, mac1_vld, mac2_vld;
    logic                   mac0_ovf, mac1_ovf, mac2_ovf;

    // Trojan-infected inst0 (or clean, controlled by USE_TROJAN)
    generate
        if (USE_TROJAN) begin : trojan_inst
            mac_trojan #(.DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) u_mac0 (
                .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .acc_clr(acc_clr),
                .a(a), .b(b), .acc_out(mac0_out), .valid_out(mac0_vld), .overflow(mac0_ovf)
            );
        end else begin : clean_inst0
            mac_core #(.DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) u_mac0 (
                .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .acc_clr(acc_clr),
                .a(a), .b(b), .acc_out(mac0_out), .valid_out(mac0_vld), .overflow(mac0_ovf)
            );
        end
    endgenerate

    // Clean instances (1 and 2)
    mac_core #(.DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) u_mac1 (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .acc_clr(acc_clr),
        .a(a), .b(b), .acc_out(mac1_out), .valid_out(mac1_vld), .overflow(mac1_ovf)
    );

    mac_core #(.DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) u_mac2 (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .acc_clr(acc_clr),
        .a(a), .b(b), .acc_out(mac2_out), .valid_out(mac2_vld), .overflow(mac2_ovf)
    );

    // TMR voter
    logic              tmr_alarm;
    logic [2:0]        fault_flags;
    logic [1:0]        tmr_severity;

    tmr_wrapper #(.DATA_WIDTH(ACC_WIDTH)) u_tmr (
        .clk(clk), .rst_n(rst_n),
        .din_0(mac0_out), .din_1(mac1_out), .din_2(mac2_out),
        .dout_voted(acc_safe),
        .tmr_alarm(tmr_alarm),
        .fault_flags(fault_flags),
        .voter_disagreements(tmr_severity)
    );

    assign valid_out = mac1_vld;  // Use clean instance for timing

    // =========================================================================
    // LAYER 2: CRC-16 BITSTREAM MONITOR (Temporal)
    // =========================================================================

    logic        crc_alarm;
    logic [15:0] crc_live;
    logic        crc_check;

    // Feed upper 8 bits of voted output to CRC monitor
    crc16_monitor #(.CHECK_INTERVAL(256)) u_crc (
        .clk(clk), .rst_n(rst_n),
        .data_in(acc_safe[31:24]),
        .valid(valid_out),
        .golden_crc(16'hB4C8),   // Pre-computed golden value (clean MAC, 256 samples)
        .golden_load(1'b0),      // Set HIGH during trusted boot to latch golden
        .crc_live(crc_live),
        .crc_alarm(crc_alarm),
        .check_pulse(crc_check)
    );

    // =========================================================================
    // LAYER 3: RING OSCILLATOR ARRAY (Parametric)
    // =========================================================================

    logic [15:0]  ro_count   [N_RO];
    logic [15:0]  ro_baseline[N_RO];
    logic [N_RO-1:0] ro_anomaly;
    logic         ro_alarm;
    logic         cal_done;

    // Trojan drives power anomaly on inst0 region (for simulation)
    logic [N_RO-1:0] trojan_power_sim;
    assign trojan_power_sim = USE_TROJAN ? {{(N_RO-2){1'b0}}, 2'b11} : '0;

    ro_sensor_array #(.N_RO(N_RO)) u_ro (
        .clk(clk), .rst_n(rst_n),
        .cal_en(cal_en),
        .sample_en(ro_sample_en),
        .trojan_active_sim(trojan_power_sim),
        .ro_count(ro_count),
        .ro_baseline(ro_baseline),
        .ro_anomaly(ro_anomaly),
        .ro_alarm(ro_alarm),
        .cal_done(cal_done)
    );

    // =========================================================================
    // ALARM FUSION
    // =========================================================================

    logic pr_trigger_int;

    alarm_aggregator #(.N_RO(N_RO)) u_alarm (
        .clk(clk), .rst_n(rst_n),
        .tmr_alarm(tmr_alarm),
        .tmr_severity(tmr_severity),
        .crc_alarm(crc_alarm),
        .ro_anomaly(ro_anomaly),
        .ro_alarm(ro_alarm),
        .global_alarm(global_alarm),
        .pr_trigger(pr_trigger_int),
        .conf_score(conf_score),
        .alarm_source(),
        .alarm_status(alarm_status)
    );

    // =========================================================================
    // MITIGATION: PARTIAL RECONFIGURATION CONTROLLER
    // =========================================================================

    logic pr_error;

    pr_controller u_pr (
        .clk(clk), .rst_n(rst_n),
        .pr_trigger(pr_trigger_int),
        .pr_busy(pr_busy),
        .pr_done(pr_done),
        .pr_error(pr_error),
        .pr_state_debug(),
        .icap_csn(icap_csn),
        .icap_rdwrn(icap_rdwrn),
        .icap_din(icap_din),
        .icap_avail(icap_avail)
    );

endmodule
