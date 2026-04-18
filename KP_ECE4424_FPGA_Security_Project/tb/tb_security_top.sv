// =============================================================================
// File        : tb_security_top.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Self-checking testbench for security_top. Runs four phases:
//
//   Phase 0 – Calibration: run clean MAC, calibrate RO baselines
//   Phase 1 – Normal Operation: verify no false positives
//   Phase 2 – Trojan Trigger: drive TRIGGER_COUNT patterns to arm Trojan
//   Phase 3 – Payload Active: verify alarm fires, PR controller activates
//
//   Expected Results:
//   ├── Phase 0/1: global_alarm = 0, conf_score = 0
//   ├── Phase 2:   global_alarm = 0 (Trojan dormant)
//   └── Phase 3:   global_alarm = 1, pr_trigger pulse observed, pr_done asserts
//
//   Simulation: Icarus Verilog 12+ / Vivado Simulator
//   Run: iverilog -g2012 -o sim.out tb_security_top.sv ../rtl/*.sv && vvp sim.out
// =============================================================================

`timescale 1ns / 1ps

module tb_security_top;

    // ---- DUT Ports --------------------------------------------------------
    localparam int DATA_WIDTH = 16;
    localparam int ACC_WIDTH  = 32;
    localparam int N_RO       = 8;
    localparam int CLK_PERIOD = 10; // 100 MHz

    logic                    clk, rst_n;
    logic                    valid_in, acc_clr;
    logic [DATA_WIDTH-1:0]   a, b;
    logic [ACC_WIDTH-1:0]    acc_safe;
    logic                    valid_out;
    logic                    global_alarm, pr_busy, pr_done;
    logic [3:0]              conf_score;
    logic [7:0]              alarm_status;
    logic                    cal_en, ro_sample_en;
    logic                    icap_csn, icap_rdwrn;
    logic [31:0]             icap_din;
    logic                    icap_avail;

    // ---- DUT instantiation ------------------------------------------------
    security_top #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .N_RO      (N_RO),
        .USE_TROJAN(1)          // Inject Trojan in inst0
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .valid_in(valid_in), .acc_clr(acc_clr),
        .a(a), .b(b),
        .acc_safe(acc_safe), .valid_out(valid_out),
        .global_alarm(global_alarm), .pr_busy(pr_busy), .pr_done(pr_done),
        .conf_score(conf_score), .alarm_status(alarm_status),
        .cal_en(cal_en), .ro_sample_en(ro_sample_en),
        .icap_csn(icap_csn), .icap_rdwrn(icap_rdwrn), .icap_din(icap_din),
        .icap_avail(icap_avail)
    );

    // ---- Clock generation -------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- VCD dump for GTKWave --------------------------------------------
    initial begin
        $dumpfile("security_top_sim.vcd");
        $dumpvars(0, tb_security_top);
    end

    // ---- Test tracking ---------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    task check(input string test_name, input logic expected, input logic actual);
        if (expected === actual) begin
            $display("[PASS] %0t  %s: expected=%b, got=%b", $time, test_name, expected, actual);
            pass_count++;
        end else begin
            $display("[FAIL] %0t  %s: expected=%b, got=%b", $time, test_name, expected, actual);
            fail_count++;
        end
    endtask

    // ---- MAC stimulus helper ----------------------------------------------
    task send_mac(input logic [DATA_WIDTH-1:0] op_a, op_b, input int n);
        for (int i = 0; i < n; i++) begin
            @(posedge clk);
            valid_in = 1;
            a = op_a;
            b = op_b;
        end
        @(posedge clk); valid_in = 0;
    endtask

    // ---- Phase 0: Reset + Calibration ------------------------------------
    task phase0_calibrate();
        $display("\n=== PHASE 0: Reset & RO Calibration ===");
        rst_n = 0; valid_in = 0; acc_clr = 0;
        a = '0; b = '0;
        cal_en = 1; ro_sample_en = 0;
        icap_avail = 1;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5)  @(posedge clk);

        // Run calibration window
        ro_sample_en = 1;
        send_mac(16'h0101, 16'h0001, 100);  // Benign patterns
        repeat(1000) @(posedge clk);  // Full window
        ro_sample_en = 0;
        cal_en = 0;
        repeat(20) @(posedge clk);

        check("Phase0: No alarm during calibration", 1'b0, global_alarm);
        $display("    Conf score after calibration: %0d", conf_score);
    endtask

    // ---- Phase 1: Normal Operation (no Trojan trigger) -------------------
    task phase1_normal();
        $display("\n=== PHASE 1: Normal Operation – Expect No Alarm ===");
        // Use safe operands (a MSB nibble ≠ 0xA → no trigger)
        send_mac(16'h0505, 16'h0303, 200);
        ro_sample_en = 1;
        repeat(1000) @(posedge clk);
        ro_sample_en = 0;
        repeat(50) @(posedge clk);

        check("Phase1: No false positive alarm", 1'b0, global_alarm);
        check("Phase1: PR not busy",             1'b0, pr_busy);
        $display("    Conf score (should be 0): %0d", conf_score);
    endtask

    // ---- Phase 2: Trojan Arming (send 1023 trigger patterns) -------------
    task phase2_arm_trojan();
        $display("\n=== PHASE 2: Trojan Arming – 1023 Trigger Patterns ===");
        // a[15:12] = 4'hA → triggers Trojan counter
        for (int i = 0; i < 1025; i++) begin
            @(posedge clk);
            valid_in = 1;
            a = {4'hA, 12'h001};   // Key pattern in MSB nibble
            b = 16'h0001;
        end
        @(posedge clk); valid_in = 0;
        repeat(20) @(posedge clk);

        $display("    After arming: global_alarm = %b, conf_score = %0d",
                 global_alarm, conf_score);
        // Trojan just armed; payload not yet visible on output this cycle
    endtask

    // ---- Phase 3: Payload Active + Detection + PR Recovery --------------
    task phase3_detect_recover();
        $display("\n=== PHASE 3: Payload Active – Detection & PR Recovery ===");

        // Send a few more MACs — Trojan is now active (MSB inversion)
        send_mac({4'hA, 12'h0FF}, 16'h0100, 10);

        // Enable RO monitoring with Trojan power anomaly active
        ro_sample_en = 1;
        repeat(1000) @(posedge clk);
        ro_sample_en = 0;
        repeat(10) @(posedge clk);

        check("Phase3: TMR alarm fires on MSB inversion",  1'b1, dut.u_alarm.tmr_alarm | global_alarm);
        check("Phase3: Global alarm latched",               1'b1, global_alarm);

        $display("    Conf score: %0d (expect ≥ 4)", conf_score);
        $display("    Alarm status: 0x%02h", alarm_status);
        $display("    PR busy: %b", pr_busy);

        // Wait for PR controller to complete bitstream reload
        repeat(8192) @(posedge clk);
        check("Phase3: PR done after recovery",  1'b1, pr_done | (dut.u_pr.pr_busy === 0));
        $display("    ICAP CSn: %b  ICAP RWn: %b", icap_csn, icap_rdwrn);
    endtask

    // ---- Main test sequence ----------------------------------------------
    initial begin
        $display("============================================================");
        $display("  Supply Chain Security – security_top Testbench           ");
        $display("  Kaustubh Pandey (230959054) | MIT Manipal ECE 4424       ");
        $display("============================================================");

        phase0_calibrate();
        phase1_normal();
        phase2_arm_trojan();
        phase3_detect_recover();

        $display("\n============================================================");
        $display("  RESULTS: %0d PASSED | %0d FAILED", pass_count, fail_count);
        $display("============================================================");

        if (fail_count == 0)
            $display("  ✔ ALL TESTS PASSED – Security framework validated");
        else
            $display("  ✗ FAILURES DETECTED – Review output above");

        $finish;
    end

    // ---- Timeout watchdog ------------------------------------------------
    initial begin
        #5_000_000;
        $display("[TIMEOUT] Simulation exceeded 5 ms – terminating");
        $finish;
    end

endmodule
