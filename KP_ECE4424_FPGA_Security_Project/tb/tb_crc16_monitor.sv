// =============================================================================
// File        : tb_crc16_monitor.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Self-checking unit test for crc16_monitor.
//               Tests: (1) clean stream matches golden → no alarm
//                      (2) single byte corruption → alarm after check interval
//                      (3) reset clears CRC and alarm
// =============================================================================

`timescale 1ns / 1ps

module tb_crc16_monitor;

    localparam int CLK_P    = 10;
    localparam int INTERVAL = 16;  // Smaller interval for faster simulation

    logic        clk, rst_n;
    logic [7:0]  data_in;
    logic        valid, golden_load;
    logic [15:0] golden_crc;
    logic [15:0] crc_live;
    logic        crc_alarm, check_pulse;

    crc16_monitor #(.CHECK_INTERVAL(INTERVAL)) dut (
        .clk(clk), .rst_n(rst_n),
        .data_in(data_in), .valid(valid),
        .golden_crc(golden_crc), .golden_load(golden_load),
        .crc_live(crc_live),
        .crc_alarm(crc_alarm),
        .check_pulse(check_pulse)
    );

    initial clk = 0;
    always #(CLK_P/2) clk = ~clk;

    int pass_cnt = 0, fail_cnt = 0;
    task chk(input string name, input logic exp, input logic got);
        if (exp === got) begin $display("[PASS] %s", name); pass_cnt++; end
        else             begin $display("[FAIL] %s: exp=%b got=%b", name, exp, got); fail_cnt++; end
    endtask

    // Software CRC-16/CCITT reference for test pattern
    // Pattern: bytes 0x00..0x0F repeated
    // Pre-computed golden: 0x3B37 (CRC-16/CCITT of 16 bytes 0x00..0x0F)
    localparam logic [15:0] GOLDEN_CLEAN = 16'h3B37;

    task send_stream(input logic [7:0] base, input int corrupt_idx = -1);
        for (int i = 0; i < INTERVAL; i++) begin
            @(posedge clk);
            valid = 1;
            data_in = (i == corrupt_idx) ? (base + 8'hFF) : (base + 8'(i));
        end
        @(posedge clk); valid = 0;
        // Wait for check_pulse
        repeat(4) @(posedge clk);
    endtask

    initial begin
        $display("=== tb_crc16_monitor ===");
        rst_n = 0; valid = 0; data_in = 0;
        golden_crc = 0; golden_load = 0;
        repeat(4) @(posedge clk); rst_n = 1; repeat(2) @(posedge clk);

        // ---- Test 1: Load golden, send clean stream → no alarm
        $display("\nTest 1: Clean stream matches golden");
        golden_crc  = GOLDEN_CLEAN;
        golden_load = 1;
        @(posedge clk); golden_load = 0;
        send_stream(8'h00, -1);  // No corruption
        $display("    CRC live: 0x%04h  Golden: 0x%04h", crc_live, GOLDEN_CLEAN);
        chk("T1: No alarm on clean stream", 1'b0, crc_alarm);

        // ---- Test 2: Corrupt one byte → alarm
        $display("\nTest 2: Single byte corruption → alarm");
        rst_n = 0; @(posedge clk); rst_n = 1;  // Reset CRC state
        golden_crc  = GOLDEN_CLEAN;
        golden_load = 1; @(posedge clk); golden_load = 0;
        send_stream(8'h00, 7);  // Corrupt byte 7
        $display("    CRC live: 0x%04h (expect ≠ 0x%04h)", crc_live, GOLDEN_CLEAN);
        chk("T2: Alarm fires on corrupted stream", 1'b1, crc_alarm);

        // ---- Test 3: Reset clears alarm
        $display("\nTest 3: Reset clears CRC alarm");
        rst_n = 0; repeat(4) @(posedge clk); rst_n = 1;
        repeat(2) @(posedge clk);
        chk("T3: CRC alarm cleared after reset", 1'b0, crc_alarm);
        chk("T3: CRC reset to 0xFFFF",           1'b1, crc_live == 16'hFFFF);

        $display("\nCRC Monitor Results: %0d PASS, %0d FAIL", pass_cnt, fail_cnt);
        $finish;
    end

endmodule
