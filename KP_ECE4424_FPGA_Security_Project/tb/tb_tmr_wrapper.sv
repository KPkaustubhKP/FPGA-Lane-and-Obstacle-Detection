// =============================================================================
// File        : tb_tmr_wrapper.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Self-checking unit test for tmr_wrapper.
//               Tests: (1) all-agree → no alarm
//                      (2) single bit-flip in one voter → alarm + corrected output
//                      (3) two voters corrupted → alarm + severity = critical
// =============================================================================

`timescale 1ns / 1ps

module tb_tmr_wrapper;

    localparam int W = 32;
    localparam int CLK_P = 10;

    logic clk, rst_n;
    logic [W-1:0] d0, d1, d2, dout;
    logic         tmr_alarm;
    logic [2:0]   fault_flags;
    logic [1:0]   voter_disc;

    tmr_wrapper #(.DATA_WIDTH(W)) dut (
        .clk(clk), .rst_n(rst_n),
        .din_0(d0), .din_1(d1), .din_2(d2),
        .dout_voted(dout),
        .tmr_alarm(tmr_alarm),
        .fault_flags(fault_flags),
        .voter_disagreements(voter_disc)
    );

    initial clk = 0;
    always #(CLK_P/2) clk = ~clk;

    int pass_cnt = 0, fail_cnt = 0;

    task check(input string name, input logic exp, input logic got);
        if (exp === got) begin $display("[PASS] %s", name); pass_cnt++; end
        else             begin $display("[FAIL] %s: exp=%b got=%b", name, exp, got); fail_cnt++; end
    endtask

    task check32(input string name, input logic [W-1:0] exp, input logic [W-1:0] got);
        if (exp === got) begin $display("[PASS] %s: 0x%08h", name, got); pass_cnt++; end
        else             begin $display("[FAIL] %s: exp=0x%08h got=0x%08h", name, exp, got); fail_cnt++; end
    endtask

    initial begin
        $display("=== tb_tmr_wrapper ===");
        rst_n = 0; d0 = 0; d1 = 0; d2 = 0;
        repeat(4) @(posedge clk); rst_n = 1;

        // Test 1: All agree
        @(posedge clk); d0 = 32'hDEADBEEF; d1 = 32'hDEADBEEF; d2 = 32'hDEADBEEF;
        #1;
        check  ("T1: No alarm when all agree",     1'b0, tmr_alarm);
        check32("T1: Voted output correct",        32'hDEADBEEF, dout);

        // Test 2: Single bit-flip in voter 0
        @(posedge clk); d0 = 32'hDEADBEEE; d1 = 32'hDEADBEEF; d2 = 32'hDEADBEEF;
        #1;
        check  ("T2: Alarm on voter-0 bit-flip",  1'b1, tmr_alarm);
        check32("T2: Corrected output = majority", 32'hDEADBEEF, dout);
        check  ("T2: fault_flags[0] set",          1'b1, fault_flags[0]);
        check  ("T2: fault_flags[1] clear",        1'b0, fault_flags[1]);
        check  ("T2: Severity = 1",                1'b1, voter_disc == 2'd1);

        // Test 3: Two voters corrupted
        @(posedge clk); d0 = 32'h00000000; d1 = 32'h00000000; d2 = 32'hDEADBEEF;
        #1;
        check  ("T3: Alarm on two corrupted",     1'b1, tmr_alarm);
        // d0=d1=0, d2=0xDEADBEEF → majority=0, only d2 disagrees → severity=1 (1 faulty module)
        check  ("T3: Severity = 1 (d2 outlier)",  1'b1, voter_disc == 2'd1);
        $display("    Voted output (trust majority): 0x%08h", dout);

        // Test 4: All differ
        @(posedge clk); d0 = 32'hAAAAAAAA; d1 = 32'h55555555; d2 = 32'hFFFFFFFF;
        #1;
        check("T4: Alarm on all-differ",          1'b1, tmr_alarm);

        $display("\nTMR Results: %0d PASS, %0d FAIL", pass_cnt, fail_cnt);
        $finish;
    end

endmodule
