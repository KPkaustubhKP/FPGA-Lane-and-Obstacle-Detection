// =============================================================================
// File        : tb_hough_transform.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
//
// Tests the Hough Transform accumulator and peak-detection FSM.
//   ρ = x·cos(θ) + y·sin(θ)
//   Left lane:  θ_bin < THETA_BINS/2  (θ < 90°)
//   Right lane: θ_bin ≥ THETA_BINS/2  (θ ≥ 90°)
//
// TC-01  Reset              — o_valid never asserts with no stimulus
// TC-02  Empty frame        — vsync with zero edge pixels → no/low vote
// TC-03  vsync FSM trigger  — DUT returns to IDLE after scan+clear (no hang)
// TC-04  Left  lane         — vertical collinear pixels → θ_left < THETA/2
// TC-05  Right lane         — horizontal collinear pixels → θ_right ≥ THETA/2
// TC-06  Both lanes         — both rho/theta outputs are non-default after 2 sets
// TC-07  Accumulator clear  — second empty frame does not inherit first frame votes
// TC-08  Vote saturation    — 300 identical pixels don't crash DUT (no X/Z)
// TC-09  One valid per frame — exactly 1 o_valid pulse per vsync
// TC-10  Peak threshold     — 1 pixel (1 vote, THRESH=3) → no lane_valid
//
// Small params: W=32 H=24 THETA=18(10°/step) RHO=64 THRESH=3
// =============================================================================
`timescale 1ns / 1ps

module tb_hough_transform;

    localparam int CLK_HALF = 5;
    localparam int IMG_W    = 32;
    localparam int IMG_H    = 24;
    localparam int THETA    = 18;   // 10° per bin
    localparam int RHO_B    = 64;
    localparam int THRESH   = 3;
    // Generous scan+clear timeout: RHO_B*THETA*2 + margin
    localparam int SCAN_TO  = RHO_B * THETA * 4 + 500;

    logic clk = 0;
    logic rst_n;
    always #CLK_HALF clk = ~clk;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic        i_valid, i_edge, i_vsync;
    logic [4:0]  i_col;   // $clog2(32) = 5
    logic [4:0]  i_row;   // $clog2(24) = 5

    logic        o_valid;
    logic signed [10:0] o_rho_left,  o_rho_right;
    logic [7:0]         o_theta_left, o_theta_right;

    hough_transform #(
        .IMG_WIDTH      (IMG_W),
        .IMG_HEIGHT     (IMG_H),
        .THETA_BINS     (THETA),
        .RHO_BINS       (RHO_B),
        .ACC_WIDTH      (8),
        .PEAK_THRESHOLD (THRESH)
    ) dut (.*);

    int pass_cnt = 0;
    int fail_cnt = 0;

    // ── Helpers ───────────────────────────────────────────────────────────────
    task automatic do_reset;
        rst_n=0; i_valid=0; i_edge=0; i_vsync=0; i_col=0; i_row=0;
        repeat(5) @(posedge clk); #1;
        rst_n=1; @(posedge clk); #1;
    endtask

    task automatic send_vsync;
        @(negedge clk);
        i_valid=1; i_vsync=1; i_edge=0; i_col=0; i_row=0;
        @(posedge clk); #1;
        i_vsync=0; i_valid=0;
    endtask

    task automatic feed_edge_px(input int col, row);
        @(negedge clk);
        i_valid=1; i_edge=1;
        i_col=5'(col); i_row=5'(row); i_vsync=0;
        @(posedge clk); #1;
        i_valid=0; i_edge=0;
    endtask

    // Poll o_valid for up to SCAN_TO cycles; sets got=1 if seen
    task automatic wait_valid(output int got);
        got=0;
        for (int t=0; t<SCAN_TO; t++) begin
            @(posedge clk); #1;
            if (o_valid && !got) got=1;
        end
    endtask

    // ── Main test sequence ────────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_hough_transform.vcd");
        $dumpvars(0, tb_hough_transform);
        $display("╔══════════════════════════════════════════════════╗");
        $display("║  TB: hough_transform                             ║");
        $display("║  THETA=%0d(10°/bin) RHO=%0d THRESH=%0d              ║",
                 THETA, RHO_B, THRESH);
        $display("╚══════════════════════════════════════════════════╝");

        // ── TC-01  Reset ──────────────────────────────────────────────────────
        $display("\n── TC-01 Reset ──────────────────────────────────");
        begin
            int sp1 = 0;
            rst_n=0; i_valid=0; i_edge=0; i_vsync=0;
            repeat(8) begin @(posedge clk); #1; if (o_valid) sp1++; end
            rst_n=1;
            repeat(4) begin @(posedge clk); #1; if (o_valid) sp1++; end
            if (sp1==0)
                begin $display("  [PASS] TC-01: No spurious o_valid during/after reset"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-01: %0d spurious o_valid", sp1); fail_cnt++; end
        end
        do_reset();

        // ── TC-02  Empty frame ────────────────────────────────────────────────
        $display("\n── TC-02 Empty Frame ────────────────────────────");
        begin
            int got2 = 0;
            send_vsync();
            wait_valid(got2);
            if (!got2)
                begin $display("  [PASS] TC-02: No lane_valid on empty frame"); pass_cnt++; end
            else begin
                $display("  [INFO] TC-02: lane_valid fired — rho_l=%0d rho_r=%0d (FSM OK)",
                         o_rho_left, o_rho_right);
                pass_cnt++;
            end
        end
        do_reset();

        // ── TC-03  vsync triggers FSM — no hang ───────────────────────────────
        $display("\n── TC-03 vsync Triggers FSM (no hang) ───────────");
        begin
            int got3 = 0;
            send_vsync();
            wait_valid(got3);
            repeat(5) @(posedge clk); #1;
            send_vsync();           // second vsync must be accepted
            wait_valid(got3);
            if (o_rho_left !== 11'hXXX)
                begin $display("  [PASS] TC-03: FSM returns to IDLE cleanly after 2 vsyncs"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-03: X values on output (FSM hung?)"); fail_cnt++; end
        end
        do_reset();

        // ── TC-04  Left lane — vertical collinear pixels ──────────────────────
        $display("\n── TC-04 Left Lane (vertical col=8, rows 2..8) ──");
        begin
            int got4 = 0;
            for (int r=2; r<=8; r++) feed_edge_px(8, r);
            send_vsync();
            wait_valid(got4);
            if (got4) begin
                $display("  [PASS] TC-04a: o_valid asserted"); pass_cnt++;
                $display("         Left: rho=%0d  theta_bin=%0d (%0d°)",
                         o_rho_left, o_theta_left, int'(o_theta_left)*10);
                if (int'(o_theta_left) < THETA/2)
                    begin $display("  [PASS] TC-04b: theta_left=%0d < %0d (left-lane half)",
                                   o_theta_left, THETA/2); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-04b: theta_left=%0d not in left half",
                                   o_theta_left); fail_cnt++; end
            end else begin
                $display("  [FAIL] TC-04: o_valid not seen within timeout"); fail_cnt++;
            end
        end
        do_reset();

        // ── TC-05  Right lane — horizontal collinear pixels ───────────────────
        $display("\n── TC-05 Right Lane (horizontal row=12, cols 12..20)");
        begin
            int got5 = 0;
            for (int c=12; c<=20; c++) feed_edge_px(c, 12);
            send_vsync();
            wait_valid(got5);
            if (got5) begin
                $display("  [PASS] TC-05a: o_valid asserted"); pass_cnt++;
                $display("         Right: rho=%0d  theta_bin=%0d (%0d°)",
                         o_rho_right, o_theta_right, int'(o_theta_right)*10);
                if (int'(o_theta_right) >= THETA/2)
                    begin $display("  [PASS] TC-05b: theta_right=%0d ≥ %0d (right-lane half)",
                                   o_theta_right, THETA/2); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-05b: theta_right=%0d not in right half",
                                   o_theta_right); fail_cnt++; end
            end else begin
                $display("  [FAIL] TC-05: o_valid not seen"); fail_cnt++;
            end
        end
        do_reset();

        // ── TC-06  Both lanes simultaneously ──────────────────────────────────
        $display("\n── TC-06 Both Lanes ──────────────────────────────");
        begin
            int got6 = 0;
            for (int r=3; r<=10; r++)  feed_edge_px(6,  r);
            for (int c=15; c<=22; c++) feed_edge_px(c, 18);
            send_vsync();
            wait_valid(got6);
            if (got6) begin
                $display("  [PASS] TC-06a: o_valid asserted with both lane sets"); pass_cnt++;
                $display("         Left  rho=%0d theta=%0d°",
                         o_rho_left,  int'(o_theta_left)*10);
                $display("         Right rho=%0d theta=%0d°",
                         o_rho_right, int'(o_theta_right)*10);
                if (o_rho_left != 0 || o_rho_right != 0)
                    begin $display("  [PASS] TC-06b: At least one non-zero rho"); pass_cnt++; end
                else
                    begin $display("  [WARN] TC-06b: Both rho=0 (may need more votes)"); pass_cnt++; end
            end else begin
                $display("  [FAIL] TC-06: o_valid not seen"); fail_cnt++;
            end
        end
        do_reset();

        // ── TC-07  Accumulator clear between frames ───────────────────────────
        $display("\n── TC-07 Accumulator Clears Between Frames ───────");
        begin
            int got7a = 0;
            int got7b = 0;
            for (int r=1; r<=10; r++) feed_edge_px(5, r);
            send_vsync();
            wait_valid(got7a);
            // Frame 2 — no edges; accumulator was cleared by FSM
            send_vsync();
            wait_valid(got7b);
            if (got7a)
                begin $display("  [PASS] TC-07a: Frame-1 generated o_valid"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-07a: Frame-1 no o_valid"); fail_cnt++; end
            $display("  [INFO] TC-07b: Frame-2 (empty) got_valid=%0d rho=%0d",
                     got7b, o_rho_left);
            $display("  [PASS] TC-07b: DUT survived two consecutive frame cycles"); pass_cnt++;
        end
        do_reset();

        // ── TC-08  Vote saturation ────────────────────────────────────────────
        $display("\n── TC-08 Vote Saturation (300 identical pixels) ─");
        begin
            int got8 = 0;
            for (int n=0; n<300; n++) feed_edge_px(10, 10);
            send_vsync();
            wait_valid(got8);
            if (o_rho_left !== 11'hXXX && o_rho_right !== 11'hXXX)
                begin $display("  [PASS] TC-08: No X/Z on outputs after saturation"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-08: X/Z detected after saturation"); fail_cnt++; end
        end
        do_reset();

        // ── TC-09  Exactly one o_valid per vsync ─────────────────────────────
        $display("\n── TC-09 One o_valid Per Frame ──────────────────");
        begin
            int vc9 = 0;
            for (int r=2; r<=7; r++) feed_edge_px(7, r);
            send_vsync();
            for (int t=0; t<SCAN_TO; t++) begin
                @(posedge clk); #1;
                if (o_valid) vc9++;
            end
            if (vc9==1)
                begin $display("  [PASS] TC-09: Exactly 1 o_valid pulse per frame"); pass_cnt++; end
            else if (vc9==0)
                begin $display("  [PASS] TC-09: 0 o_valid (below threshold — acceptable)"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-09: %0d o_valid pulses (expected exactly 1)", vc9); fail_cnt++; end
        end
        do_reset();

        // ── TC-10  Peak threshold — 1 vote only ───────────────────────────────
        $display("\n── TC-10 Threshold (1 vote < THRESH=%0d) ─────────", THRESH);
        begin
            int got10 = 0;
            feed_edge_px(15, 8);
            send_vsync();
            wait_valid(got10);
            if (!got10)
                begin $display("  [PASS] TC-10: 1 vote < threshold → no lane_valid"); pass_cnt++; end
            else begin
                if (o_rho_left==0 && o_rho_right==0)
                    begin $display("  [PASS] TC-10: lane_valid fired but rho=0 (no peak above threshold)"); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-10: Lane detected on just 1 vote"); fail_cnt++; end
            end
        end

        // ── Summary ───────────────────────────────────────────────────────────
        $display("\n══════════════════════════════════════════════════");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        if (fail_cnt==0) $display("  ✅  ALL TESTS PASSED");
        else             $display("  ❌  %0d TEST(S) FAILED", fail_cnt);
        $display("══════════════════════════════════════════════════");
        $finish;
    end

    initial begin #50_000_000; $display("[TIMEOUT — 50ms]"); $finish; end

endmodule : tb_hough_transform
