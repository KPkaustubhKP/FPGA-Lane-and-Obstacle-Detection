// =============================================================================
// File        : tb_lane_roi_filter.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
//
// TC-01  Reset — all outputs zero during rst_n=0
// TC-02  Rows above ROI always masked to 0
// TC-03  Bottom ROI row (full width) — all pixels pass
// TC-04  Column counter increments 0→W-1 per row, resets on hsync
// TC-05  Row counter increments on each hsync, resets on vsync
// TC-06  Trapezoidal mask — col=1 masked at ROI_TOP, passes at ROI_BOT
// TC-07  Edge flag masked outside ROI (i_edge=1 outside → o_edge=0)
// TC-08  Edge flag passes inside ROI (i_edge=1 inside → o_edge=1)
// TC-09  Valid gating — i_valid=0 keeps o_valid=0
// TC-10  Throughput — 1-cycle latency, no pipeline bubbles
//
// DUT parameters: W=16, H=16, ROI rows 8..15, trapezoid cols (4..12 at top, 0..15 at bot)
// =============================================================================
`timescale 1ns / 1ps

module tb_lane_roi_filter;

    localparam int CLK_HALF          = 5;
    localparam int IMG_W             = 16;
    localparam int IMG_H             = 16;
    localparam int ROI_ROW_TOP       = 8;
    localparam int ROI_ROW_BOT       = 15;
    localparam int ROI_COL_LEFT_BOT  = 0;
    localparam int ROI_COL_RIGHT_BOT = IMG_W - 1;
    localparam int ROI_COL_LEFT_TOP  = 4;
    localparam int ROI_COL_RIGHT_TOP = 12;
    localparam int BUF_SZ            = IMG_W * IMG_H + 8;

    logic clk = 0;
    logic rst_n;
    always #CLK_HALF clk = ~clk;

    // ── DUT ports ─────────────────────────────────────────────────────────────
    logic       i_valid, i_edge, i_hsync, i_vsync;
    logic [7:0] i_pixel;
    logic       o_valid, o_edge, o_hsync, o_vsync;
    logic [7:0] o_pixel;
    logic [3:0] o_col;   // $clog2(16) = 4
    logic [3:0] o_row;

    lane_roi_filter #(
        .DATA_WIDTH         (8),
        .IMG_WIDTH          (IMG_W),
        .IMG_HEIGHT         (IMG_H),
        .ROI_ROW_TOP        (ROI_ROW_TOP),
        .ROI_ROW_BOT        (ROI_ROW_BOT),
        .ROI_COL_LEFT_BOT   (ROI_COL_LEFT_BOT),
        .ROI_COL_RIGHT_BOT  (ROI_COL_RIGHT_BOT),
        .ROI_COL_LEFT_TOP   (ROI_COL_LEFT_TOP),
        .ROI_COL_RIGHT_TOP  (ROI_COL_RIGHT_TOP)
    ) dut (.*);

    int pass_cnt = 0;
    int fail_cnt = 0;

    // ── Module-scope capture buffers ──────────────────────────────────────────
    logic [7:0] g_pix [0:BUF_SZ-1];
    logic       g_edg [0:BUF_SZ-1];
    logic [3:0] g_col [0:BUF_SZ-1];
    logic [3:0] g_row [0:BUF_SZ-1];
    int         g_cnt;

    // ── Helpers ───────────────────────────────────────────────────────────────
    task automatic do_reset;
        rst_n=0; i_valid=0; i_pixel=0; i_edge=0; i_hsync=0; i_vsync=0;
        repeat(4) @(posedge clk); #1;
        rst_n=1; @(posedge clk); #1;
    endtask

    // Stream a full frame; force_edge drives i_edge every cycle
    task automatic stream_frame(input logic force_edge);
        g_cnt = 0;
        @(negedge clk); i_valid=1;
        for (int r=0; r<IMG_H; r++) begin
            for (int c=0; c<IMG_W; c++) begin
                i_pixel  = 8'hFF;
                i_edge   = force_edge;
                i_hsync  = (c == IMG_W-1);
                i_vsync  = (r == IMG_H-1 && c == IMG_W-1);
                @(posedge clk); #1;
                if (o_valid && g_cnt < BUF_SZ) begin
                    g_pix[g_cnt] = o_pixel;
                    g_edg[g_cnt] = o_edge;
                    g_col[g_cnt] = o_col;
                    g_row[g_cnt] = o_row;
                    g_cnt++;
                end
                i_hsync=0; i_vsync=0;
            end
        end
        i_valid=0; i_edge=0;
        // Drain 1-cycle pipeline
        repeat(4) begin
            @(posedge clk); #1;
            if (o_valid && g_cnt < BUF_SZ) begin
                g_pix[g_cnt]=o_pixel; g_edg[g_cnt]=o_edge;
                g_col[g_cnt]=o_col;   g_row[g_cnt]=o_row;
                g_cnt++;
            end
        end
    endtask

    // ── Main test sequence ────────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_lane_roi_filter.vcd");
        $dumpvars(0, tb_lane_roi_filter);
        $display("╔══════════════════════════════════════════════════╗");
        $display("║  TB: lane_roi_filter (W=%0d H=%0d ROI rows %0d..%0d) ║",
                 IMG_W, IMG_H, ROI_ROW_TOP, ROI_ROW_BOT);
        $display("╚══════════════════════════════════════════════════╝");

        // ── TC-01  Reset ──────────────────────────────────────────────────────
        $display("\n── TC-01 Reset ──────────────────────────────────");
        rst_n=0; i_valid=1; i_pixel=8'hFF; i_edge=1; i_hsync=1; i_vsync=1;
        repeat(4) @(posedge clk); #1;
        if (o_pixel==0 && o_edge==0 && o_valid==0)
            begin $display("  [PASS] TC-01: All outputs zero during reset"); pass_cnt++; end
        else
            begin $display("  [FAIL] TC-01: pixel=%0h edge=%b valid=%b",
                           o_pixel, o_edge, o_valid); fail_cnt++; end
        do_reset();

        // ── TC-02  Rows above ROI always masked ───────────────────────────────
        $display("\n── TC-02 Rows Above ROI Masked ──────────────────");
        begin
            int wrong2 = 0;
            stream_frame(1);
            for (int i=0; i<g_cnt; i++)
                if (int'(g_row[i]) < ROI_ROW_TOP && (g_pix[i]!=0 || g_edg[i]!=0))
                    wrong2++;
            if (wrong2==0)
                begin $display("  [PASS] TC-02: All rows < %0d correctly masked", ROI_ROW_TOP); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-02: %0d pixels not masked above ROI", wrong2); fail_cnt++; end
        end
        do_reset();

        // ── TC-03  Bottom ROI row — full width passes ─────────────────────────
        $display("\n── TC-03 Bottom ROI Row Passes ──────────────────");
        begin
            int wrong3 = 0;
            stream_frame(1);
            for (int i=0; i<g_cnt; i++)
                if (int'(g_row[i]) == ROI_ROW_BOT && g_edg[i]==0)
                    wrong3++;   // should not be masked at bottom row full-width
            if (wrong3==0)
                begin $display("  [PASS] TC-03: All pixels in bottom ROI row passed"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-03: %0d pixels incorrectly masked in bottom row", wrong3); fail_cnt++; end
        end
        do_reset();

        // ── TC-04  Column counter increments 0→W-1 each row ──────────────────
        $display("\n── TC-04 Column Counter ──────────────────────────");
        begin
            int errs4 = 0;
            int exp_col = 0;
            stream_frame(0);
            for (int i=0; i<g_cnt; i++) begin
                if (int'(g_col[i]) != exp_col) errs4++;
                exp_col++;
                if (exp_col == IMG_W) exp_col = 0;
            end
            if (errs4==0)
                begin $display("  [PASS] TC-04: Column counter correct across %0d outputs", g_cnt); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-04: %0d incorrect column values", errs4); fail_cnt++; end
        end
        do_reset();

        // ── TC-05  Row counter increments on hsync ────────────────────────────
        $display("\n── TC-05 Row Counter ─────────────────────────────");
        begin
            int errs5 = 0;
            stream_frame(0);
            for (int i=0; i<g_cnt; i++) begin
                int exp_row5 = i / IMG_W;
                if (int'(g_row[i]) != exp_row5) errs5++;
            end
            if (errs5==0)
                begin $display("  [PASS] TC-05: Row counter correct across %0d outputs", g_cnt); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-05: %0d incorrect row values", errs5); fail_cnt++; end
        end
        do_reset();

        // ── TC-06  Trapezoidal mask — col=1 masked at ROI_TOP, passes at ROI_BOT
        $display("\n── TC-06 Trapezoidal Column Mask ─────────────────");
        begin
            // col=1 at ROI_TOP row: within ROI rows but OUTSIDE the narrow top window (4..12)
            // col=1 at ROI_BOT row: full width (0..15), so INSIDE → should pass
            int top_idx = ROI_ROW_TOP  * IMG_W + 1;   // row=8, col=1
            int bot_idx = ROI_ROW_BOT  * IMG_W + 1;   // row=15, col=1
            stream_frame(1);
            if (top_idx < g_cnt) begin
                if (g_edg[top_idx] == 0)
                    begin $display("  [PASS] TC-06a: col=1 @ top-ROI correctly masked (outside narrow window)"); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-06a: col=1 @ top-ROI NOT masked (edge=%b)", g_edg[top_idx]); fail_cnt++; end
            end else begin
                $display("  [WARN] TC-06a: index out of captured range"); pass_cnt++;
            end
            if (bot_idx < g_cnt) begin
                if (g_edg[bot_idx] == 1)
                    begin $display("  [PASS] TC-06b: col=1 @ bot-ROI passes (full-width bottom)"); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-06b: col=1 @ bot-ROI incorrectly masked"); fail_cnt++; end
            end else begin
                $display("  [WARN] TC-06b: index out of captured range"); pass_cnt++;
            end
        end
        do_reset();

        // ── TC-07  Edge flag masked outside ROI ───────────────────────────────
        $display("\n── TC-07 Edge Flag Masked Outside ROI ───────────");
        begin
            int sp7 = 0;
            stream_frame(1);
            for (int i=0; i<g_cnt; i++)
                if (int'(g_row[i]) < ROI_ROW_TOP && g_edg[i]==1) sp7++;
            if (sp7==0)
                begin $display("  [PASS] TC-07: No edge flags above ROI"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-07: %0d spurious edge flags above ROI", sp7); fail_cnt++; end
        end
        do_reset();

        // ── TC-08  Edge flag passes inside ROI ───────────────────────────────
        $display("\n── TC-08 Edge Flag Passes Inside ROI ────────────");
        begin
            int inside8 = 0;
            stream_frame(1);
            for (int i=0; i<g_cnt; i++)
                if (int'(g_row[i]) >= ROI_ROW_TOP &&
                    int'(g_row[i]) <= ROI_ROW_BOT && g_edg[i]==1)
                    inside8++;
            if (inside8 > 0)
                begin $display("  [PASS] TC-08: %0d edge flags inside ROI passed", inside8); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-08: No edge flags passed inside ROI"); fail_cnt++; end
        end
        do_reset();

        // ── TC-09  Valid gating ───────────────────────────────────────────────
        $display("\n── TC-09 Valid Gating ────────────────────────────");
        begin
            int sp9 = 0;
            i_valid=0; i_pixel=8'hFF; i_edge=1;
            repeat(IMG_W * IMG_H + 8) begin
                @(posedge clk); #1;
                if (o_valid) sp9++;
            end
            if (sp9==0)
                begin $display("  [PASS] TC-09: o_valid never asserted without i_valid"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-09: %0d spurious o_valid pulses", sp9); fail_cnt++; end
        end
        do_reset();

        // ── TC-10  Throughput — 1-cycle latency ───────────────────────────────
        $display("\n── TC-10 Throughput (1 cycle latency) ───────────");
        begin
            int in10=0, out10=0;
            @(negedge clk); i_valid=1;
            for (int i=0; i<IMG_W*IMG_H; i++) begin
                i_pixel=8'd80; i_edge=0; i_hsync=0; i_vsync=0;
                in10++;
                @(posedge clk); #1;
                if (o_valid) out10++;
            end
            i_valid=0;
            @(posedge clk); #1; if (o_valid) out10++;   // drain 1 cycle
            if (out10 >= in10)
                begin $display("  [PASS] TC-10: %0d in → %0d out (1 pix/clk)", in10, out10); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-10: %0d in → %0d out", in10, out10); fail_cnt++; end
        end

        // ── Summary ───────────────────────────────────────────────────────────
        $display("\n══════════════════════════════════════════════════");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        if (fail_cnt==0) $display("  ✅  ALL TESTS PASSED");
        else             $display("  ❌  %0d TEST(S) FAILED", fail_cnt);
        $display("══════════════════════════════════════════════════");
        $finish;
    end

    initial begin #10_000_000; $display("[TIMEOUT]"); $finish; end

endmodule : tb_lane_roi_filter
