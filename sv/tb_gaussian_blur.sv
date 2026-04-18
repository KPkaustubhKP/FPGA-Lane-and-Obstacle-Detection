// =============================================================================
// File        : tb_gaussian_blur.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// TC-01 Reset | TC-02 Constant field | TC-03 Impulse | TC-04 Ramp
// TC-05 Checkerboard | TC-06 Sync | TC-07 Valid gate | TC-08 Throughput
// =============================================================================
`timescale 1ns / 1ps

module tb_gaussian_blur;

    localparam int CLK_HALF = 5;
    localparam int IMG_W    = 16;
    localparam int IMG_H    = 8;
    localparam int WARMUP   = 2*IMG_W + 8;
    localparam int BUF_SZ   = IMG_W*IMG_H + WARMUP;

    logic clk = 0;
    logic rst_n;
    always #CLK_HALF clk = ~clk;

    logic       i_valid, i_hsync, i_vsync;
    logic [7:0] i_pixel, o_pixel;
    logic       o_valid, o_hsync, o_vsync;

    gaussian_blur #(.DATA_WIDTH(8),.IMG_WIDTH(IMG_W),.IMG_HEIGHT(IMG_H)) dut (.*);

    int pass_cnt = 0;
    int fail_cnt = 0;

    // Shared capture buffer (module scope)
    logic [7:0] g_outbuf [0:BUF_SZ-1];
    int         g_out_cnt;

    task automatic do_reset;
        rst_n=0; i_valid=0; i_pixel=0; i_hsync=0; i_vsync=0;
        repeat(4) @(posedge clk); #1;
        rst_n=1; @(posedge clk); #1;
    endtask

    // Stream one full frame (pixel = pattern(r,c)), collect into g_outbuf
    // pattern: 0=constant128, 1=impulse(r=H/2,c=W/2), 2=ramp(=col), 3=checkerboard
    task automatic stream_frame(input int pattern);
        g_out_cnt = 0;
        @(negedge clk); i_valid=1;
        for (int r=0; r<IMG_H; r++) begin
            for (int c=0; c<IMG_W; c++) begin
                case (pattern)
                    0: i_pixel = 8'd128;
                    1: i_pixel = (r==IMG_H/2 && c==IMG_W/2) ? 8'd255 : 8'd0;
                    2: i_pixel = 8'(c);
                    3: i_pixel = ((r+c)%2==0) ? 8'd255 : 8'd0;
                    default: i_pixel = 8'd0;
                endcase
                i_hsync = (c==IMG_W-1); i_vsync = (r==IMG_H-1 && c==IMG_W-1);
                @(posedge clk); #1;
                if (o_valid && g_out_cnt < BUF_SZ) begin g_outbuf[g_out_cnt]=o_pixel; g_out_cnt++; end
                i_hsync=0; i_vsync=0;
            end
        end
        i_valid=0;
        for (int d=0; d<WARMUP; d++) begin
            @(posedge clk); #1;
            if (o_valid && g_out_cnt < BUF_SZ) begin g_outbuf[g_out_cnt]=o_pixel; g_out_cnt++; end
        end
    endtask

    initial begin
        $dumpfile("tb_gaussian_blur.vcd"); $dumpvars(0,tb_gaussian_blur);
        $display("╔══════════════════════════════════════════════════╗");
        $display("║  TB: gaussian_blur (W=%0d H=%0d)                  ║", IMG_W, IMG_H);
        $display("╚══════════════════════════════════════════════════╝");

        // TC-01 Reset
        $display("\n── TC-01 Reset ──────────────────────────────────");
        rst_n=0; i_valid=1; i_pixel=8'hFF;
        repeat(5) @(posedge clk); #1;
        if (o_pixel==0 && o_valid==0) begin
            $display("  [PASS] TC-01: Outputs zero during reset"); pass_cnt++;
        end else begin
            $display("  [FAIL] TC-01: pixel=%0h valid=%b", o_pixel, o_valid); fail_cnt++;
        end
        do_reset();

        // TC-02 Constant field — all outputs must equal 128
        $display("\n── TC-02 Constant Field (all=128) ───────────────");
        begin
            int wrong2 = 0;
            stream_frame(0);
            for (int i=0; i<g_out_cnt; i++)
                if (g_outbuf[i] != 8'd128) wrong2++;
            if (wrong2==0 && g_out_cnt>0)
                begin $display("  [PASS] TC-02: All %0d outputs == 128", g_out_cnt); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-02: %0d of %0d wrong", wrong2, g_out_cnt); fail_cnt++; end
        end
        do_reset();

        // TC-03 Impulse — centre pixel: 255×4/16 = 63±2
        $display("\n── TC-03 Impulse Response ───────────────────────");
        begin
            int ctr_idx = (IMG_H/2)*IMG_W + (IMG_W/2);
            int adj_idx = ctr_idx + 1;
            stream_frame(1);
            if (ctr_idx < g_out_cnt) begin
                if (int'(g_outbuf[ctr_idx]) >= 60 && int'(g_outbuf[ctr_idx]) <= 66)
                    begin $display("  [PASS] TC-03a: Centre = %0d (expect 63±3)", g_outbuf[ctr_idx]); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-03a: Centre = %0d", g_outbuf[ctr_idx]); fail_cnt++; end
            end
            if (adj_idx < g_out_cnt) begin
                if (int'(g_outbuf[adj_idx]) >= 26 && int'(g_outbuf[adj_idx]) <= 36)
                    begin $display("  [PASS] TC-03b: H-adj = %0d (expect 31±5)", g_outbuf[adj_idx]); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-03b: H-adj = %0d", g_outbuf[adj_idx]); fail_cnt++; end
            end
        end
        do_reset();

        // TC-04 Ramp — column value should be preserved in interior
        $display("\n── TC-04 Vertical Ramp ───────────────────────────");
        begin
            int wrong4 = 0;
            stream_frame(2);
            // Interior: rows 2..H-3, cols 2..W-4
            for (int r=2; r<IMG_H-2; r++) begin
                for (int c=2; c<IMG_W-3; c++) begin
                    int idx4 = r*IMG_W + c;
                    if (idx4 < g_out_cnt) begin
                        int diff4 = (int'(g_outbuf[idx4]) > c) ? int'(g_outbuf[idx4])-c : c-int'(g_outbuf[idx4]);
                        if (diff4 > 1) wrong4++;
                    end
                end
            end
            if (wrong4==0)
                begin $display("  [PASS] TC-04: Interior ramp pixels correct (±1)"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-04: %0d interior pixels wrong", wrong4); fail_cnt++; end
        end
        do_reset();

        // TC-05 Checkerboard — interior outputs must not be 0 or 255
        $display("\n── TC-05 Checkerboard Noise Reduction ───────────");
        begin
            int extremes = 0;
            stream_frame(3);
            for (int i=IMG_W+1; i<g_out_cnt-IMG_W-1; i++)
                if (g_outbuf[i]==8'd0 || g_outbuf[i]==8'd255) extremes++;
            if (extremes==0)
                begin $display("  [PASS] TC-05: No extreme values in interior"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-05: %0d interior extreme values", extremes); fail_cnt++; end
        end
        do_reset();

        // TC-06 Sync propagation — count hsync pulses seen at output
        $display("\n── TC-06 Sync Propagation ────────────────────────");
        begin
            int hs_n = 0;
            @(negedge clk); i_valid=1;
            for (int r=0; r<IMG_H; r++) begin
                for (int c=0; c<IMG_W; c++) begin
                    i_pixel=8'd100;
                    i_hsync=(c==IMG_W-1); i_vsync=0;
                    @(posedge clk); #1;
                    if (o_hsync) hs_n++;
                    i_hsync=0;
                end
            end
            i_valid=0;
            repeat(WARMUP) begin @(posedge clk); #1; if (o_hsync) hs_n++; end
            if (hs_n==IMG_H)
                begin $display("  [PASS] TC-06: Exactly %0d hsync pulses (one per row)", hs_n); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-06: %0d pulses (expected %0d)", hs_n, IMG_H); fail_cnt++; end
        end
        do_reset();

        // TC-07 Valid gating
        $display("\n── TC-07 Valid Gating ────────────────────────────");
        begin
            int sp7 = 0;
            i_valid=0; i_pixel=8'hFF;
            repeat(WARMUP) begin @(posedge clk); #1; if (o_valid) sp7++; end
            if (sp7==0)
                begin $display("  [PASS] TC-07: No o_valid without i_valid"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-07: %0d spurious o_valid", sp7); fail_cnt++; end
        end
        do_reset();

        // TC-08 Throughput — after warm-up, 1 out per 1 in
        $display("\n── TC-08 Throughput ──────────────────────────────");
        begin
            int in8=0, out8=0;
            @(negedge clk); i_valid=1;
            // Warm-up 2 rows
            for (int i=0; i<2*IMG_W; i++) begin
                i_pixel=8'd128; i_hsync=0; i_vsync=0;
                @(posedge clk); #1; if(o_valid) out8++;
            end
            // Measurement: 1 row
            for (int i=0; i<IMG_W; i++) begin
                i_pixel=8'd128; in8++;
                @(posedge clk); #1; if(o_valid) out8++;
            end
            i_valid=0;
            if (out8 >= in8)
                begin $display("  [PASS] TC-08: %0d in → %0d out (1 pix/clk after warmup)", in8, out8); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-08: %0d in → %0d out (bubble)", in8, out8); fail_cnt++; end
        end

        $display("\n══════════════════════════════════════════════════");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        if (fail_cnt==0) $display("  ✅  ALL TESTS PASSED");
        else             $display("  ❌  %0d TEST(S) FAILED", fail_cnt);
        $finish;
    end
    initial begin #10_000_000; $display("[TIMEOUT]"); $finish; end

endmodule : tb_gaussian_blur
