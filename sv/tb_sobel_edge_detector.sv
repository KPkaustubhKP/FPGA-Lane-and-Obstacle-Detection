// =============================================================================
// File        : tb_sobel_edge_detector.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// TC-01 Reset | TC-02 Flat | TC-03 Vertical edge | TC-04 Horizontal edge
// TC-05 Diagonal | TC-06 Threshold | TC-07 Clamping | TC-08 Sync | TC-09 Valid | TC-10 Throughput
// =============================================================================
`timescale 1ns / 1ps

module tb_sobel_edge_detector;

    localparam int CLK_HALF = 5;
    localparam int IMG_W    = 16;
    localparam int IMG_H    = 10;
    localparam int THRESH   = 32;
    localparam int WARMUP   = 2*IMG_W + 8;
    localparam int BUF_SZ   = IMG_W*IMG_H + WARMUP;

    logic clk = 0;
    logic rst_n;
    always #CLK_HALF clk = ~clk;

    logic       i_valid, i_hsync, i_vsync;
    logic [7:0] i_pixel;
    logic       o_valid, o_edge, o_hsync, o_vsync;
    logic [7:0] o_gradient;

    sobel_edge_detector #(
        .DATA_WIDTH(8),.IMG_WIDTH(IMG_W),.IMG_HEIGHT(IMG_H),.THRESHOLD(THRESH)
    ) dut (.*);

    int pass_cnt = 0;
    int fail_cnt = 0;

    logic [7:0] g_grad [0:BUF_SZ-1];
    logic       g_edge [0:BUF_SZ-1];
    int         g_cnt;

    task automatic do_reset;
        rst_n=0; i_valid=0; i_pixel=0; i_hsync=0; i_vsync=0;
        repeat(4) @(posedge clk); #1; rst_n=1; @(posedge clk); #1;
    endtask

    // pattern: 0=flat200, 1=vertical(L=0,R=255), 2=horizontal(T=0,B=255), 3=diagonal, 4=flat128
    task automatic stream_frame(input int pattern);
        g_cnt=0;
        @(negedge clk); i_valid=1;
        for (int r=0; r<IMG_H; r++) begin
            for (int c=0; c<IMG_W; c++) begin
                case(pattern)
                    0: i_pixel = 8'd200;
                    1: i_pixel = (c < IMG_W/2) ? 8'd0 : 8'd255;
                    2: i_pixel = (r < IMG_H/2) ? 8'd0 : 8'd255;
                    3: begin int v3=(r+c)*8; i_pixel=(v3>255)?8'd255:8'(v3); end
                    4: i_pixel = 8'd128;
                    5: i_pixel = ((r+c)%2==0) ? 8'd255 : 8'd0;
                    default: i_pixel = 8'd0;
                endcase
                i_hsync=(c==IMG_W-1); i_vsync=(r==IMG_H-1 && c==IMG_W-1);
                @(posedge clk); #1;
                if (o_valid && g_cnt<BUF_SZ) begin g_grad[g_cnt]=o_gradient; g_edge[g_cnt]=o_edge; g_cnt++; end
                i_hsync=0; i_vsync=0;
            end
        end
        i_valid=0;
        for (int d=0; d<WARMUP; d++) begin
            @(posedge clk); #1;
            if (o_valid && g_cnt<BUF_SZ) begin g_grad[g_cnt]=o_gradient; g_edge[g_cnt]=o_edge; g_cnt++; end
        end
    endtask

    initial begin
        $dumpfile("tb_sobel_edge_detector.vcd"); $dumpvars(0,tb_sobel_edge_detector);
        $display("╔══════════════════════════════════════════════════╗");
        $display("║  TB: sobel_edge_detector (THRESH=%0d)             ║", THRESH);
        $display("╚══════════════════════════════════════════════════╝");

        // TC-01 Reset
        $display("\n── TC-01 Reset ──────────────────────────────────");
        rst_n=0; i_valid=1; i_pixel=8'hFF;
        repeat(5) @(posedge clk); #1;
        if (o_gradient==0 && o_valid==0 && o_edge==0)
            begin $display("  [PASS] TC-01"); pass_cnt++; end
        else
            begin $display("  [FAIL] TC-01: grad=%0d valid=%b edge=%b", o_gradient,o_valid,o_edge); fail_cnt++; end
        do_reset();

        // TC-02 Flat field → gradient=0 everywhere
        $display("\n── TC-02 Flat Field ──────────────────────────────");
        begin
            int nz2=0;
            stream_frame(0);
            for (int i=0; i<g_cnt; i++) if (g_grad[i]!=0||g_edge[i]!=0) nz2++;
            if (nz2==0 && g_cnt>0)
                begin $display("  [PASS] TC-02: All %0d outputs zero", g_cnt); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-02: %0d non-zero outputs", nz2); fail_cnt++; end
        end
        do_reset();

        // TC-03 Vertical step edge → strong Gx
        $display("\n── TC-03 Vertical Edge ───────────────────────────");
        begin
            int max3=0;
            stream_frame(1);
            for (int r=2; r<IMG_H-2; r++) begin
                for (int c=IMG_W/2-1; c<=IMG_W/2+1; c++) begin
                    int idx3=r*IMG_W+c;
                    if (idx3<g_cnt && int'(g_grad[idx3])>max3) max3=int'(g_grad[idx3]);
                end
            end
            if (max3>=200)
                begin $display("  [PASS] TC-03a: Peak gradient=%0d at edge", max3); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-03a: Peak=%0d (expect ≥200)", max3); fail_cnt++; end
            // Edge flag at transition
            begin
                int efidx = (IMG_H/2)*IMG_W + IMG_W/2;
                if (efidx<g_cnt && g_edge[efidx]==1)
                    begin $display("  [PASS] TC-03b: Edge flag at transition"); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-03b: No edge flag at transition"); fail_cnt++; end
            end
        end
        do_reset();

        // TC-04 Horizontal step edge → strong Gy
        $display("\n── TC-04 Horizontal Edge ─────────────────────────");
        begin
            int max4=0;
            stream_frame(2);
            for (int r=IMG_H/2-1; r<=IMG_H/2+1; r++) begin
                for (int c=2; c<IMG_W-2; c++) begin
                    int idx4=r*IMG_W+c;
                    if (idx4<g_cnt && int'(g_grad[idx4])>max4) max4=int'(g_grad[idx4]);
                end
            end
            if (max4>=200)
                begin $display("  [PASS] TC-04: Peak gradient=%0d at H-edge", max4); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-04: Peak=%0d (expect ≥200)", max4); fail_cnt++; end
        end
        do_reset();

        // TC-05 Diagonal — non-zero gradient in interior
        $display("\n── TC-05 Diagonal Edge ───────────────────────────");
        begin
            int nz5=0;
            stream_frame(3);
            for (int r=2; r<IMG_H-2; r++)
                for (int c=2; c<IMG_W-2; c++) begin
                    int idx5=r*IMG_W+c;
                    if (idx5<g_cnt && g_grad[idx5]!=0) nz5++;
                end
            if (nz5>0)
                begin $display("  [PASS] TC-05: %0d interior pixels have gradient≠0", nz5); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-05: No gradient on diagonal"); fail_cnt++; end
        end
        do_reset();

        // TC-06 Threshold: strong edge → edge=1; flat → edge=0
        $display("\n── TC-06 Threshold ───────────────────────────────");
        begin
            int edges6=0;
            stream_frame(1);   // vertical edge → edge=1
            for (int i=0; i<g_cnt; i++) if (g_edge[i]) edges6++;
            if (edges6>0)
                begin $display("  [PASS] TC-06a: %0d edge pixels on strong step", edges6); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-06a: No edges on strong step"); fail_cnt++; end
        end
        do_reset();
        begin
            int edges6b=0;
            stream_frame(4);   // flat 128 → edge=0
            for (int i=0; i<g_cnt; i++) if (g_edge[i]) edges6b++;
            if (edges6b==0)
                begin $display("  [PASS] TC-06b: 0 edges on flat field"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-06b: %0d spurious edges", edges6b); fail_cnt++; end
        end
        do_reset();

        // TC-07 Clamping — no X/Z in gradient output
        $display("\n── TC-07 Gradient Clamping ───────────────────────");
        begin
            int xz7=0;
            stream_frame(5);   // checkerboard max contrast
            for (int i=0; i<g_cnt; i++)
                if (g_grad[i]===8'hXX || g_grad[i]===8'hZZ) xz7++;
            if (xz7==0)
                begin $display("  [PASS] TC-07: All gradients well-defined"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-07: %0d X/Z values", xz7); fail_cnt++; end
        end
        do_reset();

        // TC-08 Sync — one hsync pulse per row
        $display("\n── TC-08 Sync Propagation ────────────────────────");
        begin
            int hs8=0;
            @(negedge clk); i_valid=1;
            for (int r=0; r<IMG_H; r++) begin
                for (int c=0; c<IMG_W; c++) begin
                    i_pixel=8'd128; i_hsync=(c==IMG_W-1); i_vsync=0;
                    @(posedge clk); #1; if(o_hsync) hs8++; i_hsync=0;
                end
            end
            i_valid=0;
            repeat(WARMUP) begin @(posedge clk); #1; if(o_hsync) hs8++; end
            if (hs8==IMG_H)
                begin $display("  [PASS] TC-08: %0d hsync pulses (1/row)", hs8); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-08: %0d pulses (expected %0d)", hs8, IMG_H); fail_cnt++; end
        end
        do_reset();

        // TC-09 Valid gate
        $display("\n── TC-09 Valid Gating ────────────────────────────");
        begin
            int sp9=0;
            i_valid=0; i_pixel=8'hAA;
            repeat(WARMUP) begin @(posedge clk); #1; if(o_valid) sp9++; end
            if (sp9==0)
                begin $display("  [PASS] TC-09"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-09: %0d spurious o_valid", sp9); fail_cnt++; end
        end
        do_reset();

        // TC-10 Throughput
        $display("\n── TC-10 Throughput ──────────────────────────────");
        begin
            int in10=0, out10=0;
            @(negedge clk); i_valid=1;
            for (int i=0; i<2*IMG_W; i++) begin i_pixel=8'd64; @(posedge clk); #1; if(o_valid) out10++; end
            for (int i=0; i<IMG_W; i++) begin i_pixel=8'd64; in10++; @(posedge clk); #1; if(o_valid) out10++; end
            i_valid=0;
            if (out10>=in10)
                begin $display("  [PASS] TC-10: %0d in → %0d out", in10, out10); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-10: %0d in → %0d out", in10, out10); fail_cnt++; end
        end

        $display("\n══════════════════════════════════════════════════");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        if (fail_cnt==0) $display("  ✅  ALL TESTS PASSED");
        else             $display("  ❌  %0d TEST(S) FAILED", fail_cnt);
        $finish;
    end
    initial begin #10_000_000; $display("[TIMEOUT]"); $finish; end

endmodule : tb_sobel_edge_detector
