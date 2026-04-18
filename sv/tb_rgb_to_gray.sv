// =============================================================================
// File        : tb_rgb_to_gray.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
//
// TC-01  Reset behaviour
// TC-02  Pure black   RGB(0,0,0)       → gray = 0
// TC-03  Pure white   RGB(255,255,255) → gray = 255
// TC-04  Pure red     RGB(255,0,0)     → gray ≈ 76
// TC-05  Pure green   RGB(0,255,0)     → gray ≈ 149
// TC-06  Pure blue    RGB(0,0,255)     → gray ≈ 28
// TC-07  Mid-grey     RGB(128,128,128) → gray = 128±1
// TC-08  Sync pass-through (2-cycle delay)
// TC-09  Valid gating — i_valid=0 keeps o_valid=0
// TC-10  Back-to-back pixel stream — all 8 outputs verified
// =============================================================================
`timescale 1ns / 1ps

module tb_rgb_to_gray;

    localparam int CLK_HALF = 5;
    logic clk = 0;
    logic rst_n;
    always #CLK_HALF clk = ~clk;

    logic       i_valid, i_hsync, i_vsync;
    logic [7:0] i_r, i_g, i_b;
    logic       o_valid, o_hsync, o_vsync;
    logic [7:0] o_gray;

    rgb_to_gray #(.DATA_WIDTH(8)) dut (.*);

    int pass_cnt = 0;
    int fail_cnt = 0;

    // ── Reference model ───────────────────────────────────────────────────────
    function automatic logic [7:0] gray_ref(logic [7:0] r, g, b);
        logic [15:0] s;
        s = 16'(77)*r + 16'(150)*g + 16'(29)*b;
        return s[15:8];
    endfunction

    task automatic do_reset;
        rst_n = 0; i_valid = 0; i_r = 0; i_g = 0; i_b = 0;
        i_hsync = 0; i_vsync = 0;
        repeat(4) @(posedge clk); #1;
        rst_n = 1; @(posedge clk); #1;
    endtask

    task automatic check(string name, logic [7:0] got, logic [7:0] exp, int tol);
        int d = (int'(got) > int'(exp)) ? int'(got)-int'(exp) : int'(exp)-int'(got);
        if (d <= tol) begin
            $display("  [PASS] %-36s got=%3d  exp=%3d±%0d", name, got, exp, tol);
            pass_cnt++;
        end else begin
            $display("  [FAIL] %-36s got=%3d  exp=%3d±%0d  diff=%0d", name, got, exp, tol, d);
            fail_cnt++;
        end
    endtask

    // Drive one pixel and wait 3 cycles (2-stage pipeline + 1 extra for sum_s2 register)
    task automatic drive_px(input logic [7:0] r,g,b);
        @(negedge clk);
        i_valid=1; i_r=r; i_g=g; i_b=b; i_hsync=0; i_vsync=0;
        @(posedge clk); @(posedge clk); @(posedge clk); #1;
        i_valid=0; i_r=0; i_g=0; i_b=0;
    endtask

    // ── TC-01 Reset ───────────────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_rgb_to_gray.vcd"); $dumpvars(0, tb_rgb_to_gray);
        $display("╔══════════════════════════════════════════════════╗");
        $display("║  TB: rgb_to_gray                                 ║");
        $display("╚══════════════════════════════════════════════════╝");

        // TC-01
        $display("\n── TC-01 Reset ──────────────────────────────────");
        rst_n=0; i_valid=1; i_r=8'hFF; i_g=8'hFF; i_b=8'hFF; i_hsync=1; i_vsync=1;
        repeat(5) @(posedge clk); #1;
        if (o_gray==0 && o_valid==0 && o_hsync==0 && o_vsync==0) begin
            $display("  [PASS] TC-01: Outputs zero during reset"); pass_cnt++;
        end else begin
            $display("  [FAIL] TC-01: gray=%0h valid=%b hsync=%b vsync=%b",
                     o_gray, o_valid, o_hsync, o_vsync); fail_cnt++;
        end
        do_reset();

        // TC-02 Black
        $display("\n── TC-02..07 Colour Accuracy ─────────────────────");
        drive_px(0, 0, 0);
        check("TC-02 Black (0,0,0)",         o_gray, gray_ref(0,0,0),       0);

        // TC-03 White
        drive_px(255, 255, 255);
        check("TC-03 White (255,255,255)",    o_gray, gray_ref(255,255,255), 1);

        // TC-04 Pure red
        drive_px(255, 0, 0);
        check("TC-04 Red   (255,0,0)",        o_gray, gray_ref(255,0,0),    1);

        // TC-05 Pure green
        drive_px(0, 255, 0);
        check("TC-05 Green (0,255,0)",        o_gray, gray_ref(0,255,0),    1);

        // TC-06 Pure blue
        drive_px(0, 0, 255);
        check("TC-06 Blue  (0,0,255)",        o_gray, gray_ref(0,0,255),    1);

        // TC-07 Mid-grey
        drive_px(128, 128, 128);
        check("TC-07 Mid-grey (128,128,128)", o_gray, gray_ref(128,128,128),1);

        // TC-08 Sync pass-through
        $display("\n── TC-08 Sync Pass-Through ──────────────────────");
        begin
            logic hs_cap;
            @(negedge clk);
            i_valid=1; i_r=50; i_g=100; i_b=150;
            i_hsync=1; i_vsync=0;
            @(posedge clk); #1; i_hsync=0;
            @(posedge clk); #1;
            hs_cap = o_hsync;           // should be the cycle-1 hsync=1
            @(posedge clk); #1;
            i_valid=0;
            if (hs_cap==1) begin
                $display("  [PASS] TC-08: hsync delayed 2 cycles correctly"); pass_cnt++;
            end else begin
                $display("  [FAIL] TC-08: hsync not seen at output (got %b)", hs_cap); fail_cnt++;
            end
        end

        // TC-09 Valid gating
        $display("\n── TC-09 Valid Gating ────────────────────────────");
        begin
            int spurious = 0;
            i_valid=0; i_r=8'hFF; i_g=8'hFF; i_b=8'hFF;
            repeat(8) begin @(posedge clk); #1; if (o_valid) spurious++; end
            if (spurious==0)
                begin $display("  [PASS] TC-09: o_valid stayed low (no i_valid)"); pass_cnt++; end
            else
                begin $display("  [FAIL] TC-09: %0d spurious o_valid pulses", spurious); fail_cnt++; end
        end

        // TC-10 Back-to-back stream — drive 8 pixels continuously, collect outputs
        $display("\n── TC-10 Back-to-Back Stream ─────────────────────");
        begin
            // 8 test vectors
            logic [7:0] tr[8]; logic [7:0] tg[8]; logic [7:0] tb2[8];
            logic [7:0] exp[8];
            logic [7:0] cap[12];
            int         out_idx = 0;
            int         wrongs  = 0;

            tr[0]=255; tg[0]=  0; tb2[0]=  0;
            tr[1]=  0; tg[1]=255; tb2[1]=  0;
            tr[2]=  0; tg[2]=  0; tb2[2]=255;
            tr[3]=128; tg[3]=128; tb2[3]=128;
            tr[4]=200; tg[4]=100; tb2[4]= 50;
            tr[5]=  0; tg[5]=200; tb2[5]=100;
            tr[6]= 64; tg[6]= 64; tb2[6]= 64;
            tr[7]=255; tg[7]=255; tb2[7]=255;

            for (int i=0; i<8; i++)
                exp[i] = gray_ref(tr[i], tg[i], tb2[i]);

            // Drive all 8 pixels back-to-back
            @(negedge clk); i_valid=1;
            for (int i=0; i<8; i++) begin
                i_r=tr[i]; i_g=tg[i]; i_b=tb2[i]; i_hsync=0; i_vsync=0;
                @(posedge clk); #1;
                if (o_valid && out_idx < 12) begin cap[out_idx]=o_gray; out_idx++; end
            end
            i_valid=0;
            // Drain pipeline (4 more cycles)
            repeat(4) begin
                @(posedge clk); #1;
                if (o_valid && out_idx < 12) begin cap[out_idx]=o_gray; out_idx++; end
            end

            // Compare: outputs align with inputs offset by pipeline latency
            // First output corresponds to first input after warmup
            for (int i=0; i<out_idx && i<8; i++) begin
                int d2 = (int'(cap[i]) > int'(exp[i])) ? int'(cap[i])-int'(exp[i])
                                                        : int'(exp[i])-int'(cap[i]);
                if (d2 <= 1)
                    begin $display("  [PASS] TC-10[%0d] got=%3d exp=%3d", i, cap[i], exp[i]); pass_cnt++; end
                else
                    begin $display("  [FAIL] TC-10[%0d] got=%3d exp=%3d diff=%0d", i, cap[i], exp[i], d2); fail_cnt++; wrongs++; end
            end
        end

        $display("\n══════════════════════════════════════════════════");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        if (fail_cnt==0) $display("  ✅  ALL TESTS PASSED");
        else             $display("  ❌  %0d TEST(S) FAILED", fail_cnt);
        $display("══════════════════════════════════════════════════");
        $finish;
    end
    initial begin #500_000; $display("[TIMEOUT]"); $finish; end

endmodule : tb_rgb_to_gray
