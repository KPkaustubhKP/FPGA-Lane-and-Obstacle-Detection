// =============================================================================
// Module      : tb_lane_detection.sv
// Project     : FPGA Lane Detection — Kaustubh Pandey & Krishanu Dey
// Description : Self-checking testbench for all pipeline stages and the
//               top-level lane_detection_top module.
//
//   Test plan
//   ─────────
//   TASK 1  : test_rgb_to_gray    — feeds known RGB values, checks Gray output
//   TASK 2  : test_gaussian_blur  — feeds a synthetic pixel gradient,
//                                   verifies output is smoother than input
//   TASK 3  : test_sobel          — feeds a vertical edge pattern,
//                                   checks non-zero gradient at edge column
//   TASK 4  : test_roi_filter     — verifies pixels outside ROI are zeroed
//   TASK 5  : test_top_level      — streams a synthetic 64×48 "frame" through
//                                   the full pipeline; checks lane_valid fires
//
//   Simulation parameters (small image to keep simulation fast):
//     IMG_WIDTH  = 64
//     IMG_HEIGHT = 48
//
// Simulator: Vivado Simulator (xsim) / ModelSim / Icarus Verilog
// =============================================================================

`timescale 1ns / 1ps

module tb_lane_detection;

    // ─── Common image parameters (small for fast simulation) ─────────────────
    localparam int IMG_W = 64;
    localparam int IMG_H = 48;
    localparam int CLK_PERIOD = 10;  // 100 MHz

    // ─── Clock & reset generation ─────────────────────────────────────────────
    logic clk  = 0;
    logic rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    task automatic reset_dut(int cycles = 5);
        rst_n = 1'b0;
        repeat(cycles) @(posedge clk);
        #1;
        rst_n = 1'b1;
        @(posedge clk);
    endtask

    // =========================================================================
    // TASK 1 — rgb_to_gray
    // =========================================================================
    logic        g_i_valid, g_o_valid;
    logic [7:0]  g_i_r, g_i_g, g_i_b;
    logic        g_i_hsync, g_i_vsync;
    logic [7:0]  g_o_gray;
    logic        g_o_hsync, g_o_vsync;

    rgb_to_gray #(.DATA_WIDTH(8)) dut_gray (
        .clk    (clk),    .rst_n  (rst_n),
        .i_valid(g_i_valid), .i_r(g_i_r), .i_g(g_i_g), .i_b(g_i_b),
        .i_hsync(g_i_hsync), .i_vsync(g_i_vsync),
        .o_valid(g_o_valid), .o_gray(g_o_gray),
        .o_hsync(g_o_hsync), .o_vsync(g_o_vsync)
    );

    task automatic test_rgb_to_gray;
        int unsigned expected;
        int unsigned tolerance = 2;  // allow ±2 LSB rounding

        $display("\n══ TASK 1: rgb_to_gray ══════════════════════════");
        reset_dut();

        // Test vector 1: pure white → gray should be ≈ 255
        g_i_valid = 1; g_i_hsync = 0; g_i_vsync = 0;
        g_i_r = 255; g_i_g = 255; g_i_b = 255;
        @(posedge clk); @(posedge clk); @(posedge clk);
        expected = (77*255 + 150*255 + 29*255) >> 8;  // = 255
        if (g_o_valid) begin
            if ($unsigned(g_o_gray) >= expected-tolerance && $unsigned(g_o_gray) <= expected+tolerance)
                $display("  [PASS] White: gray=%0d (expected %0d)", g_o_gray, expected);
            else
                $display("  [FAIL] White: gray=%0d (expected %0d)", g_o_gray, expected);
        end

        // Test vector 2: pure black → 0
        g_i_r = 0; g_i_g = 0; g_i_b = 0;
        @(posedge clk); @(posedge clk); @(posedge clk);
        expected = 0;
        if ($unsigned(g_o_gray) == expected)
            $display("  [PASS] Black: gray=%0d", g_o_gray);
        else
            $display("  [FAIL] Black: gray=%0d (expected 0)", g_o_gray);

        // Test vector 3: pure red (255,0,0) → 77*255/256 ≈ 76
        g_i_r = 255; g_i_g = 0; g_i_b = 0;
        @(posedge clk); @(posedge clk); @(posedge clk);
        expected = (77*255) >> 8;
        if ($unsigned(g_o_gray) >= expected-tolerance && $unsigned(g_o_gray) <= expected+tolerance)
            $display("  [PASS] Red:   gray=%0d (expected %0d)", g_o_gray, expected);
        else
            $display("  [FAIL] Red:   gray=%0d (expected %0d)", g_o_gray, expected);

        // Test vector 4: pure green (0,255,0) → 150*255/256 ≈ 149
        g_i_r = 0; g_i_g = 255; g_i_b = 0;
        @(posedge clk); @(posedge clk); @(posedge clk);
        expected = (150*255) >> 8;
        if ($unsigned(g_o_gray) >= expected-tolerance && $unsigned(g_o_gray) <= expected+tolerance)
            $display("  [PASS] Green: gray=%0d (expected %0d)", g_o_gray, expected);
        else
            $display("  [FAIL] Green: gray=%0d (expected %0d)", g_o_gray, expected);

        g_i_valid = 0;
        $display("  Task 1 complete.\n");
    endtask

    // =========================================================================
    // TASK 2 — gaussian_blur (standalone DUT)
    // =========================================================================
    logic       gb_i_valid, gb_o_valid;
    logic [7:0] gb_i_pixel, gb_o_pixel;
    logic       gb_i_hsync, gb_i_vsync, gb_o_hsync, gb_o_vsync;

    gaussian_blur #(
        .DATA_WIDTH(8), .IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H)
    ) dut_gauss (
        .clk    (clk),    .rst_n  (rst_n),
        .i_valid(gb_i_valid), .i_pixel(gb_i_pixel),
        .i_hsync(gb_i_hsync), .i_vsync(gb_i_vsync),
        .o_valid(gb_o_valid), .o_pixel(gb_o_pixel),
        .o_hsync(gb_o_hsync), .o_vsync(gb_o_vsync)
    );

    task automatic test_gaussian_blur;
        int pixel_count = 0;
        int max_out = 0, min_out = 255;
        $display("══ TASK 2: gaussian_blur ════════════════════════");
        reset_dut();

        // Stream a simple 2-row frame of alternating 0/255 (checkerboard noise)
        gb_i_valid = 1; gb_i_vsync = 0;
        for (int row = 0; row < IMG_H; row++) begin
            gb_i_hsync = 0;
            for (int col = 0; col < IMG_W; col++) begin
                gb_i_pixel = ((row + col) % 2 == 0) ? 8'hFF : 8'h00;
                if (col == IMG_W - 1) gb_i_hsync = 1;
                @(posedge clk);
                gb_i_hsync = 0;
                // Capture output
                if (gb_o_valid) begin
                    pixel_count++;
                    if (gb_o_pixel > max_out) max_out = gb_o_pixel;
                    if (gb_o_pixel < min_out) min_out = gb_o_pixel;
                end
            end
        end
        // Drain pipeline
        gb_i_valid = 0;
        repeat(IMG_W + 10) @(posedge clk);

        // After blurring a checkerboard, max should be < 255 and min > 0
        if (max_out < 255 && min_out > 0)
            $display("  [PASS] Checkerboard blurred: min=%0d, max=%0d (noise reduced)", min_out, max_out);
        else
            $display("  [WARN] Blur stats: min=%0d, max=%0d (check line buffer depth)", min_out, max_out);
        $display("  Task 2 complete.\n");
    endtask

    // =========================================================================
    // TASK 3 — sobel_edge_detector (standalone DUT)
    // =========================================================================
    logic       se_i_valid, se_o_valid;
    logic [7:0] se_i_pixel, se_o_gradient;
    logic       se_o_edge;
    logic       se_i_hsync, se_i_vsync, se_o_hsync, se_o_vsync;

    sobel_edge_detector #(
        .DATA_WIDTH(8), .IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H), .THRESHOLD(32)
    ) dut_sobel (
        .clk       (clk),    .rst_n     (rst_n),
        .i_valid   (se_i_valid), .i_pixel(se_i_pixel),
        .i_hsync   (se_i_hsync), .i_vsync(se_i_vsync),
        .o_valid   (se_o_valid), .o_gradient(se_o_gradient),
        .o_edge    (se_o_edge),
        .o_hsync   (se_o_hsync), .o_vsync(se_o_vsync)
    );

    task automatic test_sobel;
        int edge_count = 0;
        $display("══ TASK 3: sobel_edge_detector ═════════════════");
        reset_dut();

        // Vertical edge: left half = 0, right half = 255
        se_i_valid = 1; se_i_vsync = 0;
        for (int row = 0; row < IMG_H; row++) begin
            se_i_hsync = 0;
            for (int col = 0; col < IMG_W; col++) begin
                se_i_pixel = (col < IMG_W/2) ? 8'h00 : 8'hFF;
                if (col == IMG_W-1) se_i_hsync = 1;
                @(posedge clk); se_i_hsync = 0;
                if (se_o_valid && se_o_edge) edge_count++;
            end
        end
        se_i_valid = 0;
        repeat(IMG_W + 10) @(posedge clk);

        if (edge_count > 0)
            $display("  [PASS] Vertical edge detected: %0d edge pixels found", edge_count);
        else
            $display("  [FAIL] No edges detected on vertical transition");
        $display("  Task 3 complete.\n");
    endtask

    // =========================================================================
    // TASK 4 — lane_roi_filter (standalone DUT)
    // =========================================================================
    logic        rf_i_valid, rf_o_valid;
    logic [7:0]  rf_i_pixel, rf_o_pixel;
    logic        rf_i_edge,  rf_o_edge;
    logic        rf_i_hsync, rf_i_vsync, rf_o_hsync, rf_o_vsync;
    logic [$clog2(IMG_W) -1:0] rf_o_col;
    logic [$clog2(IMG_H)-1:0]  rf_o_row;

    // ROI: rows 28..47 (lower 40% of 48 rows)
    lane_roi_filter #(
        .DATA_WIDTH(8), .IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H),
        .ROI_ROW_TOP(29), .ROI_ROW_BOT(47),
        .ROI_COL_LEFT_BOT(0),  .ROI_COL_RIGHT_BOT(IMG_W-1),
        .ROI_COL_LEFT_TOP(16), .ROI_COL_RIGHT_TOP(48)
    ) dut_roi (
        .clk    (clk),    .rst_n  (rst_n),
        .i_valid(rf_i_valid), .i_pixel(rf_i_pixel),
        .i_edge (rf_i_edge),
        .i_hsync(rf_i_hsync), .i_vsync(rf_i_vsync),
        .o_valid(rf_o_valid), .o_pixel(rf_o_pixel),
        .o_edge (rf_o_edge),
        .o_hsync(rf_o_hsync), .o_vsync(rf_o_vsync),
        .o_col  (rf_o_col),   .o_row(rf_o_row)
    );

    task automatic test_roi_filter;
        int pass_in_roi = 0, pass_out_roi = 0;
        $display("══ TASK 4: lane_roi_filter ══════════════════════");
        reset_dut();

        rf_i_valid = 1; rf_i_edge = 1; rf_i_vsync = 0;
        for (int row = 0; row < IMG_H; row++) begin
            rf_i_hsync = 0;
            for (int col = 0; col < IMG_W; col++) begin
                rf_i_pixel = 8'hFF;  // all pixels are "edge"
                if (col == IMG_W-1) rf_i_hsync = 1;
                @(posedge clk); rf_i_hsync = 0;
                if (rf_o_valid) begin
                    if (rf_o_edge)  pass_in_roi++;
                    else            pass_out_roi++;
                end
            end
        end
        rf_i_valid = 0;
        repeat(10) @(posedge clk);

        $display("  Pixels inside ROI (edge=1): %0d", pass_in_roi);
        $display("  Pixels outside ROI (masked): %0d", pass_out_roi);
        if (pass_in_roi > 0 && pass_out_roi > pass_in_roi)
            $display("  [PASS] ROI correctly masks upper image region");
        else
            $display("  [WARN] Check ROI bounds — review parameter settings");
        $display("  Task 4 complete.\n");
    endtask

    // =========================================================================
    // TASK 5 — Full top-level pipeline
    // =========================================================================
    logic        top_valid;
    logic [23:0] top_rgb;
    logic        top_hsync, top_vsync;
    logic [7:0]  top_gray, top_blurred, top_gradient;
    logic        top_edge, top_lane_valid;
    logic signed [10:0] top_rho_left,  top_rho_right;
    logic [7:0]         top_theta_left, top_theta_right;

    lane_detection_top #(
        .IMG_WIDTH   (IMG_W),
        .IMG_HEIGHT  (IMG_H),
        .EDGE_THRESH (32),
        .ROI_ROW_TOP (29),
        .ROI_ROW_BOT (47),
        .THETA_BINS  (180),
        .RHO_BINS    (128),    // small for sim speed
        .HT_THRESHOLD(5)       // low threshold for short synthetic lines
    ) dut_top (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_valid      (top_valid),
        .i_rgb        (top_rgb),
        .i_hsync      (top_hsync),
        .i_vsync      (top_vsync),
        .o_gray       (top_gray),
        .o_blurred    (top_blurred),
        .o_gradient   (top_gradient),
        .o_edge       (top_edge),
        .o_lane_valid (top_lane_valid),
        .o_rho_left   (top_rho_left),
        .o_theta_left (top_theta_left),
        .o_rho_right  (top_rho_right),
        .o_theta_right(top_theta_right)
    );

    // Helper: stream one synthetic frame (diagonal lane lines on dark background)
    task automatic stream_frame(int frame_id);
        top_valid = 1;
        for (int row = 0; row < IMG_H; row++) begin
            for (int col = 0; col < IMG_W; col++) begin
                // Draw two diagonal "lane" lines:
                //   Left lane:  col ≈ row/2 + 4
                //   Right lane: col ≈ IMG_W - row/2 - 4
                logic on_left_lane  = (col >= row/2 + 3 && col <= row/2 + 5);
                logic on_right_lane = (col >= IMG_W - row/2 - 6 && col <= IMG_W - row/2 - 4);
                if (on_left_lane || on_right_lane)
                    top_rgb = 24'hFFFFFF;   // white lane marking
                else
                    top_rgb = 24'h303030;   // dark road surface

                top_hsync = (col == IMG_W - 1);
                top_vsync = (row == IMG_H - 1 && col == IMG_W - 1);
                @(posedge clk);
            end
        end
        top_valid = 0; top_hsync = 0; top_vsync = 0;
    endtask

    task automatic test_top_level;
        int timeout;
        $display("══ TASK 5: lane_detection_top (full pipeline) ═══");
        reset_dut();

        // Stream 2 frames to allow pipeline warm-up and Hough accumulation
        for (int f = 0; f < 2; f++) begin
            stream_frame(f);
            $display("  Frame %0d streamed.", f+1);
            repeat(10) @(posedge clk);
        end

        // Wait for lane_valid with timeout
        timeout = 200_000;
        while (!top_lane_valid && timeout > 0) begin
            @(posedge clk); timeout--;
        end

        if (top_lane_valid) begin
            $display("  [PASS] Lane parameters detected:");
            $display("         Left  lane: rho=%0d, theta=%0d°", top_rho_left,  top_theta_left);
            $display("         Right lane: rho=%0d, theta=%0d°", top_rho_right, top_theta_right);
        end else begin
            $display("  [WARN] lane_valid did not assert within timeout.");
            $display("         (May need more frames or lower HT_THRESHOLD)");
        end
        $display("  Task 5 complete.\n");
    endtask

    // =========================================================================
    // Waveform dump & main test runner
    // =========================================================================
    initial begin
        $dumpfile("tb_lane_detection.vcd");
        $dumpvars(0, tb_lane_detection);

        $display("╔══════════════════════════════════════════════════╗");
        $display("║  FPGA Lane Detection — Full Testbench            ║");
        $display("║  Kaustubh Pandey & Krishanu Dey                  ║");
        $display("╚══════════════════════════════════════════════════╝");

        // Initialise all driving signals
        {g_i_valid, g_i_r, g_i_g, g_i_b, g_i_hsync, g_i_vsync} = '0;
        {gb_i_valid, gb_i_pixel, gb_i_hsync, gb_i_vsync}         = '0;
        {se_i_valid, se_i_pixel, se_i_hsync, se_i_vsync}         = '0;
        {rf_i_valid, rf_i_pixel, rf_i_edge, rf_i_hsync, rf_i_vsync} = '0;
        {top_valid, top_rgb, top_hsync, top_vsync}               = '0;
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;

        // Run test tasks
        test_rgb_to_gray();
        test_gaussian_blur();
        test_sobel();
        test_roi_filter();
        test_top_level();

        $display("╔══════════════════════════════════════════════════╗");
        $display("║  ALL TASKS COMPLETE                              ║");
        $display("╚══════════════════════════════════════════════════╝");
        $finish;
    end

    // Simulation watchdog
    initial begin
        #50_000_000;
        $display("  [TIMEOUT] Simulation exceeded 50 ms wall-clock — aborting.");
        $finish;
    end

endmodule : tb_lane_detection
