// =============================================================================
// File        : mac_core.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : Clean 16x16 MAC core (trusted 3PIP reference)
// Target      : Artix-7, Vivado 2024.1
// =============================================================================
`timescale 1ns / 1ps

module mac_core #(
    parameter int DATA_WIDTH = 16,
    parameter int ACC_WIDTH  = 32
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
    logic [ACC_WIDTH-1:0]    acc_next;
    logic                    overflow_reg;

    // Sign extension helper: produce 33-bit sign-extended sum
    // For DATA_WIDTH=16, ACC_WIDTH=32: product is 32 bits already so no extension needed
    always_comb begin : comb_logic
        logic signed [ACC_WIDTH:0] sext_product;
        logic signed [ACC_WIDTH:0] sext_acc;
        product     = a * b;
        sext_product = $signed({{1{product[2*DATA_WIDTH-1]}}, product});
        sext_acc     = $signed({acc_out[ACC_WIDTH-1], acc_out});
        acc_ext      = sext_acc + sext_product;
        if (acc_ext[ACC_WIDTH] != acc_ext[ACC_WIDTH-1])
            acc_next = acc_ext[ACC_WIDTH] ? {1'b1, {(ACC_WIDTH-1){1'b0}}}
                                          : {1'b0, {(ACC_WIDTH-1){1'b1}}};
        else
            acc_next = acc_ext[ACC_WIDTH-1:0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_out      <= '0;
            valid_out    <= 1'b0;
            overflow_reg <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (acc_clr) begin
                acc_out      <= '0;
                overflow_reg <= 1'b0;
            end else if (valid_in) begin
                acc_out      <= acc_next;
                overflow_reg <= (acc_ext[ACC_WIDTH] != acc_ext[ACC_WIDTH-1]);
            end
        end
    end

    assign overflow = overflow_reg;
endmodule
