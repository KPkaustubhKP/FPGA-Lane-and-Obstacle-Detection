// =============================================================================
// File        : crc16_monitor.sv
// Project     : Supply Chain Security in 3PIP FPGA
// Author      : Kaustubh Pandey (230959054)
// Description : CRC-16/CCITT (polynomial 0x1021) streaming monitor for IP
//               output bitstream integrity. The golden CRC is computed offline
//               (or during a trusted boot phase) and stored in a read-only
//               register. On every CHECK_INTERVAL clock cycles the live CRC
//               is compared against the golden. A mismatch raises crc_alarm.
//
//               Detection target: subtle sequential Trojans that cause
//               systematic LSB-inversions or data drift over time — changes
//               that evade single-cycle TMR but accumulate in the CRC.
//
//               CRC-16/CCITT:
//                 Polynomial : x^16 + x^12 + x^5 + 1  (0x1021)
//                 Init value : 0xFFFF
//                 Bit order  : MSB first
// =============================================================================

`timescale 1ns / 1ps

module crc16_monitor #(
    parameter int CHECK_INTERVAL = 256  // Compare every N valid samples
) (
    input  logic        clk,
    input  logic        rst_n,
    // Streaming data input (byte granularity, upper bits of acc_out)
    input  logic [7:0]  data_in,
    input  logic        valid,
    // Golden CRC (loaded at startup from trusted source)
    input  logic [15:0] golden_crc,
    input  logic        golden_load,    // Pulse to latch golden_crc
    // Outputs
    output logic [15:0] crc_live,       // Running CRC value
    output logic        crc_alarm,      // Mismatch alarm
    output logic        check_pulse     // Indicates a comparison event
);

    // ---- CRC state ----------------------------------------------------------
    logic [15:0] crc_reg;
    logic [15:0] golden_reg;
    logic [$clog2(CHECK_INTERVAL)-1:0] sample_cnt;

    // ---- CRC Update Function (CRC-16/CCITT, MSB-first) ----------------------
    function automatic logic [15:0] crc16_update (
        input logic [15:0] crc,
        input logic [7:0]  data
    );
        logic [15:0] tmp;
        logic        xbit;
        tmp = crc;
        for (int i = 7; i >= 0; i--) begin
            xbit = tmp[15] ^ data[i];
            tmp  = {tmp[14:0], 1'b0};
            if (xbit) tmp ^= 16'h1021;
        end
        return tmp;
    endfunction

    // ---- Golden CRC latch ---------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)         golden_reg <= 16'h0000;
        else if (golden_load) golden_reg <= golden_crc;
    end

    // ---- Streaming CRC computation and periodic check -----------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg     <= 16'hFFFF;
            crc_alarm   <= 1'b0;
            check_pulse <= 1'b0;
            sample_cnt  <= '0;
        end else begin
            check_pulse <= 1'b0;
            if (valid) begin
                crc_reg    <= crc16_update(crc_reg, data_in);
                sample_cnt <= sample_cnt + 1;

                // Periodic check
                if (sample_cnt == (CHECK_INTERVAL - 1)) begin
                    sample_cnt  <= '0;
                    check_pulse <= 1'b1;
                    // Alarm if CRC deviates — resets after golden re-load
                    if (golden_reg != 16'h0000)  // Only alarm after golden is set
                        crc_alarm <= (crc16_update(crc_reg, data_in) != golden_reg);
                end
            end
        end
    end

    assign crc_live = crc_reg;

endmodule
