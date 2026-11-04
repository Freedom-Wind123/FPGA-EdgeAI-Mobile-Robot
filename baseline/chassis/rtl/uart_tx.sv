`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
) (
    input  logic       clk,
    input  logic       rst,
    input  logic [7:0] data,
    input  logic       start,
    output logic       tx,
    output logic       busy
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer CW           = $clog2(CLKS_PER_BIT+1);
    logic [CW-1:0] count;
    logic [3:0] bit_index;
    logic [9:0] frame;

    always_ff @(posedge clk) begin
        if (rst) begin
            tx        <= 1'b1;
            busy      <= 1'b0;
            count     <= '0;
            bit_index <= '0;
            frame     <= 10'h3ff;
        end else if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                frame     <= {1'b1, data, 1'b0};
                tx        <= 1'b0;
                count     <= CLKS_PER_BIT-1;
                bit_index <= 0;
                busy      <= 1'b1;
            end
        end else if (count == 0) begin
            if (bit_index == 4'd9) begin
                tx   <= 1'b1;
                busy <= 1'b0;
            end else begin
                bit_index <= bit_index + 1'b1;
                tx        <= frame[bit_index+1'b1];
                count     <= CLKS_PER_BIT-1;
            end
        end else begin
            count <= count - 1'b1;
        end
    end
endmodule
