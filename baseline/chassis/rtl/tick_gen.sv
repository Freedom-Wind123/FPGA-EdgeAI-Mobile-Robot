`timescale 1ns/1ps

module tick_gen #(
    parameter integer CLK_HZ = 50_000_000
) (
    input  logic clk,
    input  logic rst,
    output logic tick_1ms,
    output logic tick_10ms,
    output logic tick_20ms
);
    localparam integer CYCLES_PER_MS = CLK_HZ / 1000;
    logic [$clog2(CYCLES_PER_MS)-1:0] div_count;
    logic [4:0] ms_count;

    always_ff @(posedge clk) begin
        if (rst) begin
            div_count <= '0;
            ms_count   <= '0;
            tick_1ms   <= 1'b0;
            tick_10ms  <= 1'b0;
            tick_20ms  <= 1'b0;
        end else begin
            tick_1ms  <= 1'b0;
            tick_10ms <= 1'b0;
            tick_20ms <= 1'b0;
            if (div_count == CYCLES_PER_MS-1) begin
                div_count <= '0;
                tick_1ms  <= 1'b1;
                if (ms_count == 5'd19)
                    ms_count <= '0;
                else
                    ms_count <= ms_count + 1'b1;
                if ((ms_count == 5'd9) || (ms_count == 5'd19))
                    tick_10ms <= 1'b1;
                if (ms_count == 5'd19)
                    tick_20ms <= 1'b1;
            end else begin
                div_count <= div_count + 1'b1;
            end
        end
    end
endmodule
