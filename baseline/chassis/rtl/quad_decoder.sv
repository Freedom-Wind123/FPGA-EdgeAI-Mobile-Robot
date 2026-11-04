`timescale 1ns/1ps

module quad_decoder #(
    parameter bit REVERSE = 1'b0,
    parameter integer CLK_HZ = 50_000_000
) (
    input  logic               clk,
    input  logic               rst,
    input  logic               phase_a,
    input  logic               phase_b,
    output logic signed [31:0] count
);
    localparam integer SAMPLE_DIV = CLK_HZ / 1_000_000;
    logic [$clog2(SAMPLE_DIV)-1:0] sample_count;
    logic [2:0] a_sync, b_sync;
    logic [2:0] a_history, b_history;
    logic a_filtered, b_filtered;
    logic [1:0] previous;
    logic [1:0] current;
    logic signed [1:0] step;

    always_comb begin
        current = {a_filtered, b_filtered};
        case ({previous, current})
            // STM32 TI12, non-inverted inputs: RM0090 Table 97.
            // phase_a = old timer TI1, phase_b = TI2 (not always harness A/B).
            // TI1 leads TI2: 00 -> 10 -> 11 -> 01 -> 00 increments.
            4'b0001, 4'b0111, 4'b1110, 4'b1000: step = REVERSE ? 2'sd1 : -2'sd1;
            4'b0010, 4'b1011, 4'b1101, 4'b0100: step = REVERSE ? -2'sd1 : 2'sd1;
            default: step = 2'sd0;
        endcase
    end

    always_ff @(posedge clk) begin
        a_sync <= {a_sync[1:0], phase_a};
        b_sync <= {b_sync[1:0], phase_b};
        if (rst) begin
            sample_count <= '0;
            a_sync       <= '0;
            b_sync       <= '0;
            a_history    <= '0;
            b_history    <= '0;
            a_filtered   <= 1'b0;
            b_filtered   <= 1'b0;
            previous     <= 2'b00;
            count        <= '0;
        end else if (sample_count == SAMPLE_DIV-1) begin
            sample_count <= '0;
            a_history <= {a_history[1:0], a_sync[2]};
            b_history <= {b_history[1:0], b_sync[2]};
            if (&a_history) a_filtered <= 1'b1;
            if (~|a_history) a_filtered <= 1'b0;
            if (&b_history) b_filtered <= 1'b1;
            if (~|b_history) b_filtered <= 1'b0;
            if (current != previous) begin
                count    <= count + step;
                previous <= current;
            end
        end else begin
            sample_count <= sample_count + 1'b1;
        end
    end
endmodule
