`timescale 1ns/1ps
`include "chassis_config.svh"

module motor_pwm #(
    parameter bit REVERSE = 1'b0,
    parameter integer CLK_HZ = 50_000_000,
    parameter integer PWM_HZ = 1_000
) (
    input  logic               clk,
    input  logic               rst,
    input  logic signed [15:0] command,
    output logic               pwm,
    output logic               in1,
    output logic               in2
);
    // STM32 TIM1 used a 1 MHz counter with ARR=1000: 1001 timer steps.
    localparam integer PERIOD_CLKS = (CLK_HZ / 1_000_000) * 1001;
    localparam integer CLKS_PER_STEP = CLK_HZ / 1_000_000;
    localparam integer CW = $clog2(PERIOD_CLKS);
    logic [CW-1:0] counter;
    logic signed [16:0] corrected;
    logic [15:0] magnitude;
    logic [CW+15:0] threshold_product;
    logic [CW-1:0] threshold;

    always_comb begin
        corrected = REVERSE ? -$signed(command) : $signed(command);
        magnitude = (corrected < 0) ? -corrected : corrected;
        if (magnitude > `PWM_MAX)
            magnitude = `PWM_MAX;
        threshold_product = magnitude * CLKS_PER_STEP;
        threshold = threshold_product;
        if (corrected > 0) begin
            in1 = 1'b1;
            in2 = 1'b0;
        end else if (corrected < 0) begin
            in1 = 1'b0;
            in2 = 1'b1;
        end else begin
            in1 = 1'b0;
            in2 = 1'b0;
        end
        pwm = (counter < threshold) && (corrected != 0);
        // Board-key edition: reset must blank stale PWM immediately.
        if (rst) begin pwm=1'b0;in1=1'b0;in2=1'b0;end
    end

    always_ff @(posedge clk) begin
        if (rst)
            counter <= '0;
        else if (counter == PERIOD_CLKS-1)
            counter <= '0;
        else
            counter <= counter + 1'b1;
    end
endmodule
