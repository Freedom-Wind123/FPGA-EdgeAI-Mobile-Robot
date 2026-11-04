`timescale 1ns/1ps
`include "chassis_config.svh"

module chassis_controller (
    input  logic               clk,
    input  logic               rst,
    input  logic               tick_1ms,
    input  logic               tick_10ms,
    input  logic signed [31:0] ros_vx_q16,
    input  logic signed [31:0] ros_vy_q16,
    input  logic signed [31:0] ros_wz_q16,
    input  logic               ros_command_valid,
    input  logic signed [31:0] key_target_a_q16,
    input  logic signed [31:0] key_target_b_q16,
    input  logic signed [31:0] key_target_c_q16,
    input  logic signed [31:0] key_target_d_q16,
    input  logic               key_target_valid,
    input  logic signed [31:0] encoder_a,
    input  logic signed [31:0] encoder_b,
    input  logic signed [31:0] encoder_c,
    input  logic signed [31:0] encoder_d,
    output logic signed [31:0] speed_a_q16,
    output logic signed [31:0] speed_b_q16,
    output logic signed [31:0] speed_c_q16,
    output logic signed [31:0] speed_d_q16,
    output logic signed [15:0] pwm_a,
    output logic signed [15:0] pwm_b,
    output logic signed [15:0] pwm_c,
    output logic signed [15:0] pwm_d
);
    logic signed [31:0] target_a, target_b, target_c, target_d;
    logic signed [31:0] ramp_a, ramp_b, ramp_c, ramp_d;
    logic [9:0] command_age_ms;
    logic command_seen;

    logic signed [63:0] vx_scaled, vy_scaled, wz_scaled;
    logic signed [31:0] ros_target_a, ros_target_b, ros_target_c, ros_target_d;

    always_comb begin
        vx_scaled = $signed(ros_vx_q16) * `INV_WHEEL_CIRC_Q16;
        vy_scaled = $signed(ros_vy_q16) * `INV_WHEEL_CIRC_Q16;
        wz_scaled = $signed(ros_wz_q16) * `L_OVER_CIRC_Q16;
        ros_target_a = (vx_scaled - vy_scaled - wz_scaled) >>> 16;
        ros_target_b = (vx_scaled + vy_scaled - wz_scaled) >>> 16;
        ros_target_c = (vx_scaled + vy_scaled + wz_scaled) >>> 16;
        ros_target_d = (vx_scaled - vy_scaled + wz_scaled) >>> 16;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            target_a       <= 0;
            target_b       <= 0;
            target_c       <= 0;
            target_d       <= 0;
            command_age_ms <= 0;
            command_seen   <= 1'b0;
        end else begin
            if (key_target_valid) begin
                target_a <= key_target_a_q16;
                target_b <= key_target_b_q16;
                target_c <= key_target_c_q16;
                target_d <= key_target_d_q16;
            end
            if (ros_command_valid) begin
                target_a       <= ros_target_a;
                target_b       <= ros_target_b;
                target_c       <= ros_target_c;
                target_d       <= ros_target_d;
                command_age_ms <= 0;
                command_seen   <= 1'b1;
            end else if (tick_1ms && command_seen) begin
                if (command_age_ms < 10'd999)
                    command_age_ms <= command_age_ms + 1'b1;
                if (command_age_ms >= 10'd499) begin
                    target_a <= 0;
                    target_b <= 0;
                    target_c <= 0;
                    target_d <= 0;
                end
            end
        end
    end

    wheel_pid pid_a(.clk, .rst, .tick_10ms, .target_q16(target_a), .encoder_count(encoder_a), .speed_q16(speed_a_q16), .ramp_target_q16(ramp_a), .pwm_signed(pwm_a));
    wheel_pid pid_b(.clk, .rst, .tick_10ms, .target_q16(target_b), .encoder_count(encoder_b), .speed_q16(speed_b_q16), .ramp_target_q16(ramp_b), .pwm_signed(pwm_b));
    wheel_pid pid_c(.clk, .rst, .tick_10ms, .target_q16(target_c), .encoder_count(encoder_c), .speed_q16(speed_c_q16), .ramp_target_q16(ramp_c), .pwm_signed(pwm_c));
    wheel_pid pid_d(.clk, .rst, .tick_10ms, .target_q16(target_d), .encoder_count(encoder_d), .speed_q16(speed_d_q16), .ramp_target_q16(ramp_d), .pwm_signed(pwm_d));
endmodule
