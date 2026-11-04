`timescale 1ns/1ps
`include "chassis_config.svh"

// One wheel's STM32-equivalent 100 Hz speed loop.
module wheel_pid (
    input  logic               clk,
    input  logic               rst,
    input  logic               tick_10ms,
    input  logic signed [31:0] target_q16,
    input  logic signed [31:0] encoder_count,
    output logic signed [31:0] speed_q16,
    output logic signed [31:0] ramp_target_q16,
    output logic signed [15:0] pwm_signed
);
    logic signed [31:0] last_count;
    logic signed [31:0] last_bias;
    logic signed [31:0] last_last_bias;

    logic signed [31:0] delta_count;
    logic signed [63:0] speed_product;
    logic signed [31:0] speed_next;
    logic signed [31:0] ramp_next;
    logic signed [31:0] bias_next;
    logic signed [63:0] pid_q16;
    logic signed [63:0] control_q16;
    logic signed [31:0] control_next;
    logic signed [31:0] abs_speed;
    logic [4:0] stage;
    logic signed [31:0] target_hold;
    logic signed [63:0] p_error, i_error, d_error, d_partial;
    logic signed [63:0] p_term, i_term, d_term, pi_sum;

    function automatic signed [31:0] trunc_q16(input logic signed [63:0] value);
        logic signed [63:0] positive;
        begin
            if (value < 0) begin
                positive  = -value;
                trunc_q16 = -(positive >>> 16);
            end else begin
                trunc_q16 = value >>> 16;
            end
        end
    endfunction

    function automatic signed [31:0] q24_to_q16(input logic signed [63:0] value);
        logic signed [63:0] positive;
        begin
            if (value < 0) begin
                positive = -value;
                q24_to_q16 = -(positive >>> 8);
            end else begin
                q24_to_q16 = value >>> 8;
            end
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            last_count        <= encoder_count;
            speed_q16         <= 0;
            ramp_target_q16   <= 0;
            last_bias         <= 0;
            last_last_bias    <= 0;
            pwm_signed        <= 0;
            stage<=0; target_hold<=0;delta_count<=0;speed_product<=0;speed_next<=0;
            ramp_next<=0;bias_next<=0;p_error<=0;i_error<=0;d_error<=0;d_partial<=0;
            p_term<=0;i_term<=0;d_term<=0;pi_sum<=0;pid_q16<=0;control_q16<=0;control_next<=0;abs_speed<=0;
        end else case(stage)
        0: if(tick_10ms) begin
            // Capture exactly on the original 10ms tick; hold the sample during arithmetic.
            delta_count<=encoder_count-last_count;last_count<=encoder_count;
            target_hold<=target_q16;stage<=1;
        end
        1: begin
            speed_product<=delta_count*32'sd1075463;
            if((target_hold-ramp_target_q16)>`RAMP_STEP_Q16)
                ramp_next<=ramp_target_q16+`RAMP_STEP_Q16;
            else if((ramp_target_q16-target_hold)>`RAMP_STEP_Q16)
                ramp_next<=ramp_target_q16-`RAMP_STEP_Q16;
            else ramp_next<=target_hold;
            stage<=2;
        end
        2: begin speed_next<=q24_to_q16(speed_product);stage<=3;end
        3: begin bias_next<=ramp_next-speed_next;abs_speed<=(speed_next<0)?-speed_next:speed_next;stage<=4;end
        4: begin
            // Keep the original expression's signed 64-bit context, including shifted history.
            p_error<=$signed(bias_next)-$signed(last_bias);
            i_error<=$signed(bias_next);
            d_partial<=$signed(bias_next)-($signed({{32{last_bias[31]}},last_bias})<<<1);
            stage<=5;
        end
        5: begin
            d_error<=d_partial+$signed(last_last_bias);
            p_term<=`PID_KP*p_error;i_term<=`PID_KI*i_error;stage<=6;
        end
        6: begin d_term<=`PID_KD*d_error;pi_sum<=p_term+i_term;stage<=7;end
        7: begin pid_q16<=pi_sum+d_term;stage<=8;end
        8: begin control_q16<=($signed({{48{pwm_signed[15]}},pwm_signed})<<<16)+pid_q16;stage<=9;end
        9: begin control_next<=trunc_q16(control_q16);stage<=10;end
        10: begin
            speed_q16<=speed_next;ramp_target_q16<=ramp_next;
            if ((ramp_next == 0) && (abs_speed < `STOP_EPS_Q16)) begin
                last_bias      <= 0;
                last_last_bias <= 0;
                pwm_signed     <= 0;
            end else begin
                last_last_bias <= last_bias;
                last_bias      <= bias_next;
                if(control_next>`PWM_MAX) pwm_signed<=`PWM_MAX;
                else if(control_next< -`PWM_MAX) pwm_signed<=-`PWM_MAX;
                else pwm_signed<=control_next[15:0];
            end
            stage<=0;
        end
        default: stage<=0;
        endcase
    end
endmodule
