`timescale 1ns/1ps
`include "chassis_config.svh"

module chassis_boardkey_top (
    input  logic clk_50m,

    // Onboard CH340E, used for the unchanged ROS text protocol.
    input  logic ros_uart_rx,
    output logic ros_uart_tx,

    // External WT901BCTTL.
    input  logic imu_uart_rx,
    output logic imu_uart_tx,

    input  logic key_mode_n, // Onboard S1, A20; pressed low.
    input  logic key_stop_n, // Onboard S0, F15; reset/disable, pressed low.
    output logic [3:0] led,  // ACTIVE HIGH, schematic sheet 10.

    input  logic enc_a_phase_a,
    input  logic enc_a_phase_b,
    input  logic enc_b_phase_a,
    input  logic enc_b_phase_b,
    input  logic enc_c_phase_a,
    input  logic enc_c_phase_b,
    input  logic enc_d_phase_a,
    input  logic enc_d_phase_b,

    output logic motor_a_pwm,
    output logic motor_a_in1,
    output logic motor_a_in2,
    output logic motor_b_pwm,
    output logic motor_b_in1,
    output logic motor_b_in2,
    output logic motor_c_pwm,
    output logic motor_c_in1,
    output logic motor_c_in2,
    output logic motor_d_pwm,
    output logic motor_d_in1,
    output logic motor_d_in2,
    output logic motor_stby
);
    logic [15:0] por_shift = 16'hffff;
    logic rst;
    always_ff @(posedge clk_50m) por_shift <= {por_shift[14:0], 1'b0};
    (* syn_preserve=1 *) logic [1:0] stop_release = 0;
    always_ff @(posedge clk_50m or negedge key_stop_n)
        if(!key_stop_n) stop_release <= 0;
        else stop_release <= {stop_release[0],1'b1};
    assign rst = por_shift[15] || !stop_release[1] || !key_stop_n;
    assign motor_stby = ~rst;

    logic tick_1ms, tick_10ms, tick_20ms;
    tick_gen #(.CLK_HZ(`CLK_HZ)) ticks(.clk(clk_50m), .rst, .tick_1ms, .tick_10ms, .tick_20ms);

    logic [7:0] ros_rx_data, imu_rx_data;
    logic ros_rx_valid, imu_rx_valid;
    uart_rx #(.CLK_HZ(`CLK_HZ),.BAUD(`UART_BAUD)) ros_rx(
        .clk(clk_50m),.rst,.rx(ros_uart_rx),.data(ros_rx_data),.valid(ros_rx_valid));
    uart_rx #(.CLK_HZ(`CLK_HZ),.BAUD(`UART_BAUD)) imu_rx(
        .clk(clk_50m),.rst,.rx(imu_uart_rx),.data(imu_rx_data),.valid(imu_rx_valid));

    logic signed [31:0] ros_vx, ros_vy, ros_wz;
    logic ros_command_valid;
    ros_command_parser command_parser(
        .clk(clk_50m),.rst,.rx_data(ros_rx_data),.rx_valid(ros_rx_valid),
        .vx_q16(ros_vx),.vy_q16(ros_vy),.wz_q16(ros_wz),.command_valid(ros_command_valid));

    logic signed [31:0] enc_a, enc_b, enc_c, enc_d;
    quad_decoder #(.REVERSE(`ENCODER_A_REVERSE),.CLK_HZ(`CLK_HZ)) qdec_a(.clk(clk_50m),.rst,.phase_a(enc_a_phase_a),.phase_b(enc_a_phase_b),.count(enc_a));
    quad_decoder #(.REVERSE(`ENCODER_B_REVERSE),.CLK_HZ(`CLK_HZ)) qdec_b(.clk(clk_50m),.rst,.phase_a(enc_b_phase_a),.phase_b(enc_b_phase_b),.count(enc_b));
    quad_decoder #(.REVERSE(`ENCODER_C_REVERSE),.CLK_HZ(`CLK_HZ)) qdec_c(.clk(clk_50m),.rst,.phase_a(enc_c_phase_a),.phase_b(enc_c_phase_b),.count(enc_c));
    quad_decoder #(.REVERSE(`ENCODER_D_REVERSE),.CLK_HZ(`CLK_HZ)) qdec_d(.clk(clk_50m),.rst,.phase_a(enc_d_phase_a),.phase_b(enc_d_phase_b),.count(enc_d));

    logic signed [31:0] key_ta, key_tb, key_tc, key_td;
    logic key_target_valid, imu_zero_request;
    logic key_pressed, key_down, key_up, key_ready;
    logic [2:0] mode_index;
    logic test_active;
    button_filter key_filter(.clk(clk_50m),.rst,.tick_1ms,.button_n(key_mode_n),
        .pressed(key_pressed),.down(key_down),.up(key_up),.ready(key_ready));
    assign led = rst ? 4'b0 : {test_active,mode_index};
    key_controller keys(
        .clk(clk_50m),.rst,.tick_1ms,.key_pressed,.key_down,.key_up,.key_ready,
        .target_a_q16(key_ta),.target_b_q16(key_tb),.target_c_q16(key_tc),.target_d_q16(key_td),
        .target_valid(key_target_valid),.imu_zero_request,.mode_index,.test_active);

    logic signed [31:0] speed_a, speed_b, speed_c, speed_d;
    logic signed [15:0] pwm_a, pwm_b, pwm_c, pwm_d;
    chassis_controller controller(
        .clk(clk_50m),.rst,.tick_1ms,.tick_10ms,
        .ros_vx_q16(ros_vx),.ros_vy_q16(ros_vy),.ros_wz_q16(ros_wz),.ros_command_valid,
        .key_target_a_q16(key_ta),.key_target_b_q16(key_tb),.key_target_c_q16(key_tc),.key_target_d_q16(key_td),.key_target_valid,
        .encoder_a(enc_a),.encoder_b(enc_b),.encoder_c(enc_c),.encoder_d(enc_d),
        .speed_a_q16(speed_a),.speed_b_q16(speed_b),.speed_c_q16(speed_c),.speed_d_q16(speed_d),
        .pwm_a,.pwm_b,.pwm_c,.pwm_d);

    motor_pwm #(.REVERSE(`MOTOR_A_REVERSE),.CLK_HZ(`CLK_HZ),.PWM_HZ(`PWM_HZ)) out_a(.clk(clk_50m),.rst,.command(pwm_a),.pwm(motor_a_pwm),.in1(motor_a_in1),.in2(motor_a_in2));
    motor_pwm #(.REVERSE(`MOTOR_B_REVERSE),.CLK_HZ(`CLK_HZ),.PWM_HZ(`PWM_HZ)) out_b(.clk(clk_50m),.rst,.command(pwm_b),.pwm(motor_b_pwm),.in1(motor_b_in1),.in2(motor_b_in2));
    motor_pwm #(.REVERSE(`MOTOR_C_REVERSE),.CLK_HZ(`CLK_HZ),.PWM_HZ(`PWM_HZ)) out_c(.clk(clk_50m),.rst,.command(pwm_c),.pwm(motor_c_pwm),.in1(motor_c_in1),.in2(motor_c_in2));
    motor_pwm #(.REVERSE(`MOTOR_D_REVERSE),.CLK_HZ(`CLK_HZ),.PWM_HZ(`PWM_HZ)) out_d(.clk(clk_50m),.rst,.command(pwm_d),.pwm(motor_d_pwm),.in1(motor_d_in1),.in2(motor_d_in2));

    logic signed [31:0] imu_roll, imu_pitch, imu_yaw, imu_gx, imu_gy, imu_gz;
    logic imu_updated;
    logic imu_pending;
    logic [31:0] imu_checksum_errors;
    imu_parser imu_protocol(
        .clk(clk_50m),.rst,.tick_1ms,.rx_data(imu_rx_data),.rx_valid(imu_rx_valid),.zero_request(imu_zero_request),
        .roll_q16(imu_roll),.pitch_q16(imu_pitch),.yaw_q16(imu_yaw),
        .gyro_x_q16(imu_gx),.gyro_y_q16(imu_gy),.gyro_z_q16(imu_gz),
        .attitude_updated(imu_updated),.checksum_errors(imu_checksum_errors));

    // The STM32 flag remained set until the 20 ms odometry task consumed it.
    always_ff @(posedge clk_50m) begin
        if (rst)
            imu_pending <= 1'b0;
        else begin
            if (tick_20ms && imu_pending)
                imu_pending <= 1'b0;
            if (imu_updated)
                imu_pending <= 1'b1;
        end
    end

    logic [7:0] imu_tx_data;
    logic imu_tx_start, imu_tx_busy, imu_zero_busy;
    imu_zero_tx zero_sender(.clk(clk_50m),.rst,.tick_1ms,.request(imu_zero_request),.uart_busy(imu_tx_busy),.uart_data(imu_tx_data),.uart_start(imu_tx_start),.busy(imu_zero_busy));
    uart_tx #(.CLK_HZ(`CLK_HZ),.BAUD(`UART_BAUD)) imu_tx(.clk(clk_50m),.rst,.data(imu_tx_data),.start(imu_tx_start),.tx(imu_uart_tx),.busy(imu_tx_busy));

    logic signed [31:0] odom_x, odom_y, odom_th, odom_vx, odom_vy, odom_wz;
    logic signed [31:0] odom_roll, odom_pitch, odom_wx, odom_wy;
    logic odom_updated;
    odometry odom(
        .clk(clk_50m),.rst,.tick_20ms,.encoder_a(enc_a),.encoder_b(enc_b),.encoder_c(enc_c),.encoder_d(enc_d),
        .imu_roll_q16(imu_roll),.imu_pitch_q16(imu_pitch),.imu_yaw_q16(imu_yaw),
        .imu_gx_q16(imu_gx),.imu_gy_q16(imu_gy),.imu_gz_q16(imu_gz),.imu_updated(imu_pending),.zero_request(imu_zero_request),
        .x_q16(odom_x),.y_q16(odom_y),.th_q16(odom_th),.vx_q16(odom_vx),.vy_q16(odom_vy),.wz_q16(odom_wz),
        .roll_q16(odom_roll),.pitch_q16(odom_pitch),.wx_q16(odom_wx),.wy_q16(odom_wy),.updated(odom_updated));

    logic [7:0] telemetry_data;
    logic telemetry_valid, telemetry_busy;
    logic ros_tx_busy;
    telemetry_formatter telemetry(
        .clk(clk_50m),.rst,.start(odom_updated && !telemetry_busy),
        .value0(odom_x),.value1(odom_y),.value2(odom_th),.value3(odom_vx),.value4(odom_vy),
        .value5(odom_wz),.value6(odom_roll),.value7(odom_pitch),.value8(odom_wx),.value9(odom_wy),
        .out_ready(!ros_tx_busy),.out_data(telemetry_data),.out_valid(telemetry_valid),.busy(telemetry_busy));
    uart_tx #(.CLK_HZ(`CLK_HZ),.BAUD(`UART_BAUD)) ros_tx(
        .clk(clk_50m),.rst,.data(telemetry_data),.start(telemetry_valid && !ros_tx_busy),.tx(ros_uart_tx),.busy(ros_tx_busy));
endmodule
