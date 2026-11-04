`include "chassis_config.svh"
// Independent, read-only encoder observation. No connection back to control.
// Reuses baseline encoder direction/filter and 10 ms RPS calculation exactly;
// sample phase can differ from the controller, so this is display telemetry.
module osd_wheels(input wire clk,reset_n,input wire [3:0] phase_a,phase_b,
 output wire [319:0] ascii_fields);
 wire tick10,tick20;
 tick_gen ticks(.clk(clk),.rst(!reset_n),.tick_1ms(),.tick_10ms(tick10),.tick_20ms(tick20));
 wire signed [31:0] count[0:3],speed[0:3];
 localparam [3:0] REVERSE={(`ENCODER_D_REVERSE!=0),(`ENCODER_C_REVERSE!=0),(`ENCODER_B_REVERSE!=0),(`ENCODER_A_REVERSE!=0)};
 genvar i;
 generate for(i=0;i<4;i=i+1)begin: wheel
  quad_decoder #(.REVERSE(REVERSE[i]),.CLK_HZ(`CLK_HZ)) decoder(
   .clk(clk),.rst(!reset_n),.phase_a(phase_a[i]),.phase_b(phase_b[i]),.count(count[i]));
  wheel_pid measurement(.clk(clk),.rst(!reset_n),.tick_10ms(tick10),
   .target_q16(32'sd0),.encoder_count(count[i]),.speed_q16(speed[i]),
   .ramp_target_q16(),.pwm_signed());
 end endgenerate
 wire [7:0] data;wire valid;wire [799:0] text_fields;
 telemetry_formatter formatter(.clk(clk),.rst(!reset_n),.start(tick20),
  .value0(speed[0]),.value1(speed[1]),.value2(speed[2]),.value3(speed[3]),
  .value4(32'sd0),.value5(32'sd0),.value6(32'sd0),.value7(32'sd0),.value8(32'sd0),.value9(32'sd0),
  .out_ready(1'b1),.out_data(data),.out_valid(valid),.busy());
 osd_telemetry_rx collect(.clk(clk),.reset_n(reset_n),.byte_valid(valid),.byte_data(data),
  .fields(text_fields),.frame_valid());
 assign ascii_fields=text_fields[319:0];
endmodule
