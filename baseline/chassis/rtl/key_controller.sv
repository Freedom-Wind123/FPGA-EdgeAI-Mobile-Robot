`timescale 1ns/1ps
`include "chassis_config.svh"
// STM32 key_proc movement targets; debounced release advances the mode.
// Long hold stops; release zeros IMU without clearing the remembered mode.
module key_controller (
 input logic clk,rst,tick_1ms,key_pressed,key_down,key_up,key_ready,
 output logic signed [31:0] target_a_q16,target_b_q16,target_c_q16,target_d_q16,
 output logic target_valid,imu_zero_request,
 output logic [2:0] mode_index,output logic test_active
);
 logic armed,press_seen,long_seen;
 logic [9:0] held_ms;
 task automatic set_targets(input logic [2:0] n);
  begin
   case(n)
    0: begin target_a_q16<=0;target_b_q16<=0;target_c_q16<=0;target_d_q16<=0;end
    1: begin target_a_q16<= `KEY_WHEEL_RPS_Q16;target_b_q16<= `KEY_WHEEL_RPS_Q16;target_c_q16<= `KEY_WHEEL_RPS_Q16;target_d_q16<= `KEY_WHEEL_RPS_Q16;end
    2: begin target_a_q16<=-`KEY_WHEEL_RPS_Q16;target_b_q16<=-`KEY_WHEEL_RPS_Q16;target_c_q16<=-`KEY_WHEEL_RPS_Q16;target_d_q16<=-`KEY_WHEEL_RPS_Q16;end
    3: begin target_a_q16<=-`KEY_WHEEL_RPS_Q16;target_b_q16<= `KEY_WHEEL_RPS_Q16;target_c_q16<= `KEY_WHEEL_RPS_Q16;target_d_q16<=-`KEY_WHEEL_RPS_Q16;end
    4: begin target_a_q16<= `KEY_WHEEL_RPS_Q16;target_b_q16<=-`KEY_WHEEL_RPS_Q16;target_c_q16<=-`KEY_WHEEL_RPS_Q16;target_d_q16<= `KEY_WHEEL_RPS_Q16;end
    5: begin target_a_q16<=-`KEY_WHEEL_RPS_Q16;target_b_q16<=-`KEY_WHEEL_RPS_Q16;target_c_q16<= `KEY_WHEEL_RPS_Q16;target_d_q16<= `KEY_WHEEL_RPS_Q16;end
    6: begin target_a_q16<= `KEY_WHEEL_RPS_Q16;target_b_q16<= `KEY_WHEEL_RPS_Q16;target_c_q16<=-`KEY_WHEEL_RPS_Q16;target_d_q16<=-`KEY_WHEEL_RPS_Q16;end
    default:begin target_a_q16<=0;target_b_q16<=0;target_c_q16<=0;target_d_q16<=0;end
   endcase
   target_valid<=1;test_active<=(n!=0);
  end
 endtask
 always_ff @(posedge clk) begin
  target_valid<=0;imu_zero_request<=0;
  if(rst) begin
   armed<=0;press_seen<=0;held_ms<=0;long_seen<=0;mode_index<=0;test_active<=0;
   target_a_q16<=0;target_b_q16<=0;target_c_q16<=0;target_d_q16<=0;
  end else if(!armed) begin
   // A key held across reset must not create a release-to-run command.
   if(key_ready && !key_pressed) armed<=1;
  end else begin
   if(key_down) begin press_seen<=1;held_ms<=0;long_seen<=0;end
   if(tick_1ms && key_pressed && press_seen) begin
    if(held_ms<999) held_ms<=held_ms+1'b1;
    if(held_ms>=499) begin long_seen<=1;set_targets(0);end
   end
   if(key_up && press_seen) begin
    if(long_seen || held_ms>=500) begin set_targets(0);imu_zero_request<=1;end
    else begin
     mode_index <= (mode_index==6)?0:mode_index+1'b1;
     set_targets((mode_index==6)?3'd0:mode_index+1'b1);
    end
    press_seen<=0;held_ms<=0;long_seen<=0;
   end
  end
 end
endmodule
