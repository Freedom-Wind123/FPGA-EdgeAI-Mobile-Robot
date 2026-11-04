`timescale 1ns/1ps
module tb_guard;
 reg enable=0,estop=0,pose_fresh=1,scan_fresh=1,command_fresh=1;
 reg [127:0] ranges_mm={8{16'd1500}};reg [7:0] valid_mask=255;
 reg signed [31:0] req_vx=30000,req_vy=0,req_wz=0;
 wire signed [31:0] vx,vy,wz;wire motor_enable;wire [3:0] reason;
 motion_guard dut(.*);
 initial begin
  #1;if(vx!=0 || motor_enable)$fatal(1,"boot inhibition");
  enable=1;#1;if(vx!=9830 || !motor_enable)$fatal(1,"speed cap");
  ranges_mm[15:0]=800;#1;if(vx!=2457 || reason!=6)$fatal(1,"slowdown");
  ranges_mm[15:0]=500;#1;if(vx!=0 || motor_enable || reason!=5)$fatal(1,"obstacle stop");
  ranges_mm={8{16'd1500}};pose_fresh=0;#1;if(motor_enable || reason!=2)$fatal(1,"stale pose");
  pose_fresh=1;scan_fresh=0;#1;if(motor_enable || reason!=3)$fatal(1,"stale scan");
  scan_fresh=1;command_fresh=0;#1;if(motor_enable || reason!=4)$fatal(1,"stale cmd");
  command_fresh=1;req_vx=0;req_wz=10000;valid_mask=8'h7f;#1;if(motor_enable)$fatal(1,"rotation unknown octant");
  valid_mask=255;req_wz=0;req_vy=-5000;ranges_mm[47:32]=300;#1;if(motor_enable)$fatal(1,"right obstacle");
  ranges_mm={8{16'd1500}};estop=1;#1;if(motor_enable)$fatal(1,"estop");
  $display("PASS guard: inhibit, cap, slow, stop, stale data, rotation, lateral");$finish;
 end
endmodule
