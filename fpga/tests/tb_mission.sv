`timescale 1ns/1ps
module tb_mission;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,tick_ms=0,estop=0,healthy=1,packet_valid=0,nav_done=0;
 reg [7:0] kind=0;reg [191:0] payload=0;
 wire armed;wire [1:0] mode;wire signed [31:0] goal_x,goal_y,manual_vx,manual_vy,manual_wz,bearing_q16;
 wire [15:0] target_mm;wire target_fresh,command_fresh,goal_changed,target_changed;wire [31:0] rejected;
 mission_control dut(.*);
 task packet(input [7:0] k,input signed [31:0] a,b,c);begin
  @(negedge clk);kind=k;payload={96'b0,c,b,a};packet_valid=1;
  @(negedge clk);packet_valid=0;repeat(2)@(negedge clk);
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=0;
  packet(8'h13,5000,0,0);if(armed || mode!=0)$fatal(1,"unarmed manual");
  packet(3,0,0,0);packet(8'h13,5000,0,0);if(!armed || mode!=1)$fatal(1,"manual arm");
  tick_ms=1;repeat(305)@(negedge clk);tick_ms=0;
  if(armed || mode!=0)$fatal(1,"manual timeout");
  packet(1,0,0,0);if(armed)$fatal(1,"heartbeat resumed");
  packet(3,0,0,0);packet(8'h11,0,0,0);packet(8'h12,13000,900,1);
  if(mode!=3 || !target_fresh || target_mm!=900)$fatal(1,"follow observation");
  tick_ms=1;repeat(205)@(negedge clk);tick_ms=0;
  if(armed || mode!=0)$fatal(1,"target timeout");
  packet(3,0,0,0);packet(8'h10,65536,0,0);
  tick_ms=1;repeat(400)@(negedge clk);tick_ms=0;
  if(mode!=2 || !armed)$fatal(1,"local map needs RK heartbeat");
  healthy=0;repeat(3)@(negedge clk);healthy=1;repeat(3)@(negedge clk);
  if(armed || mode!=0)$fatal(1,"sensor fault auto resumed");
  packet(3,0,0,0);estop=1;repeat(3)@(negedge clk);
  if(armed)$fatal(1,"estop");
  $display("PASS mission: arm, leases, local autonomy, sensor latch, stop");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
