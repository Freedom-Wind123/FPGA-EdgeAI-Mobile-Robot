`timescale 1ns/1ps
module tb_servo;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,start=0;reg signed [31:0] pose_x=0,pose_y=0,pose_yaw=0,goal_x=65536,goal_y=0;
 wire busy,done,reached;wire signed [31:0] vx,vy,wz;
 waypoint_servo dut(.*);
 task run;begin @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=0;run();
  if(vx<6500 || vy>10 || vy< -10)$fatal(1,"world X yaw zero");
  pose_yaw=102944;run();if(vy> -6500 || vx>10 || vx< -10)$fatal(1,"body transform 90 deg");
  goal_x=0;run();if(!reached || vx!=0 || vy!=0)$fatal(1,"arrival");
  goal_y=-65536;pose_yaw=0;run();if(vy> -6500)$fatal(1,"negative direction");
  $display("PASS waypoint: bounded control, yaw transform, arrival");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
