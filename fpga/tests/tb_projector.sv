`timescale 1ns/1ps
module tb_projector;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,point_valid=0,pose_fresh=1,ray_ready=1;
 reg [14:0] angle_q6=0;reg [15:0] distance_q2=4000;reg [5:0] quality=20;
 reg signed [31:0] pose_x=0,pose_y=0,pose_yaw=0;
 wire ray_valid;wire signed [15:0] sx,sy,ex,ey;wire [31:0] dropped;
 scan_projector dut(.*);
 task run;begin @(negedge clk);point_valid=1;@(negedge clk);point_valid=0;wait(ray_valid);repeat(4)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=0;run();
  if(sx!=32 || sy!=32 || ex<35 || ex>36 || ey<31 || ey>32)$fatal(1,"forward ray");
  angle_q6=90*64;run();if(ey<27 || ey>28 || ex<31 || ex>32)$fatal(1,"clockwise ray");
  angle_q6=0;pose_yaw=102944;run();if(ey<35 || ey>36 || ex<31 || ex>32)$fatal(1,"yaw rotation");
  pose_x=-65536;pose_y=-65536;pose_yaw=0;run();if(sx!=28 || sy!=28)$fatal(1,"negative pose");
  ray_ready=0;@(negedge clk);point_valid=1;@(negedge clk);point_valid=0;wait(ray_valid);
  repeat(30)begin @(negedge clk);if(!ray_valid)$fatal(1,"ray dropped under backpressure");end
  ray_ready=1;repeat(4)@(negedge clk);if(ray_valid)$fatal(1,"ray did not retire");
  $display("PASS projector: polar transform, clockwise convention, signed pose");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
