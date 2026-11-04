`timescale 1ns/1ps
module tb_bfs;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,start=0;reg [3:0] start_x=3,start_y=3,goal_x=12,goal_y=3;
 wire busy,done,found;wire signed [1:0] step_dx,step_dy;wire [7:0] map_addr;
 reg [1:0] map_cell;reg [1:0] cells[0:255];integer i;
 grid_bfs #(.LOG2(4),.RADIUS(1)) dut(.*);
 always @(posedge clk)map_cell<=cells[map_addr];
 task run;begin @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);end endtask
 initial begin
  for(i=0;i<256;i=i+1)cells[i]=1;
  repeat(3)@(negedge clk);rst=0;run();
  if(!found || step_dx!=1 || step_dy!=0)$fatal(1,"straight route");
  // Wall ahead, opening above. Must take Y detour.
  for(i=0;i<7;i=i+1)cells[i*16+5]=3;
  run();if(!found || step_dx!=0 || step_dy!=1)$fatal(1,"detour or inflation");
  for(i=0;i<16;i=i+1)cells[i*16+5]=3;
  run();if(found)$fatal(1,"solid wall path");
  for(i=0;i<256;i=i+1)cells[i]=0;
  run();if(found)$fatal(1,"unknown free");
  for(i=0;i<256;i=i+1)cells[i]=1;
  goal_x=start_x;goal_y=start_y;run();
  if(!found || step_dx!=0 || step_dy!=0)$fatal(1,"same cell");
  $display("PASS BFS: straight, inflated detour, blocked, unknown, same cell");$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
