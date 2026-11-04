`timescale 1ns/1ps
module tb_map;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,freeze=0,ray_valid=0;reg signed [15:0] sx,sy,ex,ey;
 wire ray_ready,initialized;reg [7:0] rd_addr;wire [1:0] rd_cell;wire [31:0] rays_done;
 grid_map #(.LOG2(4)) dut(.*);
 task cellcheck(input [7:0] a,input [1:0] v);begin rd_addr=a;repeat(3)@(negedge clk);if(rd_cell!=v)$fatal(1,"map cell %d expected %d got %d",a,v,rd_cell);end endtask
 task ray(input integer x0,y0,x1,y1);begin
  wait(ray_ready);@(negedge clk);sx=x0;sy=y0;ex=x1;ey=y1;ray_valid=1;
  @(negedge clk);ray_valid=0;wait(ray_ready);@(negedge clk);
 end endtask
 initial begin
  rd_addr=0;repeat(3)@(negedge clk);rst=0;wait(initialized);
  cellcheck(0,0);ray(2,3,6,3);cellcheck(3*16+2,1);cellcheck(3*16+5,1);cellcheck(3*16+6,3);
  ray(2,3,7,3);cellcheck(3*16+6,2);ray(2,3,7,3);cellcheck(3*16+6,1);
  ray(8,8,12,12);cellcheck(10*16+10,1);cellcheck(12*16+12,3);
  ray(14,4,20,4);cellcheck(4*16+15,1);cellcheck(5*16,0);
  freeze=1;#1;if(ray_ready)$fatal(1,"map not frozen");
  $display("PASS map: unknown, rays, hit decay, diagonal, clipping, freeze");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
