`timescale 1ns/1ps
module tb_lidar;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,tick_ms=0,rx_valid=0;reg [7:0] rx_data;
 wire point_valid,scan_start,linked;wire [14:0] angle_q6;wire [15:0] distance_q2;wire [5:0] quality;wire [31:0] errors;
 integer count=0;
 c1_scan_parser dut(.*);
 always @(negedge clk)if(point_valid)begin
  count=count+1;
  if(angle_q6!=5760 || distance_q2!=4000 || quality!=15 || !scan_start)$fatal(1,"decoded point");
 end
 task b(input [7:0] v);begin @(negedge clk);rx_data=v;rx_valid=1;@(negedge clk);rx_valid=0;end endtask
 initial begin
  repeat(3)@(negedge clk);rst=0;
  b(8'ha5);b(8'h5a);b(5);b(0);b(0);b(8'h40);b(8'h82);
  if(linked)$fatal(1,"accept wrong scan type");
  b(8'ha5);b(8'ha5);b(8'h5a);b(5);b(0);b(0);b(8'h40);b(8'h81);
  if(!linked)$fatal(1,"descriptor");
  b(0);b(8'h3d);b(1);b(45);b(8'ha0);b(15);
  repeat(3)@(negedge clk);if(count!=1 || errors==0)$fatal(1,"sync handling");
  tick_ms=1;repeat(505)@(negedge clk);tick_ms=0;
  if(linked)$fatal(1,"link timeout");
  $display("PASS C1 descriptor, format, sync recovery, timeout");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
