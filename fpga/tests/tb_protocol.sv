`timescale 1ns/1ps
module tb_protocol;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,tick_ms=0,rx_valid=0;reg [7:0] rx_data;
 wire valid;wire [7:0] kind;wire [15:0] seq;wire [191:0] payload;wire [31:0] errors;
 reg [7:0] bytes[0:191];integer count=0,i;
 mission_rx dut(.*);
 always @(negedge clk)if(valid)count=count+1;
 task b(input [7:0] v);begin @(negedge clk);rx_data=v;rx_valid=1;@(negedge clk);rx_valid=0;end endtask
 task frame(input integer n);begin for(i=0;i<32;i=i+1)b(bytes[n*32+i]);repeat(2)@(negedge clk);end endtask
 initial begin
  $readmemh("evidence/frames.hex",bytes);
  repeat(3)@(negedge clk);rst=0;
  frame(0);if(count!=1)$fatal(1,"first frame");
  frame(0);if(count!=1)$fatal(1,"replay accepted");
  frame(1);if(count!=1)$fatal(1,"CRC accepted");
  frame(2);if(count!=2)$fatal(1,"new sequence");
  frame(3);if(count!=3 || kind!=2)$fatal(1,"old STOP rejected");
  frame(0);if(count!=3)$fatal(1,"STOP rolled seq backwards");
  b(8'haa);b(8'h55);b(1);tick_ms=1;repeat(25)@(negedge clk);tick_ms=0;
  frame(4);if(count!=4)$fatal(1,"timeout recovery");
  frame(5);if(count!=4)$fatal(1,"unknown version");
  $display("PASS protocol: CRC, version, replay, STOP, timeout recovery");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
