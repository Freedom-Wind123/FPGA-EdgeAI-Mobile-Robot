`timescale 1ns/1ps
module tb_telemetry_equivalence;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,start=0,out_ready=1;
 reg signed [31:0] value0,value1,value2,value3,value4,value5,value6,value7,value8,value9;
 wire [7:0] out_data,ref_data;wire out_valid,busy,ref_valid,ref_busy;
 reg [7:0] actual[0:255],expected[0:255];integer a=0,b=0,i,j,n;reg [31:0] rng=32'h194270ab;
 telemetry_formatter dut(.*);
 telemetry_formatter_ref refdut(.clk,.rst,.start,.out_ready,
  .value0,.value1,.value2,.value3,.value4,.value5,.value6,.value7,.value8,.value9,
  .out_data(ref_data),.out_valid(ref_valid),.busy(ref_busy));
 always @(posedge clk)begin
  if(!rst && out_valid && out_ready)begin actual[a]=out_data;a=a+1;end
  if(!rst && ref_valid && out_ready)begin expected[b]=ref_data;b=b+1;end
 end
 function [31:0] next_random(input [31:0] s);begin next_random={s[30:0],s[31]^s[21]^s[1]^s[0]};end endfunction
 initial begin
  value0=0;value1=1;value2=-1;value3=65535;value4=-65535;value5=32'h80000000;
  value6=32'h7fffffff;value7=65536;value8=-65536;value9=123456;
  repeat(3)@(negedge clk);rst=0;
  for(n=0;n<32;n=n+1)begin
   a=0;b=0;@(negedge clk);start=1;@(negedge clk);start=0;
   while(busy || ref_busy)begin
    @(negedge clk);rng=next_random(rng);out_ready=rng[0]|rng[3];
   end
   if(a!=b)$fatal(1,"length mismatch %d %d",a,b);
   for(j=0;j<a;j=j+1)if(actual[j]!==expected[j])$fatal(1,"byte mismatch frame %d pos %d",n,j);
   rng=next_random(rng);value0=rng;rng=next_random(rng);value1=rng;
   rng=next_random(rng);value2=rng;rng=next_random(rng);value3=rng;
   rng=next_random(rng);value4=rng;rng=next_random(rng);value5=rng;
   rng=next_random(rng);value6=rng;rng=next_random(rng);value7=rng;
   rng=next_random(rng);value8=rng;rng=next_random(rng);value9=rng;
  end
  $display("PASS telemetry equivalence: edge cases + 31 varied frames under backpressure");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
