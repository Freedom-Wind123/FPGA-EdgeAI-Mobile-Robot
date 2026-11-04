`timescale 1ns/1ps
module reset_release(input wire clk,async_n,output wire reset_n);
 (* syn_preserve=1 *) reg [2:0] release_pipe=0;
 always @(posedge clk or negedge async_n)
  if(!async_n) release_pipe<=0;
  else release_pipe<={release_pipe[1:0],1'b1};
 assign reset_n=release_pipe[2];
endmodule
