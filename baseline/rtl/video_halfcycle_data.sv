`timescale 1ns/1ps
// Data-only retiming between a posedge producer and a posedge FIFO consumer.
// Producer data/valid are stable from its posedge through the following
// negedge. Keep the original valid at the consumer: the sampled payload at
// every posedge is unchanged, but RAM data now has half a cycle of hold margin.
// Both half-cycle paths remain timed; no multicycle or false-path exception.
module video_halfcycle_data #(parameter WIDTH=16)(
 input wire clk,reset_n,
 input wire [WIDTH-1:0] data_in,
 output reg [WIDTH-1:0] data_out
);
 always @(negedge clk or negedge reset_n)
  if(!reset_n) data_out <= 0;
  else data_out <= data_in;
endmodule
