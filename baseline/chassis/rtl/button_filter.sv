`timescale 1ns/1ps
module button_filter #(parameter integer DEBOUNCE_MS=20)(
 input logic clk,rst,tick_1ms,button_n,
 output logic pressed,down,up,ready
);
 (* syn_preserve=1 *) logic [1:0] sync_n;
 logic candidate; integer stable_ms;
 always_ff @(posedge clk) begin
  down<=0;up<=0;
  if(rst) begin
   sync_n<=2'b11;candidate<=0;stable_ms<=0;pressed<=0;ready<=0;
  end else begin
   sync_n<={sync_n[0],button_n};
   if(tick_1ms) begin
    if(candidate != !sync_n[1]) begin candidate<=!sync_n[1];stable_ms<=1;end
    else if(stable_ms < DEBOUNCE_MS) begin
     stable_ms<=stable_ms+1;
     if(stable_ms==DEBOUNCE_MS-1) begin
      pressed<=candidate;ready<=1;
      if(ready && candidate!=pressed) begin down<=candidate;up<=!candidate;end
     end
    end
   end
  end
 end
endmodule
