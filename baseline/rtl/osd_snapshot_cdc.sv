// Bundled-data toggle handshake. Data is frozen before the synchronized request
// and until acknowledgement returns. Destination updates ONLY in frame blanking.
// Both domains must share async reset; source/pixel release may be synchronized.
module osd_snapshot_cdc #(parameter WIDTH=811)(
 input wire source_clk,pixel_clk,reset_n,publish,frame_begin,
 input wire [WIDTH-1:0] source_data,
 output reg [WIDTH-1:0] frame_data, output wire busy
);
 reg [WIDTH-1:0] held;
 reg request,ack;
 wire source_reset_n,pixel_reset_n;
 reset_release source_reset(.clk(source_clk),.async_n(reset_n),.reset_n(source_reset_n));
 reset_release pixel_reset(.clk(pixel_clk),.async_n(reset_n),.reset_n(pixel_reset_n));
 (* syn_preserve=1 *) reg [1:0] req_sync,ack_sync;
 assign busy=request!=ack_sync[1];
 always @(posedge source_clk or negedge source_reset_n)begin
  if(!source_reset_n)begin held<=0;request<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(publish && !busy)begin held<=source_data;request<=!request;end
  end
 end
 always @(posedge pixel_clk or negedge pixel_reset_n)begin
  if(!pixel_reset_n)begin frame_data<=0;ack<=0;req_sync<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(frame_begin && req_sync[1]!=ack)begin frame_data<=held;ack<=req_sync[1];end
  end
 end
endmodule
