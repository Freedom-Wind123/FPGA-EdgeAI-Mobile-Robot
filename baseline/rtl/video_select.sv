`timescale 1ns/1ps
module video_select(
 input wire pclk,pixel_clk,capture_reset_n,display_reset_n,
 input wire cam_vsync,write_valid,write_full,test_pattern,display_req,read_empty,
 input wire [11:0] x,input wire [23:0] camera_rgb,
 output reg [23:0] pixel_rgb,output wire read_enable,output wire error
);
 reg frame_has_data,frame_ready,vs_d,write_error;
 always @(posedge pclk or negedge capture_reset_n) begin
  if(!capture_reset_n) begin frame_has_data<=0; frame_ready<=0; vs_d<=0; write_error<=0; end
  else begin
   vs_d<=cam_vsync;
   if(write_valid) frame_has_data<=1;
   if(write_valid && write_full) write_error<=1;
   if(cam_vsync && !vs_d && frame_has_data) frame_ready<=1;
  end
 end
 (* syn_preserve=1 *) reg [1:0] frame_sync,test_sync,error_sync;
 reg read_error;
 always @(posedge pixel_clk or negedge display_reset_n) begin
  if(!display_reset_n) begin frame_sync<=0; test_sync<=0; error_sync<=0; read_error<=0; end
  else begin
   frame_sync<={frame_sync[0],frame_ready}; test_sync<={test_sync[0],test_pattern};
   error_sync<={error_sync[0],write_error};
   if(frame_sync[1] && display_req && read_empty) read_error<=1;
  end
 end
 // Keep consuming the FIFO in color-bar mode to preserve display address alignment.
 assign read_enable=display_req && !read_empty;
 assign error=read_error || error_sync[1];
 always @* begin
  pixel_rgb=0;
  if(test_sync[1]) begin
   if(x<160) pixel_rgb=24'hffffff;
   else if(x<320) pixel_rgb=24'hffff00;
   else if(x<480) pixel_rgb=24'h00ffff;
   else if(x<640) pixel_rgb=24'h00ff00;
   else if(x<800) pixel_rgb=24'hff00ff;
   else if(x<960) pixel_rgb=24'hff0000;
   else if(x<1120) pixel_rgb=24'h0000ff;
   else pixel_rgb=24'h000000;
  end else if(frame_sync[1] && !read_empty) pixel_rgb=camera_rgb;
 end
endmodule
