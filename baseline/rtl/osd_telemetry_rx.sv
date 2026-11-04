// Read-only decoder for the existing $x,y,th,vx,vy,wz,roll,pitch,wx,wy CRLF.
// Ten fixed 10-character slots, left aligned; commit only a complete frame.
module osd_telemetry_rx(
 input wire clk, reset_n, byte_valid, input wire [7:0] byte_data,
 output reg [799:0] fields, output reg frame_valid
);
 reg [799:0] pending;
 reg active, wait_lf, dot_seen;
 reg [3:0] field_index, char_index, integers, fractions;
 always @(posedge clk or negedge reset_n) begin
  if(!reset_n) begin
   pending<={100{8'h20}};fields<={100{8'h20}};frame_valid<=0;
   active<=0;wait_lf<=0;dot_seen<=0;field_index<=0;char_index<=0;integers<=0;fractions<=0;
  end else begin
   frame_valid<=0;
   if(byte_valid) begin
    if(byte_data=="$") begin
     pending<={100{8'h20}};active<=1;wait_lf<=0;dot_seen<=0;
     field_index<=0;char_index<=0;integers<=0;fractions<=0;
    end else if(wait_lf) begin
     wait_lf<=0;
     if(byte_data==10) begin fields<=pending;frame_valid<=1;end
    end else if(active) begin
     if(byte_data>= "0" && byte_data<="9" && char_index<10) begin
      pending[(field_index*10+char_index)*8+:8]<=byte_data;char_index<=char_index+1'b1;
      if(dot_seen) fractions<=fractions+1'b1;else integers<=integers+1'b1;
     end else if(byte_data=="-" && char_index==0) begin
      pending[field_index*80+:8]<=byte_data;char_index<=1;
     end else if(byte_data=="." && !dot_seen && integers!=0 && char_index<10) begin
      pending[(field_index*10+char_index)*8+:8]<=byte_data;char_index<=char_index+1'b1;dot_seen<=1;
     end else if(dot_seen && fractions==3 && integers!=0 && byte_data=="," && field_index<9) begin
      field_index<=field_index+1'b1;char_index<=0;integers<=0;fractions<=0;dot_seen<=0;
     end else if(dot_seen && fractions==3 && integers!=0 && byte_data==13 && field_index==9) begin
      active<=0;wait_lf<=1;
     end else active<=0;
    end
   end
  end
 end
endmodule
