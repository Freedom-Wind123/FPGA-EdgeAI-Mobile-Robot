// Eight clockwise octants: front, front-right, right, ..., front-left.
// Publish only complete revolutions. Invalid/no echo is NOT free space.
module c1_sectors #(parameter MIN_POINTS=4)(
 input logic clk,rst,tick_ms,point_valid,scan_start,
 input logic [14:0] angle_q6,input logic [15:0] distance_q2,
 input logic [5:0] quality,
 output logic [127:0] ranges_mm,output logic [7:0] valid_mask,
 output logic fresh,scan_valid
);
 logic [15:0] minima[0:7];logic [7:0] counts[0:7];
 logic begun;logic [9:0] age;integer i;
 wire usable=quality!=0 && angle_q6<23040 && distance_q2>=200 && distance_q2<=48000;
 // Parallel constant windows avoid an integer divider and variable RAM index.
 wire [7:0] sector_sel;
 assign sector_sel[0]=angle_q6<1440 || (angle_q6>=21600 && angle_q6<23040);
 genvar g;
 generate for(g=1;g<8;g=g+1)begin:octant
  assign sector_sel[g]=angle_q6>=(g*2880-1440) && angle_q6<(g*2880+1440);
 end endgenerate
 assign fresh=begun && age<250 && (&valid_mask);
 always_ff @(posedge clk) begin
  scan_valid<=0;
  if(rst) begin
   begun<=0;age<=1000;ranges_mm<=0;valid_mask<=0;
   for(i=0;i<8;i=i+1) begin minima[i]<=16'hffff;counts[i]<=0;end
  end else begin
   if(tick_ms && age<1000) age<=age+1'b1;
   if(point_valid) begin
    if(scan_start) begin
     if(begun) begin
      for(i=0;i<8;i=i+1) begin
       ranges_mm[i*16+:16]<=minima[i];valid_mask[i]<=counts[i]>=MIN_POINTS;
      end
      age<=0;scan_valid<=1;
     end
     begun<=1;
     for(i=0;i<8;i=i+1) begin
      minima[i]<= (usable && sector_sel[i])?(distance_q2>>2):16'hffff;
      counts[i]<= (usable && sector_sel[i])?1:0;
     end
    end else if(begun && usable) begin
     for(i=0;i<8;i=i+1)if(sector_sel[i])begin
      if((distance_q2>>2)<minima[i]) minima[i]<=distance_q2>>2;
      if(counts[i]!=255) counts[i]<=counts[i]+1'b1;
     end
    end
   end
  end
 end
endmodule
