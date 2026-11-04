// Visible target -> world goal with 0.7 m standoff. Q16 SI throughout.
module target_goal(
 input logic clk,rst,start,input logic signed [31:0] pose_x,pose_y,pose_yaw,bearing,
 input logic [15:0] range_mm,
 output logic valid,busy,output logic signed [31:0] x,y
);
 logic [2:0] state;logic trig_start,trig_valid,trig_busy;
 logic signed [31:0] px,py,angle,radius,s,c;
 logic signed [16:0] travel_mm;
 logic signed [63:0] dx,dy;
 cordic_sincos trig(.clk,.rst,.start(trig_start),.angle_q16(angle),.sin_q16(s),.cos_q16(c),.valid(trig_valid),.busy(trig_busy));
 always_ff @(posedge clk)begin
  valid<=0;trig_start<=0;
  if(rst)begin state<=0;busy<=0;x<=0;y<=0;px<=0;py<=0;angle<=0;radius<=0;travel_mm<=0;dx<=0;dy<=0;end
  else case(state)
   0:if(start)begin
    px<=pose_x;py<=pose_y;angle<=pose_yaw+bearing;
    travel_mm<=range_mm>700?$signed({1'b0,range_mm})-17'sd700:17'sd0;
    busy<=1;state<=1;
   end
   1:begin radius<=($signed(travel_mm)*48'sd4294967)>>>16;
     if(angle>205887)angle<=angle-411775;
     else if(angle< -205887)angle<=angle+411775;
     else begin trig_start<=1;state<=2;end
    end
   2:if(trig_valid)begin dx<=$signed(radius)*$signed(c);dy<=$signed(radius)*$signed(s);state<=3;end
   3:begin x<=px+(dx>>>16);y<=py+(dy>>>16);valid<=1;busy<=0;state<=0;end
   default:state<=0;
  endcase
 end
endmodule
