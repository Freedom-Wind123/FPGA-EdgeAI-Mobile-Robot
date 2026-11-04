// Snapshot pose per accepted ray. 64x64 map, 0.25 m/cell, origin at (32,32).
// C1 raw angle is clockwise; chassis yaw is counterclockwise.
// Extrinsics must be calibrated before hardware release.
module scan_projector #(
 parameter signed [31:0] YAW_OFFSET_Q16=0,
 parameter signed [31:0] X_OFFSET_Q16=0,Y_OFFSET_Q16=0
)(
 input logic clk,rst,point_valid,pose_fresh,ray_ready,
 input logic [14:0] angle_q6,input logic [15:0] distance_q2,
 input logic [5:0] quality,
 input logic signed [31:0] pose_x,pose_y,pose_yaw,
 output logic ray_valid,output logic signed [15:0] sx,sy,ex,ey,
 output logic [31:0] dropped
);
 logic [3:0] state;logic start,valid,busy;
 logic signed [31:0] angle,sin_v,cos_v,px,py,range_q16;
 logic signed [63:0] angle_product,dx,dy;
 cordic_sincos trig(.clk,.rst,.start,.angle_q16(angle),.sin_q16(sin_v),.cos_q16(cos_v),.valid,.busy);
 // Valid stays asserted until the map accepts this ray; planning may stall it.
 assign ray_valid=(state==5);
 always_ff @(posedge clk) begin
  start<=0;
  if(rst)begin state<=0;dropped<=0;angle<=0;px<=0;py<=0;range_q16<=0;
   sx<=0;sy<=0;ex<=0;ey<=0;angle_product<=0;dx<=0;dy<=0;end
  else begin
   if(point_valid && state!=0)dropped<=dropped+1'b1;
   case(state)
    0:if(point_valid && pose_fresh && quality!=0 && distance_q2>=200 && distance_q2<=48000)begin
      angle_product<=$signed({1'b0,angle_q6})*64'sd1171271;
      px<=pose_x+X_OFFSET_Q16;py<=pose_y+Y_OFFSET_Q16;
      angle<=pose_yaw+YAW_OFFSET_Q16;
      // 16.384 * raw_q2. Constant multiply avoids a long combinational /1000.
      // Error <= 1 Q16 LSB over the supported 0.05..12 m range.
      range_q16<=($signed({1'b0,distance_q2})*48'sd1073742)>>>16;
      state<=1;
     end
    1:begin angle<=angle-(angle_product>>>16);state<=2;end
    2:begin
     if(angle>205887)angle<=angle-411775;
     else if(angle< -205887)angle<=angle+411775;
     else begin start<=1;state<=3;end
    end
    3:if(valid)begin dx<=$signed(range_q16)*$signed(cos_v);dy<=$signed(range_q16)*$signed(sin_v);state<=4;end
    4:begin
     sx<=(px>>>14)+32;sy<=(py>>>14)+32;
     ex<=((px+(dx>>>16))>>>14)+32;ey<=((py+(dy>>>16))>>>14)+32;
     state<=5;
    end
    5:if(ray_ready)state<=6;
    6:state<=0;
    default:state<=0;
   endcase
  end
 end
endmodule
