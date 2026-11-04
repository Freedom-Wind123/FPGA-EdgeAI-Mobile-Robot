// World waypoint -> body-frame velocity, using existing iterative CORDIC.
module waypoint_servo(
 input logic clk,rst,start,
 input logic signed [31:0] pose_x,pose_y,pose_yaw,goal_x,goal_y,
 output logic busy,done,reached,
 output logic signed [31:0] vx,vy,wz
);
 logic [2:0] state;logic trig_start,trig_done,trig_busy;
 logic signed [31:0] yaw,s,c,ex,ey;
 logic signed [63:0] xc,ys,xs,yc;
 function automatic signed [31:0] clip(input signed [32:0] v);
  if(v>6554)clip=6554;else if(v< -6554)clip=-6554;else clip=v;
 endfunction
 cordic_sincos trig(.clk,.rst,.start(trig_start),.angle_q16(yaw),.sin_q16(s),.cos_q16(c),.valid(trig_done),.busy(trig_busy));
 always_ff @(posedge clk)begin
  trig_start<=0;done<=0;
  if(rst)begin busy<=0;state<=0;reached<=0;vx<=0;vy<=0;wz<=0;
   yaw<=0;ex<=0;ey<=0;xc<=0;ys<=0;xs<=0;yc<=0;end
  else case(state)
   0:if(start)begin
    ex<=clip($signed({goal_x[31],goal_x})-$signed({pose_x[31],pose_x}));
    ey<=clip($signed({goal_y[31],goal_y})-$signed({pose_y[31],pose_y}));
    reached<=($signed({goal_x[31],goal_x})-$signed({pose_x[31],pose_x})<3277 &&
              $signed({goal_x[31],goal_x})-$signed({pose_x[31],pose_x})> -3277 &&
              $signed({goal_y[31],goal_y})-$signed({pose_y[31],pose_y})<3277 &&
              $signed({goal_y[31],goal_y})-$signed({pose_y[31],pose_y})> -3277);
    yaw<=pose_yaw;busy<=1;state<=1;
   end
   1:begin trig_start<=1;state<=2;end
   2:if(trig_done)begin xc<=$signed(ex)*$signed(c);ys<=$signed(ey)*$signed(s);
     xs<=$signed(ex)*$signed(s);yc<=$signed(ey)*$signed(c);state<=3;end
   3:begin vx<=reached?0:(xc+ys)>>>16;vy<=reached?0:(yc-xs)>>>16;wz<=0;
     done<=1;busy<=0;state<=0;end
   default:state<=0;
  endcase
 end
endmodule
