// This guard is downstream of ALL planners/manual commands. Units Q16 SI.
// Conservative circular envelope. Thresholds must be measured for the chassis.
module motion_guard #(parameter STOP_MM=600,SLOW_MM=1000)(
 input logic enable,estop,pose_fresh,scan_fresh,command_fresh,
 input logic [127:0] ranges_mm,input logic [7:0] valid_mask,
 input logic signed [31:0] req_vx,req_vy,req_wz,
 output logic signed [31:0] vx,vy,wz,output logic motor_enable,
 output logic [3:0] reason
);
 logic [7:0] need,close_mask,slow_mask;logic close_hit,slow_hit;integer i;
 function automatic signed [31:0] limit(input signed [31:0] v,input signed [31:0] cap);
  if(v>cap) limit=cap;else if(v< -cap)limit=-cap;else limit=v;
 endfunction
 always_comb begin
  need=0;
  if(req_vx>0)need=need|8'b10000011;
  if(req_vx<0)need=need|8'b00111000;
  if(req_vy>0)need=need|8'b11100000;
  if(req_vy<0)need=need|8'b00001110;
  if(req_wz!=0)need=8'hff;
  for(i=0;i<8;i=i+1) begin
   close_mask[i]=!valid_mask[i] || ranges_mm[i*16+:16]<=STOP_MM;
   slow_mask[i]=ranges_mm[i*16+:16]<SLOW_MM;
  end
  close_hit=|(need & close_mask);slow_hit=|(need & slow_mask);
  vx=0;vy=0;wz=0;motor_enable=0;reason=0;
  if(!enable || estop) reason=1;
  else if(!pose_fresh)reason=2;
  else if(!scan_fresh)reason=3;
  else if(!command_fresh)reason=4;
  else if(close_hit)reason=5;
  else begin
   // Per-axis 0.15 m/s => diagonal speed <= 0.213 m/s. Rotation <= 0.3 rad/s.
   vx=limit(req_vx,9830);vy=limit(req_vy,9830);wz=limit(req_wz,19661);
   motor_enable=1;
   if(slow_hit) begin vx=vx>>>2;vy=vy>>>2;wz=wz>>>2;reason=6;end
  end
 end
endmodule
