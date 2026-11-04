// Board-independent, single-clock integration. Byte streams come from UART RX.
// All sensor/motor processing stays in FPGA; RK supplies goal/target packets.
// This core is intentionally NOT a board pin top and cannot directly be flashed.
module robot_autonomy_core #(
 parameter HARDWARE_CONFIRMED=0,STOP_MM=600,SLOW_MM=1000,MAP_RADIUS=2
)(
 input logic clk,rst,tick_ms,tick_control,estop,
 input logic lidar_rx_valid,input logic [7:0] lidar_rx_data,
 input logic mission_rx_valid,input logic [7:0] mission_rx_data,
 input logic pose_updated,imu_updated,
 input logic signed [31:0] pose_x,pose_y,pose_yaw,
 output logic signed [31:0] vx,vy,wz,output logic motor_enable,
 output logic [1:0] mode,output logic armed,
 output logic [3:0] stop_reason,output logic map_initialized,scan_fresh,
 output logic route_found,lidar_linked,output logic [31:0] lidar_errors,packet_errors,
 output logic [127:0] ranges_mm,output logic [7:0] ranges_valid
);
 logic point_valid,scan_start,linked,scan_valid;
 logic [14:0] angle;logic [15:0] distance;logic [5:0] quality;
 assign lidar_linked=linked;
 c1_scan_parser lidar(.clk,.rst,.tick_ms,.rx_valid(lidar_rx_valid),.rx_data(lidar_rx_data),
  .point_valid,.scan_start,.angle_q6(angle),.distance_q2(distance),.quality,.linked,.errors(lidar_errors));
 c1_sectors sectors(.clk,.rst,.tick_ms,.point_valid,.scan_start,.angle_q6(angle),
  .distance_q2(distance),.quality,.ranges_mm,.valid_mask(ranges_valid),.fresh(scan_fresh),.scan_valid);
 logic packet_valid;logic [7:0] kind;logic [15:0] seq;logic [191:0] payload;
 mission_rx packets(.clk,.rst,.tick_ms,.rx_valid(mission_rx_valid),.rx_data(mission_rx_data),
  .valid(packet_valid),.kind,.seq,.payload,.errors(packet_errors));
 logic [9:0] pose_age,imu_age;
 wire pose_fresh=pose_age<100 && imu_age<100 && pose_x>=-524288 && pose_x<524288 && pose_y>=-524288 && pose_y<524288;
 always_ff @(posedge clk)begin
  if(rst)begin pose_age<=1000;imu_age<=1000;end else begin
   if(tick_ms)begin if(pose_age<1000)pose_age<=pose_age+1'b1;if(imu_age<1000)imu_age<=imu_age+1'b1;end
   if(pose_updated)pose_age<=0;if(imu_updated)imu_age<=0;
  end
 end
 logic signed [31:0] goal_x,goal_y,manual_vx,manual_vy,manual_wz,bearing;
 logic [15:0] target_mm;logic target_fresh,command_fresh,goal_changed,nav_done,target_changed;
 logic [31:0] rejected;
 mission_control missions(.clk,.rst,.tick_ms,.estop,.healthy(pose_fresh && scan_fresh),
  .packet_valid,.kind,.payload,.nav_done,.armed,.mode,.goal_x,.goal_y,
  .manual_vx,.manual_vy,.manual_wz,.bearing_q16(bearing),.target_mm,.target_fresh,
  .command_fresh,.goal_changed,.target_changed,.rejected);
 logic visual_goal_valid,visual_goal_busy,visual_goal_seen,visual_replan;
 logic signed [31:0] visual_x,visual_y,follow_x,follow_y;
 logic [1:0] last_mode;
 wire nav_mode=(mode==2 || mode==3);
 wire nav_goal_changed=goal_changed || visual_replan || mode!=last_mode;
 wire signed [31:0] selected_x=mode==3?follow_x:goal_x;
 wire signed [31:0] selected_y=mode==3?follow_y:goal_y;
 target_goal visual_project(.clk,.rst(rst || mode!=3 || !armed),.start(target_changed),
  .pose_x,.pose_y,.pose_yaw,.bearing,.range_mm(target_mm),
  .valid(visual_goal_valid),.busy(visual_goal_busy),.x(visual_x),.y(visual_y));
 always_ff @(posedge clk)begin
  visual_replan<=0;
  if(rst)begin visual_goal_seen<=0;follow_x<=0;follow_y<=0;last_mode<=0;end
  else begin
   last_mode<=mode;
   if(mode!=3 || !armed)visual_goal_seen<=0;
   else if(visual_goal_valid)begin
    if(visual_x< -491520 || visual_x>491520 || visual_y< -491520 || visual_y>491520)begin
     visual_goal_seen<=0;visual_replan<=1;
    end else if(!visual_goal_seen || visual_x-follow_x>9830 || visual_x-follow_x< -9830 ||
                                   visual_y-follow_y>9830 || visual_y-follow_y< -9830)begin
     follow_x<=visual_x;follow_y<=visual_y;visual_goal_seen<=1;visual_replan<=1;
    end
   end
  end
 end
 logic ray_valid,ray_ready;logic signed [15:0] sx,sy,ex,ey;
 logic [31:0] dropped,rays_done;logic freeze,plan_pending,plan_busy,plan_start;
 scan_projector projector(.clk,.rst,.point_valid,.pose_fresh,.ray_ready,
  .angle_q6(angle),.distance_q2(distance),.quality,.pose_x,.pose_y,.pose_yaw,
  .ray_valid,.sx,.sy,.ex,.ey,.dropped);
 logic [11:0] map_addr;logic [1:0] map_cell;
 assign freeze=plan_pending || plan_busy || plan_start;
 grid_map map(.clk,.rst,.freeze,.ray_valid,.sx,.sy,.ex,.ey,.ray_ready,
  .initialized(map_initialized),.rd_addr(map_addr),.rd_cell(map_cell),.rays_done);
 logic plan_done,found;logic signed [1:0] step_dx,step_dy;
 logic [5:0] start_x,start_y,end_x,end_y;
 grid_bfs #(.RADIUS(MAP_RADIUS)) planner(.clk,.rst(rst || nav_goal_changed || !nav_mode || !armed),.start(plan_start),.start_x,.start_y,
  .goal_x(end_x),.goal_y(end_y),.busy(plan_busy),.done(plan_done),.found,
  .step_dx,.step_dy,.map_addr,.map_cell);
 logic have_waypoint,servo_start,servo_busy,servo_done,reached,nav_ready;
 logic signed [31:0] waypoint_x,waypoint_y,nav_vx,nav_vy,nav_wz;
 logic [9:0] retry_age;
 waypoint_servo servo(.clk,.rst(rst || nav_goal_changed || !have_waypoint || !nav_mode),.start(servo_start),.pose_x,.pose_y,.pose_yaw,
  .goal_x(waypoint_x),.goal_y(waypoint_y),.busy(servo_busy),.done(servo_done),.reached,
  .vx(nav_vx),.vy(nav_vy),.wz(nav_wz));
 // Plan only while stationary. Loss of route never substitutes blind motion.
 // Wait for any already accepted ray to finish before freezing the map.
 always_ff @(posedge clk)begin
  plan_start<=0;servo_start<=0;nav_done<=0;
  if(rst)begin
   plan_pending<=0;have_waypoint<=0;route_found<=0;waypoint_x<=0;waypoint_y<=0;nav_ready<=0;
   start_x<=0;start_y<=0;end_x<=0;end_y<=0;retry_age<=0;
  end else begin
   if(tick_ms && retry_age!=0)retry_age<=retry_age-1'b1;
   if(!nav_mode || !armed || nav_goal_changed || (mode==3 && !visual_goal_seen))begin
    have_waypoint<=0;route_found<=0;plan_pending<=0;retry_age<=0;nav_ready<=0;
   end else begin
    if(!have_waypoint && !plan_busy && !plan_pending && !plan_start && !plan_done && ray_ready && !ray_valid && retry_age==0)begin
     start_x<=(pose_x>>>14)+32;start_y<=(pose_y>>>14)+32;
     end_x<=(selected_x>>>14)+32;end_y<=(selected_y>>>14)+32;
     plan_pending<=1;
    end
    if(plan_pending)begin plan_start<=1;plan_pending<=0;end
    if(plan_done)begin
     route_found<=found;
     if(found)begin
      if(step_dx==0 && step_dy==0)begin waypoint_x<=selected_x;waypoint_y<=selected_y;end
      else begin
       waypoint_x<=($signed({1'b0,start_x})+$signed(step_dx)-32)*16384+8192;
       waypoint_y<=($signed({1'b0,start_y})+$signed(step_dy)-32)*16384+8192;
      end
      have_waypoint<=1;nav_ready<=0;
     end else retry_age<=250;
    end
    if(tick_control && have_waypoint && !servo_busy)servo_start<=1;
    if(servo_done && have_waypoint)nav_ready<=1;
    if(servo_done && reached && have_waypoint)begin
     have_waypoint<=0;nav_ready<=0;
     if(start_x==end_x && start_y==end_y && mode==2)nav_done<=1;
     if(mode==3)retry_age<=100;
    end
    if(stop_reason==5 && have_waypoint)begin have_waypoint<=0;route_found<=0;retry_age<=250;nav_ready<=0;end
   end
  end
 end
 logic signed [31:0] req_vx,req_vy,req_wz;
 always_comb begin
  req_vx=0;req_vy=0;req_wz=0;
  if(mode==1)begin req_vx=manual_vx;req_vy=manual_vy;req_wz=manual_wz;end
  if(nav_mode && have_waypoint && nav_ready && route_found && !plan_busy && !nav_goal_changed)begin
   req_vx=nav_vx;req_vy=nav_vy;req_wz=nav_wz;
  end
  if(mode==3 && target_fresh && target_mm!=0 && visual_goal_seen && route_found)begin
   if(bearing>3277 || bearing< -3277)req_wz=bearing>>>1;
  end
 end
 // Separate task/planner selection from sector checks. The guard still sees
 // current estop, freshness and armed state, so a stop never waits for this
 // request pipeline. A mode change flushes the previous mode's request.
 logic signed [31:0] guard_vx,guard_vy,guard_wz;
 always_ff @(posedge clk)begin
  if(rst || estop || !armed || mode!=last_mode || nav_goal_changed)begin
   guard_vx<=0;guard_vy<=0;guard_wz<=0;
  end else begin guard_vx<=req_vx;guard_vy<=req_vy;guard_wz<=req_wz;end
 end
 motion_guard #(.STOP_MM(STOP_MM),.SLOW_MM(SLOW_MM)) guard(
  .enable(HARDWARE_CONFIRMED!=0 && armed && mode!=0),.estop,.pose_fresh,
  .scan_fresh,.command_fresh,.ranges_mm,.valid_mask(ranges_valid),
  .req_vx(guard_vx),.req_vy(guard_vy),.req_wz(guard_wz),.vx,.vy,.wz,.motor_enable,.reason(stop_reason));
endmodule
