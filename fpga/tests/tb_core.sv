`timescale 1ns/1ps
module tb_core;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,tick_ms=0,tick_control=0,estop=0;
 reg lidar_rx_valid=0,mission_rx_valid=0;reg [7:0] lidar_rx_data,mission_rx_data;
 reg pose_updated=1,imu_updated=1;
 reg signed [31:0] pose_x=0,pose_y=0,pose_yaw=0;
 wire signed [31:0] vx,vy,wz;wire motor_enable,armed,map_initialized,scan_fresh,route_found,lidar_linked;
 wire [1:0] mode;wire [3:0] stop_reason;wire [31:0] lidar_errors,packet_errors;
 wire [127:0] ranges_mm;wire [7:0] ranges_valid;
 integer i,j,k;reg [15:0] crc;reg [7:0] frame[0:31];
 robot_autonomy_core #(.HARDWARE_CONFIRMED(1)) dut(.*);
 wire signed [31:0] locked_vx,locked_vy,locked_wz;
 wire locked_enable;
 robot_autonomy_core #(.HARDWARE_CONFIRMED(0)) locked(
  .clk,.rst,.tick_ms,.tick_control,.estop,.lidar_rx_valid,.lidar_rx_data,
  .mission_rx_valid,.mission_rx_data,.pose_updated,.imu_updated,.pose_x,.pose_y,.pose_yaw,
  .vx(locked_vx),.vy(locked_vy),.wz(locked_wz),.motor_enable(locked_enable));
 always @(negedge clk)if(!rst && (locked_enable!==0 || locked_vx!==0 || locked_vy!==0 || locked_wz!==0))
  $fatal(1,"development lock bypassed");
 task lb(input [7:0] b);begin @(negedge clk);lidar_rx_data=b;lidar_rx_valid=1;@(negedge clk);lidar_rx_valid=0;end endtask
 task point(input integer a,d,s);integer aq;begin
  aq=(a<<1)|1;lb(s?8'h51:8'h52);lb(aq&255);lb((aq>>8)&255);lb(d&255);lb((d>>8)&255);
 end endtask
 task scan(input integer front);begin
  for(i=0;i<8;i=i+1)for(j=0;j<4;j=j+1)point(i*2880+j,i==0?front:6000,i==0 && j==0);
 end endtask
 task packet(input [7:0] kind,input integer seq,input signed [31:0] p0,p1,p2);
  integer n,b;begin
   for(n=0;n<32;n=n+1)frame[n]=0;
   frame[0]=8'haa;frame[1]=8'h55;frame[2]=1;frame[3]=kind;frame[4]=seq&255;frame[5]=(seq>>8)&255;
   for(n=0;n<4;n=n+1)begin frame[6+n]=(p0>>(8*n))&255;frame[10+n]=(p1>>(8*n))&255;frame[14+n]=(p2>>(8*n))&255;end
   crc=16'hffff;
   for(n=2;n<30;n=n+1)begin crc=crc^{frame[n],8'b0};for(b=0;b<8;b=b+1)crc=crc[15]?(crc<<1)^16'h1021:crc<<1;end
   frame[30]=crc[7:0];frame[31]=crc[15:8];
   for(n=0;n<32;n=n+1)begin @(negedge clk);mission_rx_data=frame[n];mission_rx_valid=1;@(negedge clk);mission_rx_valid=0;end
   repeat(5)@(negedge clk);
  end
 endtask
 initial begin
  repeat(3)@(negedge clk);rst=0;
  packet(3,1,0,0,0);if(armed || motor_enable)$fatal(1,"arm without scan");
  lb(8'ha5);lb(8'h5a);lb(5);lb(0);lb(0);lb(8'h40);lb(8'h81);
  scan(6000);scan(6000);repeat(5)@(negedge clk);
  if(!scan_fresh)$fatal(1,"no fresh scan");
  packet(3,2,0,0,0);packet(8'h13,3,6000,0,0);
  if(vx!=6000 || !motor_enable)$fatal(1,"manual chain %d reason %d",vx,stop_reason);
  scan(1200);point(0,6000,1);repeat(5)@(negedge clk);
  if(motor_enable || vx!=0 || stop_reason!=5)$fatal(1,"collision chain");
  scan(6000);scan(6000);repeat(5)@(negedge clk);
  packet(2,4,0,0,0);if(motor_enable || armed)$fatal(1,"stop chain");
  wait(map_initialized);wait(dut.ray_ready);repeat(30)@(negedge clk);
  for(k=0;k<4096;k=k+1)dut.map.storage.mem[k]=1;
  packet(3,5,0,0,0);packet(8'h11,6,0,0,0);packet(8'h12,7,12000,1000,1);
  tick_control=1;wait(route_found && vx>0);@(negedge clk);
  if(wz<=0)$fatal(1,"follow chain");
  tick_ms=1;repeat(205)@(negedge clk);tick_ms=0;
  if(motor_enable || armed)$fatal(1,"target loss chain");
  // Deterministic known map fixture; ray writing itself is covered in tb_map.
  wait(map_initialized);wait(dut.ray_ready);repeat(30)@(negedge clk);
  for(k=0;k<4096;k=k+1)dut.map.storage.mem[k]=1;
  packet(3,8,0,0,0);packet(8'h10,9,65536,0,0);tick_control=1;
  wait(route_found && vx>0);@(negedge clk);
  if(mode!=2 || !motor_enable)$fatal(1,"map chain");
  // Cancel must invalidate an in-flight path, not resume on planner completion.
  packet(2,10,0,0,0);repeat(100)@(negedge clk);
  if(motor_enable || mode!=0 || vx!=0)$fatal(1,"map cancellation");
  packet(3,11,0,0,0);packet(8'h13,12,6000,0,0);
  if(!motor_enable || vx!=6000)$fatal(1,"manual after map");
  estop=1;#1;
  if(motor_enable || vx!=0 || vy!=0 || wz!=0)$fatal(1,"estop waits for request pipeline");
  repeat(3)@(negedge clk);estop=0;repeat(3)@(negedge clk);
  if(armed || motor_enable)$fatal(1,"estop release resumed old request");
  $display("PASS full core: sensors, CRC, manual, obstacle, follow timeout, map path, cancel, immediate estop, development lock");$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
