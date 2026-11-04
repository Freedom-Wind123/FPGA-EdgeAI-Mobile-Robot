// This block only listens. No output feeds a chassis control input.
// Angles/velocities are the exact ASCII telemetry values, not recomputed data.
module osd_observer #(
 parameter integer CLK_HZ=50000000, BAUD=115200,
 parameter integer MS_CYCLES=CLK_HZ/1000
)(
 input wire clk, reset_n, telemetry_tx, host_rx, imu_rx,
 input wire [3:0] key_led, input wire gpio_selected,
 output wire [810:0] snapshot, output wire publish
);
 wire [7:0] tb,hb,ib;wire tv,hv,iv,tf,command_valid;
 wire [799:0] fields;
 uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) tr(.clk(clk),.rst(!reset_n),.rx(telemetry_tx),.data(tb),.valid(tv));
 uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) hr(.clk(clk),.rst(!reset_n),.rx(host_rx),.data(hb),.valid(hv));
 uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) ir(.clk(clk),.rst(!reset_n),.rx(imu_rx),.data(ib),.valid(iv));
 osd_telemetry_rx text_rx(.clk(clk),.reset_n(reset_n),.byte_valid(tv),.byte_data(tb),.fields(fields),.frame_valid(tf));
 ros_command_parser command_rx(.clk(clk),.rst(!reset_n),.rx_data(hb),.rx_valid(hv),
  .vx_q16(),.vy_q16(),.wz_q16(),.command_valid(command_valid));
 reg [$clog2(MS_CYCLES+1)-1:0] ms_count;
 wire tick_ms=(ms_count==MS_CYCLES-1);
 reg [6:0] refresh_ms;
 assign publish=tick_ms && refresh_ms==99;
 reg [9:0] ros_age,imu_age,tlm_age;
 reg ros_seen,imu_seen,tlm_seen;
 wire [1:0] ros_status=!ros_seen?0:ros_age<500?1:2;
 wire [1:0] imu_status=!imu_seen?0:imu_age<500?1:2;
 wire [1:0] tlm_status=!tlm_seen?0:tlm_age<500?1:2;
 assign snapshot={tlm_status,imu_status,ros_status,gpio_selected,key_led,fields};
 // IMU freshness requires an actual checksum-valid 0x53 attitude frame.
 reg [3:0] imu_index;
 reg [7:0] imu_sum,imu_type;
 reg [5:0] imu_gap;
 wire good_angle=iv && imu_index==10 && ib==imu_sum && imu_type==8'h53;
 always @(posedge clk or negedge reset_n) begin
  if(!reset_n) begin
   ms_count<=0;refresh_ms<=0;ros_age<=999;imu_age<=999;tlm_age<=999;
   ros_seen<=0;imu_seen<=0;tlm_seen<=0;imu_index<=0;imu_sum<=0;imu_type<=0;imu_gap<=0;
  end else begin
   if(tick_ms) begin
    ms_count<=0;refresh_ms<=refresh_ms==99?0:refresh_ms+1'b1;
    if(ros_age<999)ros_age<=ros_age+1'b1;
    if(imu_age<999)imu_age<=imu_age+1'b1;
    if(tlm_age<999)tlm_age<=tlm_age+1'b1;
    if(imu_index!=0) begin
     if(imu_gap>=49)begin imu_index<=0;imu_gap<=0;end
     else imu_gap<=imu_gap+1'b1;
    end
   end else ms_count<=ms_count+1'b1;
   if(command_valid)begin ros_seen<=1;ros_age<=0;end
   if(tf)begin tlm_seen<=1;tlm_age<=0;end
   if(good_angle)begin imu_seen<=1;imu_age<=0;end
   if(iv)begin
    imu_gap<=0;
    if(imu_index==0)begin
     if(ib==8'h55)begin imu_index<=1;imu_sum<=8'h55;end
    end else if(imu_index==1)begin
     if(ib>=8'h50 && ib<=8'h5f)begin imu_type<=ib;imu_sum<=imu_sum+ib;imu_index<=2;end
     else imu_index<=0;
    end else if(imu_index<10)begin imu_sum<=imu_sum+ib;imu_index<=imu_index+1'b1;end
    else imu_index<=0;
   end
  end
 end
endmodule
