"""Generate separate board wrappers; never alter the retained baseline."""
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'baseline'; rtl=ROOT/'fpga/rtl'
s=(base/'chassis/rtl/chassis_top.sv').read_text()
s=s.replace('Onboard CH340E, used for the unchanged ROS text protocol.',
 'Onboard CH340E: binary task RX, retained ten-field text telemetry TX.')
s=s.replace('module chassis_boardkey_top (','module autonomy_chassis_top #(parameter HARDWARE_CONFIRMED=0)(\n    input logic lidar_rx, output logic lidar_tx,')
s=s.replace('assign motor_stby = ~rst;', '')
start=s.index('    logic signed [31:0] ros_vx')
end=s.index('    logic signed [31:0] enc_a',start)
s=s[:start]+s[end:]
start=s.index('    logic signed [31:0] key_ta')
end=s.index('    logic signed [31:0] speed_a',start)
s=s[:start]+'''    // S1 cannot inject wheel targets behind the navigation guard.
    wire imu_zero_request = 1'b0;
    wire nav_motor_enable;
    wire signed [31:0] nav_vx,nav_vy,nav_wz;
    logic nav_enable_q;
    logic signed [31:0] nav_vx_q,nav_vy_q,nav_wz_q;
    always_ff @(posedge clk_50m)begin
      if(rst)begin nav_enable_q<=0;nav_vx_q<=0;nav_vy_q<=0;nav_wz_q<=0;end
      else begin nav_enable_q<=nav_motor_enable;nav_vx_q<=nav_vx;nav_vy_q<=nav_vy;nav_wz_q<=nav_wz;end
    end
    assign motor_stby = nav_motor_enable && nav_enable_q && !rst;
    wire [1:0] nav_mode;
    wire nav_armed;
    wire lidar_valid,lidar_linked;
    wire [7:0] lidar_data;
    assign led = {nav_armed,1'b0,nav_mode};
    c1_uart lidar_port(.clk(clk_50m),.rst,.tick_ms(tick_1ms),.rx(lidar_rx),.tx(lidar_tx),
        .linked(lidar_linked),.rx_valid(lidar_valid),.rx_data(lidar_data));
    robot_autonomy_core #(.HARDWARE_CONFIRMED(HARDWARE_CONFIRMED)) autonomy(.clk(clk_50m),.rst,.tick_ms(tick_1ms),.tick_control(tick_20ms),
        .estop(!key_stop_n),.lidar_rx_valid(lidar_valid),.lidar_rx_data(lidar_data),
        .mission_rx_valid(ros_rx_valid),.mission_rx_data(ros_rx_data),
        .pose_updated(odom_updated),.imu_updated(imu_updated),
        .pose_x(odom_x),.pose_y(odom_y),.pose_yaw(odom_th),
        .vx(nav_vx),.vy(nav_vy),.wz(nav_wz),.motor_enable(nav_motor_enable),
        .mode(nav_mode),.armed(nav_armed),.lidar_linked(lidar_linked));

'''+s[end:]
s=s.replace('.clk(clk_50m),.rst,.tick_1ms,.tick_10ms,', '.clk(clk_50m),.rst(rst || !nav_enable_q),.tick_1ms,.tick_10ms,')
s=s.replace('.ros_vx_q16(ros_vx),.ros_vy_q16(ros_vy),.ros_wz_q16(ros_wz),.ros_command_valid,',
 '.ros_vx_q16(nav_vx_q),.ros_vy_q16(nav_vy_q),.ros_wz_q16(nav_wz_q),.ros_command_valid(tick_1ms),')
s=s.replace('.key_target_a_q16(key_ta),.key_target_b_q16(key_tb),.key_target_c_q16(key_tc),.key_target_d_q16(key_td),.key_target_valid,',
 ".key_target_a_q16(32'sd0),.key_target_b_q16(32'sd0),.key_target_c_q16(32'sd0),.key_target_d_q16(32'sd0),.key_target_valid(1'b0),")
s=s.replace('.rst,.command(pwm_', '.rst(rst || !nav_enable_q),.command(pwm_')
early=[]
for line in s.splitlines():
 if line.strip() in ['logic imu_updated;','logic odom_updated;','logic signed [31:0] odom_x, odom_y, odom_th, odom_vx, odom_vy, odom_wz;']:
  early.append(line)
for line in early:s=s.replace(line+'\n','')
s=s.replace('    // S1 cannot', '\n'.join(early)+'\n    // S1 cannot')
(rtl/'autonomy_chassis_top.sv').write_text(s)
s=(base/'rtl/chassis_video_top.sv').read_text()
s=s.replace('Camera plus byte-identical verified board-key chassis. See reference/chassis_sha256.json.',
 'New autonomous chassis wrapper plus retained camera/HDMI baseline.')
s=s.replace('module chassis_video_top(', 'module robot_board_top #(parameter HARDWARE_CONFIRMED=0)(\n    input wire lidar_rx,\n    output wire lidar_tx,')
s=s.replace('chassis_boardkey_top chassis(', 'autonomy_chassis_top #(.HARDWARE_CONFIRMED(HARDWARE_CONFIRMED)) chassis(.lidar_rx(lidar_rx),.lidar_tx(lidar_tx),')
(rtl/'robot_board_top.sv').write_text(s)
s=(base/'rtl/osd_overlay.sv').read_text()
start=s.index(' function automatic [7:0] key_char')
end=s.index(' // Hand-authored',start)
s=s[:start]+''' function automatic [7:0] text_char(input [2:0] row,input [5:0] col);
  begin
   text_char=" ";
   if(row==0)begin
    if(col<4)text_char=sc("YAW ",4,col);
    else if(col<14)text_char=number_char(2,col-4);
    else if(col==15)text_char=8'hb0;
   end
   if(row==1)begin
    if(col<5)text_char=sc("MODE ",5,col);
    else if(col<13)case(snapshot[801:800])
     0:text_char=sc("STOP    ",8,col-5);
     1:text_char=sc("MANUAL  ",8,col-5);
     2:text_char=sc("MAP NAV ",8,col-5);
     3:text_char=sc("FOLLOW  ",8,col-5);
    endcase
    else if(col>=14 && col<23)text_char=sc(" DEV LOCK",9,col-14);
   end
  end
 endfunction
'''+s[end:]
s=s.replace("1:text_color=24'h00e8ff", "1:text_color=24'h00ff60")
(rtl/'osd_overlay.sv').write_text(s)
constraints=ROOT/'fpga/constraints';constraints.mkdir(exist_ok=True)
s=(base/'constraints/video_60k.cst').read_text()
s+='\n// New C1 UART; unused P7 pins verified against vendor spreadsheet.\n'
s+='IO_LOC "lidar_rx" D20;\nIO_PORT "lidar_rx" IO_TYPE=LVCMOS33 PULL_MODE=UP;\n'
s+='IO_LOC "lidar_tx" E17;\nIO_PORT "lidar_tx" IO_TYPE=LVCMOS33 DRIVE=8;\n'
(constraints/'robot.cst').write_text(s)
s=(base/'constraints/video_60k.sdc').read_text()
# Two obsolete constraints targeted the removed wheel-speed OSD formatter.
# Keep real navigation/control paths timed; do not hide failures with exceptions.
s='\n'.join(line for line in s.splitlines() if 'camera/wheel_observer/formatter/' not in line)+'\n'
(constraints/'robot.sdc').write_text(s)
tree=ET.parse(base/'chassis_video_top.gprj');fl=tree.find('.//FileList')
removed={'rtl/chassis_video_top.sv','chassis/rtl/chassis_top.sv','chassis/rtl/telemetry_formatter.sv','rtl/osd_overlay.sv','constraints/video_60k.cst','constraints/video_60k.sdc'}
for e in list(fl):
 if e.attrib['path'] in removed:fl.remove(e)
 else:e.attrib['path']='../baseline/'+e.attrib['path']
for p in sorted(rtl.glob('*.sv')):ET.SubElement(fl,'File',{'path':'rtl/'+p.name,'type':'file.verilog','enable':'1'})
ET.SubElement(fl,'File',{'path':'constraints/robot.cst','type':'file.cst','enable':'1'})
ET.SubElement(fl,'File',{'path':'constraints/robot.sdc','type':'file.sdc','enable':'1'})
tree.write(ROOT/'fpga/robot_board_top.gprj',encoding='utf-8',xml_declaration=True)
(ROOT/'fpga/synth.tcl').write_text('''open_project robot_board_top.gprj
set_option -top_module robot_board_top
set_option -output_base_name robot_board_top
set_option -synthesis_tool gowinsynthesis
set_option -verilog_std sysv2017
set_option -include_path {../baseline/src/vga;../baseline/chassis/rtl}
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 0
run syn
''')
print('Prepared board project, HARDWARE_CONFIRMED=0; no programming performed.')
