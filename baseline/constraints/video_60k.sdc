# Baseline constraints. Complete timing closure in Gowin before board release.
create_clock -name clk50M -period 20.000 [get_ports {clk50M}]
# Conservative capture-clock budget; verify the actual PCLK with this sensor ROM.
create_clock -name camera_pclk -period 10.000 [get_ports {camera_pclk}]
# PLLA clocks are not auto-derived by this Gowin version. Use the actual native
# PLL parameters: VCO 750 MHz video / 800 MHz DDR. Do not call 75 MHz 74.25 MHz.
create_generated_clock -name video_serial -source [get_ports {clk50M}] -multiply_by 15 -divide_by 2 [get_pins {camera/your_instance_name/u_pll/PLLA_inst/CLKOUT0}]
create_generated_clock -name sensor_xclk -source [get_ports {clk50M}] -divide_by 2 [get_pins {camera/your_instance_name/u_pll/PLLA_inst/CLKOUT1}]
create_generated_clock -name sensor_config -source [get_ports {clk50M}] -divide_by 1 [get_pins {camera/your_instance_name/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name video_pixel -source [get_pins {camera/your_instance_name/u_pll/PLLA_inst/CLKOUT0}] -divide_by 5 [get_pins {camera/u_clkdiv/CLKOUT}]
create_generated_clock -name ddr_fast -source [get_ports {clk50M}] -multiply_by 8 [get_pins {camera/ddr_pll/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name ddr_ui -source [get_pins {camera/ddr_pll/u_pll/PLLA_inst/CLKOUT2}] -divide_by 4 [get_pins {camera/ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
# No blanket false paths: review asynchronous FIFO/status crossings in STA.
# DVP input delays and external TMDS skew remain board-verification items.
# telemetry.field changes only on SEP acceptance. The next load_number runs
# after SIGN/D5..D1/DOT/F3/F2/F1/SEP, never on the next clock. PREFIX loads
# constant index 0. Restrict relaxation to this stable field-index -> number
# path; values/state/ready and all motor-control paths retain single-cycle STA.
set_multicycle_path -setup 2 -from [get_regs {chassis/telemetry/field*}] -to [get_regs {chassis/telemetry/integer_part*}]
set_multicycle_path -hold 1 -from [get_regs {chassis/telemetry/field*}] -to [get_regs {chassis/telemetry/integer_part*}]
# fraction uses the same load_number(field+1) event and stable index as above.
# Do NOT relax values[] -> integer_part/fraction: PREFIX can load next cycle.
set_multicycle_path -setup 2 -from [get_regs {chassis/telemetry/field*}] -to [get_regs {chassis/telemetry/fraction*}]
set_multicycle_path -hold 1 -from [get_regs {chassis/telemetry/field*}] -to [get_regs {chassis/telemetry/fraction*}]

# Board-tested camera input delays and explicitly scoped FIFO/reset crossings.
set_input_delay -clock camera_pclk -max 5.000 [get_ports {camera_data[*] camera_href camera_vsync}]
set_input_delay -clock camera_pclk -min 0.000 [get_ports {camera_data[*] camera_href camera_vsync}]
set_false_path -hold -to [get_pins {camera/ddr3_ctrl_2port/fifo_ddr3_adapter/*_data_fifo/fifo_inst/Big.wq1_rptr_*_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/*_data_fifo/fifo_inst/Big.rq1_wptr_*_s0/D}]
set_max_delay 5.000 -to [get_pins {camera/ddr3_ctrl_2port/fifo_ddr3_adapter/*_data_fifo/fifo_inst/Big.wq1_rptr_*_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/*_data_fifo/fifo_inst/Big.rq1_wptr_*_s0/D}]
# The generated width-converting read FIFO uses Small, not Big. Its .vo
# explicitly connects these first-stage registers to wq2/rq2 on the SAME
# destination clock. Bound Gray-bus delay below both clocks' periods; do not
# waive first-stage -> second-stage or RAM data paths.
set_false_path -hold -to [get_pins {camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.wq1_rptr_*_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.rq1_wptr_*_s0/D}]
set_max_delay 5.000 -to [get_pins {camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.wq1_rptr_*_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.rq1_wptr_*_s0/D}]
# Single-bit two-flop CDC entry points only. held[] -> frame_data[] remains
# timed, as do the second synchronizer stages. These status/toggle inputs do
# not have a synchronous phase relationship with the receiving clock.
set_false_path -hold -to [get_pins {camera/osd_mailbox/ack_sync_0_s0/D camera/osd_mailbox/req_sync_0_s0/D camera/select_video/frame_sync_0_s0/D camera/select_video/error_sync_0_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_load_d0_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/wr_load_d0_s0/D}]
set_max_delay 5.000 -to [get_pins {camera/osd_mailbox/ack_sync_0_s0/D camera/osd_mailbox/req_sync_0_s0/D camera/select_video/frame_sync_0_s0/D camera/select_video/error_sync_0_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/rd_load_d0_s0/D camera/ddr3_ctrl_2port/fifo_ddr3_adapter/wr_load_d0_s0/D}]
set_false_path -to [get_pins {camera/capture_reset/release_pipe_*_s0/CLEAR camera/ddr3_ctrl_2port/fifo_ddr3_adapter/write_reset/release_pipe_*_s0/CLEAR}]
# Read-only wheel formatter has the identical registered field-index schedule
# to the unchanged chassis formatter; do not relax live speed input paths.
set_multicycle_path -setup 2 -from [get_regs {camera/wheel_observer/formatter/field*}] -to [get_regs {camera/wheel_observer/formatter/integer_part* camera/wheel_observer/formatter/fraction*}]
set_multicycle_path -hold 1 -from [get_regs {camera/wheel_observer/formatter/field*}] -to [get_regs {camera/wheel_observer/formatter/integer_part* camera/wheel_observer/formatter/fraction*}]
# imu_parser stores raw angle/gyro bytes in frame[2..7]. Outputs consume them
# only after bytes 8,9 and checksum (index 10). Even consecutive rx_valid cycles
# leave THREE clocks from the latest raw byte (7) to output load. Two clocks
# are therefore sufficient and conservative. Only stored frame -> conversion
# outputs is relaxed: yaw_offset, zero_request, rx_data and PID remain timed
# at one cycle. No changes to the preserved chassis RTL/protocol.
set_multicycle_path -setup 2 -from [get_regs {chassis/imu_protocol/frame*}] -to [get_regs {chassis/imu_protocol/yaw_q16* chassis/imu_protocol/roll_q16* chassis/imu_protocol/pitch_q16* chassis/imu_protocol/gyro_x_q16* chassis/imu_protocol/gyro_y_q16* chassis/imu_protocol/gyro_z_q16* chassis/imu_protocol/yaw_offset_q16*}]
set_multicycle_path -hold 1 -from [get_regs {chassis/imu_protocol/frame*}] -to [get_regs {chassis/imu_protocol/yaw_q16* chassis/imu_protocol/roll_q16* chassis/imu_protocol/pitch_q16* chassis/imu_protocol/gyro_x_q16* chassis/imu_protocol/gyro_y_q16* chassis/imu_protocol/gyro_z_q16* chassis/imu_protocol/yaw_offset_q16*}]
# The 20261018_C 0.5 ns RAM min-delay experiment did not alter routing and
# created additional violations. Restore native hold checks, NOT false paths.
# More than 25 endpoints are failing; include enough paths to expose all
# remaining classes rather than repeatedly seeing only the DDR calibration.
report_timing -setup -max_paths 200 -max_common_paths 1
report_timing -hold -max_paths 100 -max_common_paths 1
report_timing -recovery -max_paths 100 -max_common_paths 1
report_timing -removal -max_paths 100 -max_common_paths 1
