open_project robot_board_top.gprj
set_option -top_module robot_board_top
set_option -output_base_name robot_board_top
set_option -synthesis_tool gowinsynthesis
set_option -verilog_std sysv2017
set_option -include_path {../baseline/src/vga;../baseline/chassis/rtl}
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 0
run syn
