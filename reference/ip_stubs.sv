// SIMULATION / ELABORATION ONLY. Never add this file to a Gowin project.
// Port shells are not models of calibration, memory, PLL or serializer timing.
module camera_pll(input clkin,mdclk,reset,output clkout0,clkout1,clkout2,lock);
 assign {clkout0,clkout1,clkout2}={3{clkin}}; assign lock=!reset;
endmodule
module ddr_pll(input clkin,mdclk,reset,mdainc,pll_init_bypass,input [1:0] mdopc,
 input [7:0] mdwdi,output [7:0] mdrdo,output clkout0,clkout1,clkout2,lock);
 assign {clkout0,clkout1,clkout2}={3{clkin}}; assign lock=!reset;assign mdrdo=0;
endmodule
module pll_mDRP_intf(input clk,rst_n,pll_lock,wr,input [7:0] mdrp_rdata,
 output mdrp_inc,output [1:0] mdrp_op,output [7:0] mdrp_wdata);
 assign mdrp_inc=0;assign mdrp_op=0;assign mdrp_wdata=0;
endmodule
module CLKDIV #(parameter DIV_MODE="5")(input RESETN,HCLKIN,CALIB,output CLKOUT);
 assign CLKOUT=HCLKIN;
endmodule
module Gowin_DDR(input [7:0] din,input clk,output [3:0] q);
 assign q=din[3:0];
endmodule
// Interface-only serializer stub; real primitive is tested separately.
module OSER10(output Q,input D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,PCLK,FCLK,RESET);
 assign Q=RESET?0:D0;
endmodule
module ELVDS_OBUF(input I,output O,OB);
 assign O=I;assign OB=~I;
endmodule
module wr_data_fifo(input [15:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [12:0] Wnum,output [9:0] Rnum,output Almost_Empty,Almost_Full,Empty,Full,
 output [127:0] Q);
 assign Wnum=0;assign Rnum=0;assign Almost_Empty=1;assign Almost_Full=0;
 assign Empty=1;assign Full=0;assign Q=0;
endmodule
module rd_data_fifo(input [127:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [9:0] Wnum,output [12:0] Rnum,output Almost_Empty,Almost_Full,Empty,Full,
 output [15:0] Q);
 assign Wnum=0;assign Rnum=0;assign Almost_Empty=1;assign Almost_Full=0;
 assign Empty=1;assign Full=0;assign Q=0;
endmodule
// Connectivity-only stub: real IODELAY electrical timing is checked separately.
module IODELAY #(parameter C_STATIC_DLY=0,DYN_DLY_EN="FALSE",ADAPT_EN="FALSE")
 (input DI,SDTAP,VALUE,input [7:0] DLYSTEP,output DO,DF);
 assign DO=DI;assign DF=0;
endmodule
module DDR3_Memory_Interface_Top(
 input clk,memory_clk,pll_lock,rst_n,cmd_en,wr_data_en,wr_data_end,sr_req,ref_req,burst,
 input [2:0] cmd,input [27:0] addr,input [127:0] wr_data,input [15:0] wr_data_mask,
 output pll_stop,cmd_ready,wr_data_rdy,rd_data_valid,rd_data_end,sr_ack,ref_ack,
 init_calib_complete,clk_out,ddr_rst,
 output [127:0] rd_data,output [13:0] O_ddr_addr,output [2:0] O_ddr_ba,
 output O_ddr_cs_n,O_ddr_ras_n,O_ddr_cas_n,O_ddr_we_n,O_ddr_clk,O_ddr_clk_n,
 O_ddr_cke,O_ddr_odt,O_ddr_reset_n,output [1:0] O_ddr_dqm,
 inout [15:0] IO_ddr_dq,inout [1:0] IO_ddr_dqs,IO_ddr_dqs_n);
 assign clk_out=clk;
endmodule
