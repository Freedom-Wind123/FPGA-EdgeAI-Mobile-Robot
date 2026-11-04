`timescale 1ns/1ps
// Single OV5640 register access, 7-bit slave address 0x3c.
// Open-drain pins. No clock stretching supported; a stuck SCL is an error.
module sccb_master #(parameter integer QUARTER_CYCLES=313)(
 input wire clk, rst, start, read_reg,
 input wire [15:0] reg_addr, input wire [7:0] wr_data,
 inout wire scl, sda,
 output reg busy, done, error, output reg [7:0] rd_data
);
 reg scl_low, sda_low;
 assign scl=scl_low?1'b0:1'bz;
 assign sda=sda_low?1'b0:1'bz;
 reg [15:0] divider;
 reg [5:0] state;
 reg [2:0] bitno, byte_no;
 reg [7:0] tx;
 reg [15:0] addr;
 reg [7:0] value;
 reg reading;
 localparam IDLE=0,START0=1,START1=2,TX0=3,TX1=4,TX2=5,TX3=6,
  ACK0=7,ACK1=8,ACK2=9,ACK3=10,REP0=11,REP1=12,REP2=13,
  RX0=14,RX1=15,RX2=16,RX3=17,NA0=18,NA1=19,NA2=20,NA3=21,
  STOP0=22,STOP1=23,STOP2=24,STOP3=25;
 always @(posedge clk) begin
  done<=0;
  if(rst) begin
   state<=IDLE; divider<=0; scl_low<=0; sda_low<=0;
   busy<=0; done<=0; error<=0; rd_data<=0; bitno<=7; byte_no<=0;
   tx<=0; addr<=0; value<=0; reading<=0;
  end else if(state==IDLE) begin
   divider<=0;
   if(start) begin
    addr<=reg_addr; value<=wr_data; reading<=read_reg;
    tx<=8'h78; byte_no<=0; bitno<=7; error<=0; busy<=1; state<=START0;
   end
  end else if(divider==QUARTER_CYCLES-1) begin
   divider<=0;
   case(state)
    START0: begin scl_low<=0; sda_low<=1; state<=START1; end
    START1: begin scl_low<=1; state<=TX0; end
    TX0: begin sda_low<=!tx[bitno]; state<=TX1; end
    TX1: begin scl_low<=0; state<=TX2; end
    TX2: begin if(scl!==1'b1) error<=1; state<=TX3; end
    TX3: begin scl_low<=1; if(bitno==0) state<=ACK0;
      else begin bitno<=bitno-1'b1; state<=TX0; end end
    ACK0: begin sda_low<=0; state<=ACK1; end
    ACK1: begin scl_low<=0; state<=ACK2; end
    ACK2: begin if(sda!==1'b0 || scl!==1'b1) error<=1; state<=ACK3; end
    ACK3: begin
     scl_low<=1; bitno<=7;
     if(error) state<=STOP0;
     else case(byte_no)
      0: begin tx<=addr[15:8]; byte_no<=1; state<=TX0; end
      1: begin tx<=addr[7:0]; byte_no<=2; state<=TX0; end
      2: if(reading) state<=REP0;
         else begin tx<=value; byte_no<=3; state<=TX0; end
      3: if(reading) state<=RX0; else state<=STOP0;
      default: state<=STOP0;
     endcase
    end
    REP0: begin sda_low<=0; state<=REP1; end
    REP1: begin scl_low<=0; state<=REP2; end
    REP2: begin sda_low<=1; tx<=8'h79; byte_no<=3; state<=START1; end
    RX0: begin sda_low<=0; state<=RX1; end
    RX1: begin scl_low<=0; state<=RX2; end
    RX2: begin rd_data[bitno]<=sda; if(scl!==1'b1) error<=1; state<=RX3; end
    RX3: begin scl_low<=1; if(bitno==0) state<=NA0;
      else begin bitno<=bitno-1'b1; state<=RX0; end end
    NA0: begin sda_low<=0; state<=NA1; end
    NA1: begin scl_low<=0; state<=NA2; end
    NA2: state<=NA3;
    NA3: begin scl_low<=1; state<=STOP0; end
    STOP0: begin scl_low<=1; sda_low<=1; state<=STOP1; end
    STOP1: begin scl_low<=0; state<=STOP2; end
    STOP2: begin sda_low<=0; state<=STOP3; end
    STOP3: begin busy<=0; done<=1; state<=IDLE; end
    default: state<=STOP0;
   endcase
  end else divider<=divider+1'b1;
 end
endmodule
