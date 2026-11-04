`timescale 1ns/1ps
// Uses the native 60K reference's ordered 720p RGB565 register ROM.
// Reads chip ID first; stops at the first failed bus transaction.
module camera_setup #(parameter IMAGE_WIDTH=1280,IMAGE_HEIGHT=720,
 parameter integer CLK_HZ=50000000)(
 input wire Clk,Rst_n, output reg Init_Done,failed,
 output wire camera_rst_n, inout wire i2c_sclk,i2c_sdat,
 output reg [15:0] sensor_id,failed_register
);
 localparam MS=CLK_HZ/1000;
 reg [31:0] timer; reg [3:0] state;
 reg [7:0] index; wire [23:0] lut;
 reg start,reading,release_camera;
 reg [15:0] address; reg [7:0] value;
 wire done,busy,bus_error; wire [7:0] data;
 assign camera_rst_n=Rst_n && release_camera;
 ov5640_init_table_rgb #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
  .IMAGE_FLIP_EN(0),.IMAGE_MIRROR_EN(0)) rom(.clk(Clk),.addr(index),.q(lut));
 sccb_master #(.QUARTER_CYCLES(CLK_HZ/160000)) bus(.clk(Clk),.rst(!Rst_n),
  .start(start),.read_reg(reading),.reg_addr(address),.wr_data(value),
  .scl(i2c_sclk),.sda(i2c_sdat),.busy(busy),.done(done),.error(bus_error),.rd_data(data));
 always @(posedge Clk or negedge Rst_n) begin
  if(!Rst_n) begin
   state<=0; timer<=0; index<=0; start<=0; reading<=0; release_camera<=0;
   address<=0; value<=0; Init_Done<=0; failed<=0; sensor_id<=0; failed_register<=0;
  end else begin
   start<=0;
   case(state)
    0: if(timer==10*MS-1) begin timer<=0; release_camera<=1; state<=1; end else timer<=timer+1'b1;
    1: if(timer==25*MS-1) begin timer<=0; state<=2; end else timer<=timer+1'b1;
    2: begin reading<=1; address<=16'h300a; start<=1; state<=3; end
    3: if(done) begin
     if(bus_error) begin failed<=1; failed_register<=address; state<=15; end
     else begin sensor_id[15:8]<=data; state<=4; end
    end
    4: begin address<=16'h300b; start<=1; state<=5; end
    5: if(done) begin
     sensor_id[7:0]<=data;
     if(bus_error || sensor_id[15:8]!=8'h56 || data!=8'h40) begin
      failed<=1; failed_register<=address; state<=15;
     end else state<=6;
    end
    6: begin reading<=0; address<=lut[23:8]; value<=lut[7:0]; start<=1; state<=7; end
    7: if(done) begin
     if(bus_error) begin failed<=1; failed_register<=address; state<=15; end
     else begin timer<=0; state<=8; end
    end
    8: if(timer==((address==16'h3008 && value==8'h82)?10*MS:2*MS)-1) begin
     timer<=0;
     if(index==252) begin Init_Done<=1; state<=15; end
     else begin index<=index+1'b1; state<=9; end
    end else timer<=timer+1'b1;
    9: state<=6; // allow the registered ROM output to settle
    default: state<=15;
   endcase
  end
 end
endmodule
