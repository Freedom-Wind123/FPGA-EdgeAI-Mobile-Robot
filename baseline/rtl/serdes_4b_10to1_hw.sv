// GW5AT native output serializers + emulated complementary output buffers.
// One OSER10 occupies a paired IO resource; two per pair cannot be placed.
// UG304 OSER10: D0 is first; PCLK=FCLK/5; no fabric logic runs at 375 MHz.
module serdes_4b_10to1_hw(
 input wire pixelclk,clkx5,rst_n,
 input wire [9:0] datain_0,datain_1,datain_2,datain_3,
 output wire dataout_0_p,dataout_0_n,dataout_1_p,dataout_1_n,
 output wire dataout_2_p,dataout_2_n,dataout_3_p,dataout_3_n
);
 wire [39:0] words={datain_3,datain_2,datain_1,datain_0};
 wire [3:0] p,n;
 assign {dataout_3_p,dataout_2_p,dataout_1_p,dataout_0_p}=p;
 assign {dataout_3_n,dataout_2_n,dataout_1_n,dataout_0_n}=n;
 genvar i;
 generate for(i=0;i<4;i=i+1) begin: lane
  wire [9:0] d=words[i*10+:10];
  wire serial;
  OSER10 pos(.Q(serial),.D0(d[0]),.D1(d[1]),.D2(d[2]),.D3(d[3]),.D4(d[4]),
   .D5(d[5]),.D6(d[6]),.D7(d[7]),.D8(d[8]),.D9(d[9]),
   .PCLK(pixelclk),.FCLK(clkx5),.RESET(!rst_n));
  ELVDS_OBUF outbuf(.I(serial),.O(p[i]),.OB(n[i]));
 end endgenerate
endmodule
