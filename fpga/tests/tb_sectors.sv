`timescale 1ns/1ps
module tb_sectors;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,tick_ms=0,point_valid=0,scan_start=0;
 reg [14:0] angle_q6;reg [15:0] distance_q2;reg [5:0] quality=20;
 wire [127:0] ranges_mm;wire [7:0] valid_mask;wire fresh,scan_valid;
 integer i,j,a,expected;
 c1_sectors dut(.*);
 task p(input integer a,d,s);begin
  @(negedge clk);angle_q6=a;distance_q2=d;scan_start=s;point_valid=1;
  @(negedge clk);point_valid=0;
 end endtask
 initial begin
  // Exhaust all 360*64 legal angles, including wrap and every octant boundary.
  for(a=0;a<23040;a=a+1)begin
   angle_q6=a;expected=1<<(((a+1440)/2880)%8);#1;
   if(dut.sector_sel!==expected[7:0])$fatal(1,"octant mismatch at %d",a);
  end
  repeat(3)@(negedge clk);rst=0;
  for(i=0;i<8;i=i+1)for(j=0;j<4;j=j+1)p(i*2880,4000,i==0 && j==0);
  p(0,2000,1);#1;if(!fresh || valid_mask!=255 || ranges_mm[15:0]!=1000)$fatal(1,"scan publish");
  // Current scan only includes front; other sectors must not retain old free data.
  p(1,2000,0);p(2,2000,0);p(3,2000,0);p(0,4000,1);#1;
  if(fresh || valid_mask!=1 || ranges_mm[15:0]!=500)$fatal(1,"partial scan trusted");
  tick_ms=1;repeat(260)@(negedge clk);tick_ms=0;
  if(fresh)$fatal(1,"scan stale");
  $display("PASS sectors: atomic scans, minimum distance, partial rejection, age");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
