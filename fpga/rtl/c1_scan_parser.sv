// C1 normal SCAN response only: A5 5A 05 00 00 40 81, then 5-byte nodes.
// No invented checksum: this legacy format has sync/check bits, not a CRC.
module c1_scan_parser(
 input logic clk,rst,tick_ms,rx_valid,input logic [7:0] rx_data,
 output logic point_valid,scan_start,output logic [14:0] angle_q6,
 output logic [15:0] distance_q2,output logic [5:0] quality,
 output logic linked,output logic [31:0] errors
);
 logic [2:0] descriptor_pos,node_pos;
 logic [7:0] b0,b1,b2,b3;
 logic [9:0] age;
 function automatic [7:0] descriptor(input [2:0] p);
  case(p) 0:descriptor=8'ha5;1:descriptor=8'h5a;2:descriptor=5;
   3,4:descriptor=0;5:descriptor=8'h40;default:descriptor=8'h81;endcase
 endfunction
 always_ff @(posedge clk) begin
  point_valid<=0;scan_start<=0;
  if(rst) begin
   descriptor_pos<=0;node_pos<=0;age<=0;linked<=0;errors<=0;
   angle_q6<=0;distance_q2<=0;quality<=0;b0<=0;b1<=0;b2<=0;b3<=0;
  end else begin
   if(tick_ms && age<1000) age<=age+1'b1;
   if(age>=500) begin linked<=0;descriptor_pos<=0;node_pos<=0;end
   if(rx_valid) begin
    age<=0;
    if(!linked) begin
     if(rx_data==descriptor(descriptor_pos)) begin
      if(descriptor_pos==6) begin linked<=1;descriptor_pos<=0;node_pos<=0;end
      else descriptor_pos<=descriptor_pos+1'b1;
     end else begin descriptor_pos<=(rx_data==8'ha5)?1:0;end
    end else case(node_pos)
     0:if(rx_data[0]^rx_data[1]) begin b0<=rx_data;node_pos<=1;end
       else errors<=errors+1'b1;
     1:if(rx_data[0]) begin b1<=rx_data;node_pos<=2;end
       else begin node_pos<=0;errors<=errors+1'b1;end
     2:begin b2<=rx_data;node_pos<=3;end
     3:begin b3<=rx_data;node_pos<=4;end
     4:begin
      node_pos<=0;
      if({b2,b1[7:1]}<15'd23040) begin
       angle_q6<={b2,b1[7:1]};distance_q2<={rx_data,b3};quality<=b0[7:2];
       point_valid<=1;scan_start<=b0[0];
      end else errors<=errors+1'b1;
     end
     default:node_pos<=0;
    endcase
   end
  end
 end
endmodule
