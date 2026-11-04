// Fixed 32-byte frame: AA55, version=1, type, seq LE16, payload[24], CRC LE16.
// CRC-16/CCITT-FALSE over bytes 2..29. Byte timeout 20 ms.
module mission_rx(
 input logic clk,rst,tick_ms,rx_valid,input logic [7:0] rx_data,
 output logic valid,output logic [7:0] kind,output logic [15:0] seq,
 output logic [191:0] payload,output logic [31:0] errors
);
 logic [5:0] pos;logic [5:0] age;logic [15:0] crc,seq_buf,last_seq;
 logic [7:0] version_buf,kind_buf,crc_lo;
 logic [191:0] buf_payload;logic seq_seen;
 wire [15:0] delta=seq_buf-last_seq;
 function automatic [15:0] step_crc(input [15:0] c,input [7:0] b);
  reg [15:0] t;integer j;
  begin t=c^{b,8'b0};for(j=0;j<8;j=j+1)t=t[15]?(t<<1)^16'h1021:t<<1;step_crc=t;end
 endfunction
 always_ff @(posedge clk) begin
  valid<=0;
  if(rst) begin
   pos<=0;age<=0;crc<=16'hffff;seq_seen<=0;last_seq<=0;
   errors<=0;kind<=0;seq<=0;payload<=0;seq_buf<=0;kind_buf<=0;
   version_buf<=0;crc_lo<=0;buf_payload<=0;
  end else begin
   if(tick_ms && pos!=0) begin
    if(age>=19)begin pos<=0;age<=0;errors<=errors+1'b1;end else age<=age+1'b1;
   end
   if(rx_valid) begin
    age<=0;
    case(pos)
     0:if(rx_data==8'haa)pos<=1;
     1:if(rx_data==8'h55)begin pos<=2;crc<=16'hffff;end else pos<=(rx_data==8'haa) ? 1 : 0;
     30:begin crc_lo<=rx_data;pos<=31;end
     31:begin
      pos<=0;
      if({rx_data,crc_lo}==crc && version_buf==1 &&
         (kind_buf==2 || !seq_seen || (delta!=0 && !delta[15])))begin
       valid<=1;kind<=kind_buf;seq<=seq_buf;payload<=buf_payload;
       // STOP is always accepted but cannot roll sequence backwards.
       if(kind_buf!=2 || !seq_seen || (delta!=0 && !delta[15]))begin
        seq_seen<=1;last_seq<=seq_buf;
       end
      end else errors<=errors+1'b1;
     end
     default:begin
      crc<=step_crc(crc,rx_data);pos<=pos+1'b1;
      case(pos)
       2:version_buf<=rx_data;3:kind_buf<=rx_data;
       4:seq_buf[7:0]<=rx_data;5:seq_buf[15:8]<=rx_data;
       default:buf_payload[(pos-6)*8+:8]<=rx_data;
      endcase
     end
    endcase
   end
  end
 end
endmodule
