// Reissue standard STOP/SCAN after startup or stream loss. No USB host involved.
module c1_uart #(parameter CLK_HZ=50000000)(
 input logic clk,rst,tick_ms,rx,linked,
 output logic tx,rx_valid,output logic [7:0] rx_data
);
 logic [11:0] delay_ms;logic [3:0] state;logic [7:0] tx_data;logic tx_start,tx_busy;
 uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(460800)) receiver(.clk,.rst,.rx,.data(rx_data),.valid(rx_valid));
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(460800)) sender(.clk,.rst,.data(tx_data),.start(tx_start),.busy(tx_busy),.tx);
 always_ff @(posedge clk)begin
  tx_start<=0;
  if(rst)begin state<=0;delay_ms<=0;tx_data<=0;end
  else case(state)
   0:if(tick_ms)begin
    if(delay_ms==999)begin delay_ms<=0;state<=1;end else delay_ms<=delay_ms+1'b1;
   end
   1:if(!tx_busy)begin tx_data<=8'ha5;tx_start<=1;state<=2;end
   2:if(tx_busy)state<=3;
   3:if(!tx_busy)begin tx_data<=8'h25;tx_start<=1;state<=4;end
   4:if(tx_busy)begin state<=5;delay_ms<=0;end
   5:if(tick_ms)begin
    if(delay_ms==49)begin delay_ms<=0;state<=6;end else delay_ms<=delay_ms+1'b1;
   end
   6:if(!tx_busy)begin tx_data<=8'ha5;tx_start<=1;state<=7;end
   7:if(tx_busy)state<=8;
   8:if(!tx_busy)begin tx_data<=8'h20;tx_start<=1;state<=9;end
   9:if(tx_busy)begin state<=10;delay_ms<=0;end
   10:if(linked)delay_ms<=0;
    else if(tick_ms)begin if(delay_ms==999)begin state<=1;delay_ms<=0;end else delay_ms<=delay_ms+1'b1;end
   default:state<=0;
  endcase
 end
endmodule
