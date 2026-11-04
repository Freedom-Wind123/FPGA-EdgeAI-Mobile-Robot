// Single write, synchronous read. Clear through write port; no array reset.
module sync_ram #(parameter AW=12,DW=2)(
 input logic clk,we,input logic [AW-1:0] waddr,raddr,
 input logic [DW-1:0] wdata,output logic [DW-1:0] rdata
);
 (* syn_ramstyle="block_ram" *) logic [DW-1:0] mem[0:(1<<AW)-1];
 always_ff @(posedge clk)begin
  if(we)mem[waddr]<=wdata;
  rdata<=mem[raddr];
 end
endmodule
