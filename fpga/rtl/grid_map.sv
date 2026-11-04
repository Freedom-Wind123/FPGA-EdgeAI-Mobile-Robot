// 0 unknown, 1 observed free, 2/3 occupied. Serial Bresenham in synchronous RAM.
module grid_map #(parameter LOG2=6)(
 input logic clk,rst,freeze,ray_valid,
 input logic signed [15:0] sx,sy,ex,ey,
 output logic ray_ready,initialized,
 input logic [2*LOG2-1:0] rd_addr,output logic [1:0] rd_cell,
 output logic [31:0] rays_done
);
 localparam N=1<<LOG2,SIZE=N*N,AW=2*LOG2;
 logic [AW-1:0] clear_addr,waddr,raddr;
 logic [1:0] wdata,rdata;logic we;
 sync_ram #(.AW(AW),.DW(2)) storage(.clk,.we,.waddr,.raddr,.wdata,.rdata);
 logic active;logic [1:0] phase;
 integer x,y,tx,ty,dx,dy,step_x,step_y,err,e2;
 always_comb begin e2=err*2;raddr=active?(y<<LOG2)+x:rd_addr;end
 assign rd_cell=rdata;
 assign ray_ready=initialized && !active && !freeze && !we;
 always_ff @(posedge clk)begin
  we<=0;
  if(rst)begin clear_addr<=0;initialized<=0;active<=0;rays_done<=0;phase<=0;waddr<=0;wdata<=0;
   x<=0;y<=0;tx<=0;ty<=0;dx<=0;dy<=0;step_x<=0;step_y<=0;err<=0;
  end else if(!initialized)begin
   we<=1;waddr<=clear_addr;wdata<=0;
   if(clear_addr==SIZE-1)initialized<=1;else clear_addr<=clear_addr+1'b1;
  end else if(active)begin
   if(x<0 || x>=N || y<0 || y>=N)begin active<=0;rays_done<=rays_done+1'b1;end
   else if(phase<2)phase<=phase+1'b1;
   else begin
    we<=1;waddr<=raddr;phase<=0;
    if(x==tx && y==ty)begin wdata<=3;active<=0;rays_done<=rays_done+1'b1;end
    else begin
     wdata<=rdata>1?rdata-1'b1:2'd1;
     if(e2>=dy)x<=x+step_x;
     if(e2<=dx)y<=y+step_y;
     err<=err+((e2>=dy)?dy:0)+((e2<=dx)?dx:0);
    end
   end
  end else if(ray_valid && ray_ready)begin
   x<=sx;y<=sy;tx<=ex;ty<=ey;step_x<=ex>sx?1:-1;step_y<=ey>sy?1:-1;
   dx<=ex>sx?ex-sx:sx-ex;dy<=ey>sy?sy-ey:ey-sy;
   err<=(ex>sx?ex-sx:sx-ex)+(ey>sy?sy-ey:ey-sy);active<=1;phase<=0;
  end
 end
endmodule
