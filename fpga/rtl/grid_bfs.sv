// Four-connected BFS, first step output. Queue/node metadata use synchronous RAM.
// Unknown is blocked. Inflate every candidate footprint. Freeze map while busy.
module grid_bfs #(parameter LOG2=6,RADIUS=2)(
 input logic clk,rst,start,input logic [LOG2-1:0] start_x,start_y,goal_x,goal_y,
 output logic busy,done,found,output logic signed [1:0] step_dx,step_dy,
 output logic [2*LOG2-1:0] map_addr,input logic [1:0] map_cell
);
 localparam N=1<<LOG2,SIZE=N*N,AW=2*LOG2;
 localparam IDLE=0,CLEAR=1,SEED=2,POP=3,QWAIT=4,QREAD=5,NWAIT=6,NREAD=7,
            NEIGH=8,VWAIT=9,VREAD=10,CHECK=11,MWAIT=12,MREAD=13,ENQUEUE=14,MWAIT2=15;
 logic [3:0] state;
 logic n_we,q_we;logic [AW-1:0] n_wa,n_ra,q_wa,q_ra;
 logic [2:0] n_wd,n_rd;logic [AW-1:0] q_wd,q_rd;
 sync_ram #(.AW(AW),.DW(3)) nodes(.clk,.we(n_we),.waddr(n_wa),.raddr(n_ra),.wdata(n_wd),.rdata(n_rd));
 sync_ram #(.AW(AW),.DW(AW)) queue(.clk,.we(q_we),.waddr(q_wa),.raddr(q_ra),.wdata(q_wd),.rdata(q_rd));
 logic [AW-1:0] clear_addr,origin,target,current,candidate;
 logic [AW:0] head,tail;logic [2:0] direction;logic [1:0] root_dir,current_dir;
 integer nx,ny,ox,oy,px,py;
 always_comb begin
  nx=current[LOG2-1:0];ny=current[AW-1:LOG2];
  case(direction)0:nx=nx+1;1:ny=ny+1;2:nx=nx-1;3:ny=ny-1;default:begin end endcase
  px=nx+ox;py=ny+oy;
 end
 task automatic finish(input logic ok,input logic [1:0] dir,input logic same);
  begin
   found<=ok;done<=1;busy<=0;state<=IDLE;step_dx<=0;step_dy<=0;
   if(ok && !same)case(dir)0:step_dx<=1;1:step_dy<=1;2:step_dx<=-1;3:step_dy<=-1;endcase
  end
 endtask
 always_ff @(posedge clk)begin
  done<=0;n_we<=0;q_we<=0;
  if(rst)begin
   busy<=0;found<=0;state<=IDLE;clear_addr<=0;head<=0;tail<=0;direction<=0;
   step_dx<=0;step_dy<=0;map_addr<=0;origin<=0;target<=0;current<=0;candidate<=0;
   root_dir<=0;current_dir<=0;ox<=0;oy<=0;
   n_wa<=0;n_ra<=0;n_wd<=0;q_wa<=0;q_ra<=0;q_wd<=0;
  end else case(state)
   IDLE:if(start)begin
    origin<={start_y,start_x};target<={goal_y,goal_x};busy<=1;found<=0;
    clear_addr<=0;state<=CLEAR;step_dx<=0;step_dy<=0;
   end
   CLEAR:begin n_we<=1;n_wa<=clear_addr;n_wd<=0;
    if(clear_addr==SIZE-1)state<=SEED;else clear_addr<=clear_addr+1'b1;end
   SEED:begin
    q_we<=1;q_wa<=0;q_wd<=origin;n_we<=1;n_wa<=origin;n_wd<=3'b100;
    head<=0;tail<=1;state<=POP;
   end
   POP:if(head==tail)finish(0,0,0);
    else begin q_ra<=head[AW-1:0];head<=head+1'b1;state<=QWAIT;end
   QWAIT:state<=QREAD;
   QREAD:begin current<=q_rd;n_ra<=q_rd;direction<=0;state<=NWAIT;end
   NWAIT:state<=NREAD;
   NREAD:begin current_dir<=n_rd[1:0];state<=NEIGH;end
   NEIGH:begin
    if(current==target)finish(1,current_dir,current==origin);
    else if(direction==4)state<=POP;
    else if(nx<0 || ny<0 || nx>=N || ny>=N)direction<=direction+1'b1;
    else begin
     candidate<=(ny<<LOG2)+nx;n_ra<=(ny<<LOG2)+nx;
     root_dir<=current==origin?direction[1:0]:current_dir;
     ox<=-RADIUS;oy<=-RADIUS;state<=VWAIT;
    end
   end
   VWAIT:state<=VREAD;
   VREAD:if(n_rd[2])begin direction<=direction+1'b1;state<=NEIGH;end else state<=CHECK;
   CHECK:if(px<0 || py<0 || px>=N || py>=N)begin direction<=direction+1'b1;state<=NEIGH;end
    else begin map_addr<=(py<<LOG2)+px;state<=MWAIT;end
   MWAIT:state<=MWAIT2;
   MWAIT2:state<=MREAD;
   MREAD:if(map_cell!=1)begin direction<=direction+1'b1;state<=NEIGH;end
    else if(ox==RADIUS && oy==RADIUS)state<=ENQUEUE;
    else begin if(ox==RADIUS)begin ox<=-RADIUS;oy<=oy+1;end else ox<=ox+1;state<=CHECK;end
   ENQUEUE:begin
    if(tail>=SIZE)finish(0,0,0);
    else begin
     n_we<=1;n_wa<=candidate;n_wd<={1'b1,root_dir};
     q_we<=1;q_wa<=tail[AW-1:0];q_wd<=candidate;
     tail<=tail+1'b1;direction<=direction+1'b1;state<=NEIGH;
    end
   end
   default:state<=IDLE;
  endcase
 end
endmodule
