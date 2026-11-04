
module fifo_ddr3_adapter(
    //时钟及复位
    input           ui_clk              ,    //DDR用户时钟信号
    input           rst_n               ,    //外部按键复位信号

    //DDR相关信号
    input           init_calib_complete ,   //DDR初始化完成信号
    input           app_rdy             ,   //DDR IP核空闲
    input           app_wdf_rdy         ,   //DDR IP写空闲
    input           app_rd_data_valid   ,   //DDR读数据有效信号
    input   [127:0] app_rd_data         ,
    output reg [27:0] app_addr          ,   //DDR3地址信号
    output          app_en              ,   //DDR3命令和数据使能信号
    output          app_wdf_wren        ,   //DDR3用户写使能信号
    output          app_wdf_end         ,   //DDR3写数据结束信号
    output  [2:0]   app_cmd             ,   //DDR3命令信号：0：写；1：读
    output  [127:0] app_wdf_data        ,   //写入进DDR的数据
    
    //用户接口
    input           rd_load             ,   //输出源更新信号
    input           wr_load             ,   //输入源更新信号
    input   [27:0]  app_addr_rd_min     ,   //读DDR3的起始地址
    input   [27:0]  app_addr_rd_max     ,   //读DDR3的结束地址
    input   [7:0]   rd_bust_len         ,   //从DDR3中读数据时的突发长度
    input   [27:0]  app_addr_wr_min     ,   //写DD3的起始地址
    input   [27:0]  app_addr_wr_max     ,   //写DDR的结束地址
    input   [7:0]   wr_bust_len         ,   //向DDR3中写数据时的突发长度

    input           wr_clk              ,//wr_fifo的写时钟信号
    input           wfifo_wren          , //wr_fifo的写使能信号
    input   [15:0]  wfifo_din           , //写入到wr_fifo中的数据
    output          wrfifo_full         , //写FIFO为满信号
    input           rd_clk              , //rd_fifo的读时钟信号
    input           rfifo_rden          , //rd_fifo的读使能信号
    output          rdfifo_empty        , //rd_fifo的为空信号
    output  [15:0]  rfifo_dout           //rd_fifo读出的数据信号                                      
);

    //localparam 
    localparam IDLE        = 4'b0001;   //空闲状态
    localparam DDR3_DONE   = 4'b0010;   //DDR3初始化完成状态
    localparam WRITE       = 4'b0100;   //读FIFO保持状态
    localparam READ        = 4'b1000;   //写FIFO保持状态

    reg [3:0]   state       ;       //状态机
    reg         wr_rst      ;       //输入源帧复位标志信号    
    reg         wr_load_d0  ;
    reg         wr_load_d1  ;  
    reg         rd_rst      ;               //输出源帧复位标志
    reg         rd_rst_h    ;
    reg         rd_load_d0  ;
    reg         rd_load_d1  ;
    reg         rd_load_d2, wr_load_d2;

    reg [27:0]  app_addr_wr      ;   //DDR3写地址
    reg [27:0]  app_addr_wr_min_a;    //写DDR3的起始地址
    reg [27:0]  app_addr_wr_max_a;    //写DDR3的结束地址
    reg [7:0]   wr_addr_cnt      ;   // burst beats, bounded by 8-bit length (1..255)
    reg [27:0]  app_addr_rd      ;          //DDR3读地址
    reg [7:0]   rd_addr_cnt      ;          // burst beats, not a DDR address
    reg [27:0]  app_addr_rd_min_a;    //读DDR3的起始地址
    reg [27:0]  app_addr_rd_max_a;    //读DDR3的结束地址

    reg [7:0]   rd_bust_len_a    ;        //从DDR3中读数据时的突发长度
    reg [7:0]   wr_bust_len_a    ;        //从DDR3中写数据时的突发长度
    wire [7:0] wr_last = wr_bust_len_a - 8'd1;
    wire [7:0] rd_last = rd_bust_len_a - 8'd1;

    //FIFO
    wire [9:0]  wfifo_rcount;
    wire [9:0]  rfifo_wcount;
    wire        rfifo_wren  ; 

    reg  [15:0]  rd_load_d         ;  //由输出源场信号移位拼接得到           
    reg  [15:0]  wr_load_d         ;  //由输入源场信号移位拼接得到 
    reg          wrfifo_load_d0    ;
    reg          rdfifo_load_d0    ;
    reg          rdfifo_rst_h      ;  //rfifo复位信号，高有效
    reg          wfifo_rst_h       ;  //wfifo复位信号，高有效

    reg          wfifo_rden        ; // registered pop: cuts state decode -> RAM CEB
    wire [127:0] wfifo_dout        ; //从wr_fifo中读出的数据，需要写入进DDR中
    wire [127:0] rfifo_din         ; //写入rd_fifo中的数据
    wire wfifo_empty;
    reg read_rst_done;
    reg raddr_rst_h;
    reg [10:0] raddr_rst_h_cnt;
    // Prefetch from the FWFT FIFO independently of the DDR ready signals.
    // This cuts cmd_ready -> FIFO RAM read-enable without assuming ready will
    // remain asserted. A pending word is held until BOTH DDR channels accept it.
    reg [127:0] write_word;
    reg write_pending;
    wire write_accept = (state == WRITE) && write_pending && app_rdy && app_wdf_rdy;
    reg [127:0] read_word;
    reg read_valid;

    // One outstanding pop at a time. The FIFO is FWFT, so capture Q on the
    // SAME edge that consumes the registered RdEn. Do not issue another pop
    // while that word is pending at the DDR interface. This adds one UI clock
    // per beat but preserves data/address order under either ready stall.
    always @(posedge ui_clk or negedge rst_n) begin
        if (!rst_n) wfifo_rden <= 0;
        else wfifo_rden <= (state == WRITE) && !write_pending &&
                           !wfifo_rden && !wfifo_empty;
    end
    always @(posedge ui_clk or negedge rst_n) begin
        if (!rst_n) begin write_word <= 0; write_pending <= 0; end
        else if (state != WRITE) write_pending <= 0;
        else if (write_accept) write_pending <= 0;
        else if (wfifo_rden) begin
            write_word <= wfifo_dout;
            write_pending <= 1;
        end
    end
    // Keep data and valid together. The extra cycle is internal to the video
    // FIFO; it does not change the DDR command/response protocol.
    always @(posedge ui_clk or negedge rst_n) begin
        if (!rst_n) begin read_word <= 0; read_valid <= 0; end
        else begin
            read_word <= app_rd_data;
            read_valid <= app_rd_data_valid && !raddr_rst_h;
        end
    end


    //在写状态且写有效,或者在读状态，此时使能信号为高，其他情况为低
    assign app_en = ((write_accept)
                    ||(state == READ && app_rdy)) ? 1'b1:1'b0;

    //在写状态且写有效，此时拉高写使能
    assign app_wdf_wren = write_accept;

    //由于我们DDR3芯片时钟和用户时钟的分频选择4:1，突发长度为8，故两个信号相同
    assign app_wdf_end = app_wdf_wren; 

    //处于读的时候命令值为1，其他时候命令值为0
    assign app_cmd = (state == READ) ? 3'd1 :3'd0; 

    //对输入的更新信号进行打拍处理
    always @(posedge ui_clk or negedge rst_n)  begin
        if(~rst_n)begin
            rd_load_d0 <= 0;
            rd_load_d1 <= 0; 
            wr_load_d0 <= 0; 
            wr_load_d1 <= 0;
            rd_load_d2 <= 0;
            wr_load_d2 <= 0;
        end   
        else begin
            rd_load_d0 <= rd_load;
            rd_load_d1 <= rd_load_d0;  
            wr_load_d0 <= wr_load; 
            wr_load_d1 <= wr_load_d0;
            rd_load_d2 <= rd_load_d1;
            wr_load_d2 <= wr_load_d1;
        end    
    end 

    //对异步信号进行打拍处理
    always @(posedge ui_clk or negedge rst_n)  begin
        if(~rst_n)begin
            app_addr_rd_min_a <= 0;
            app_addr_rd_max_a <= 0; 
            rd_bust_len_a <= 0; 
            app_addr_wr_min_a <= 0;  
            app_addr_wr_max_a <= 0; 
            wr_bust_len_a <= 0;                            
        end   
        else begin
            app_addr_rd_min_a <= app_addr_rd_min;
            app_addr_rd_max_a <= app_addr_rd_max; 
            rd_bust_len_a <= rd_bust_len; 
            app_addr_wr_min_a <= app_addr_wr_min;  
            app_addr_wr_max_a <= app_addr_wr_max; 
            wr_bust_len_a <= wr_bust_len;                    
        end    
    end


    //输入源复位标志信号
    always@(posedge ui_clk or negedge rst_n) begin
        if(~rst_n)
            wr_rst <= 0;
        else if(wr_load_d1 && !wr_load_d2)
            wr_rst <= 1;
        else if(state == DDR3_DONE)
            wr_rst <=0;
    end

    //对输出源做个帧复位标志 
    always @(posedge ui_clk or negedge rst_n)  begin
        if(~rst_n)
            rd_rst <= 0;                
        else if(rd_load_d1 && !rd_load_d2)
            rd_rst <= 1;               
        else
            rd_rst <= 0;           
    end



    always @(posedge ui_clk or negedge rst_n)  begin
    if(~rst_n)
        raddr_rst_h <= 1'b0;
    else if(rd_load_d1 && !rd_load_d2)
        raddr_rst_h <= 1'b1;
    else if(read_rst_done)   
        raddr_rst_h <= 1'b0;
    else
        raddr_rst_h <= raddr_rst_h;              
    end


    //对输出源的帧复位脉冲进行计数 
    always @(posedge ui_clk or negedge rst_n)  begin
        if(~rst_n)
            raddr_rst_h_cnt <= 11'b0;
        else if(raddr_rst_h && raddr_rst_h_cnt != 11'h7ff)
            raddr_rst_h_cnt <= raddr_rst_h_cnt + 1'b1;
        else if(!raddr_rst_h)
            raddr_rst_h_cnt <= 11'b0;            
    end 
 
    //将数据读写地址赋给ddr地址
    always @(*)  begin
        if(~rst_n)
            app_addr = 0;
        else if(state == READ )
            app_addr = {3'b0,app_addr_rd[24:0]};
        else
            app_addr = {3'b0,app_addr_wr[24:0]};
    end

    //DDR3读写逻辑实现
    always @(posedge ui_clk or negedge rst_n) begin
        if(~rst_n) begin
            state <= IDLE;
            wr_addr_cnt <= 24'd0;
            app_addr_wr <= 28'd0;
            app_addr_rd <= 28'd0;
            rd_addr_cnt <= 24'd0;
            read_rst_done <= 1'd0;
        end
        else begin
            read_rst_done <= 1'b0; // one-cycle acknowledgement, never stale across frames
            case(state)
                IDLE:begin
                    if(init_calib_complete)
                        state <= DDR3_DONE;
                    else
                        state <= IDLE;
                end

                DDR3_DONE:begin
                    if(wr_rst) begin   //当检测到写入更新标志之后，对地址进行复位计数
                        state <= DDR3_DONE;
                        wr_addr_cnt <= 24'd0;
                        app_addr_wr <= app_addr_wr_min_a;
                    end
                    else if((app_addr_rd >= app_addr_rd_max_a) ) begin// exclusive end address
                        state <= DDR3_DONE;
                        rd_addr_cnt <= 24'd0;
                        app_addr_rd <= app_addr_rd_min_a;
                    end
                    else if(app_addr_wr >= app_addr_wr_max_a) begin // exclusive end address
                        state <=DDR3_DONE;
                        wr_addr_cnt <= 24'd0;
                        app_addr_wr <= app_addr_wr_min_a;
                    end
                    else if(wr_bust_len_a != 0 && wfifo_rcount >= wr_bust_len_a) begin
                        state <= WRITE; //跳至写操作
                        wr_addr_cnt <= 24'd0;
                        app_addr_wr <= app_addr_wr; 
                    end
                    else if(raddr_rst_h)begin           //当帧复位到来时，对寄存器进行复位 
                        if(raddr_rst_h_cnt >= 11'd200 && rd_bust_len_a != 0)begin
                            state <= READ;         //保证读fifo在复位时不回写入数据
                            rd_addr_cnt  <= 24'd0;      
                            app_addr_rd <= app_addr_rd_min_a; 
                            read_rst_done <= 1'd1;
                        end
                        else begin
                            state <= DDR3_DONE;
                            rd_addr_cnt  <= 24'd0;      
                            app_addr_rd <= app_addr_rd;                                
                        end                                
                    end 
                    else if(rd_bust_len_a != 0 && rfifo_wcount <= rd_bust_len_a)begin
                        read_rst_done <= 1'd0;
                        state <= READ;                              //跳到读操作
                        rd_addr_cnt <= 24'd0;
                        app_addr_rd <= app_addr_rd;  //读地址不变
                    end
                    else begin
                        state <= state;
                        wr_addr_cnt <= 24'd0;
                        rd_addr_cnt <= 24'd0;
                    end
                end
                    WRITE:  begin
                        if((wr_addr_cnt == wr_last) && write_accept) begin
                            state <= DDR3_DONE;
                            app_addr_wr <= app_addr_wr + 8; //DDR突发长度为8，一次写入8个数据
                        end
                        else if(write_accept) begin //写条件满足
                            wr_addr_cnt <= wr_addr_cnt + 1'd1;
                            app_addr_wr <= app_addr_wr + 8;
                        end
                        else begin //写条件不满足
                            wr_addr_cnt <= wr_addr_cnt;
                            app_addr_wr <= app_addr_wr;
                        end
                    end
                    READ: begin
                        if((rd_addr_cnt == rd_last) && app_rdy ) begin
                            state <= DDR3_DONE;
                            app_addr_rd <= app_addr_rd + 8;
                        end
                        else if(app_rdy) begin
                            rd_addr_cnt <= rd_addr_cnt + 1'd1; //用户地址计数器每次加一
                            app_addr_rd <= app_addr_rd + 8;
                        end
                        else begin
                            rd_addr_cnt <= rd_addr_cnt;
                            app_addr_rd <= app_addr_rd;
                        end
                    end
                    default:begin
                        state <= IDLE;
                        wr_addr_cnt <= 24'd0;
                        rd_addr_cnt <= 24'd0;
                        
                    end
            endcase
        end
    end

    assign rfifo_wren = read_valid && !raddr_rst_h;
    assign app_wdf_data = write_word;
    video_halfcycle_data #(.WIDTH(128)) read_ram_retime(
        .clk(ui_clk),.reset_n(rst_n),.data_in(read_word),.data_out(rfifo_din));
    //对输出源场信号进行移位寄存
    always @(posedge ui_clk or negedge rst_n) begin
        if(!rst_n) begin
            rd_load_d <= 1'b0;
            rdfifo_load_d0 <= 1'b0;
        end
        else begin
            rd_load_d <= {rd_load_d[14:0],rdfifo_load_d0};  
            rdfifo_load_d0 <= rd_load; 
        end   
    end 

    //产生一段复位电平，满足fifo复位时序  
    always @(posedge ui_clk or negedge rst_n) begin
        if(!rst_n)
            rdfifo_rst_h <= 1'b0;
        else if(rd_load_d[0] && !rd_load_d[14])
            rdfifo_rst_h <= 1'b1;   
        else
            rdfifo_rst_h <= 1'b0;              
    end  

    // Assert asynchronously, release only on this domain's clock.
    wire wr_reset_n;
    reset_release write_reset(.clk(wr_clk),.async_n(rst_n),.reset_n(wr_reset_n));
    //对输入源场信号进行移位寄存
    always @(posedge wr_clk or negedge wr_reset_n) begin
        if(!wr_reset_n)begin
            wrfifo_load_d0 <= 1'b0;
            wr_load_d  <= 16'b0;        
        end     
        else begin
            wrfifo_load_d0 <= wr_load;
            wr_load_d <= {wr_load_d[14:0],wrfifo_load_d0};      
        end                 
    end  

    //产生一段复位电平，满足fifo复位时序 
    always @(posedge wr_clk or negedge wr_reset_n) begin
        if(!wr_reset_n)
        wfifo_rst_h <= 1'b0;          
        else if(wr_load_d[0] && !wr_load_d[15])
        wfifo_rst_h <= 1'b1;       
        else
        wfifo_rst_h <= 1'b0;                      
    end  

	rd_data_fifo rd_data_fifo(
		.Data(rfifo_din), //input [127:0] Data
		.Reset(~rst_n|raddr_rst_h), //input Reset
		.WrClk(ui_clk), //input WrClk
		.RdClk(rd_clk), //input RdClk
		.WrEn(rfifo_wren), //input WrEn
		.RdEn(rfifo_rden), //input RdEn
		.Wnum(rfifo_wcount), //output [9:0] Wnum
		.Rnum(), //output [12:0] Rnum
		.Almost_Empty(), //output Almost_Empty
		.Almost_Full(), //output Almost_Full
		.Q(rfifo_dout), //output [15:0] Q
		.Empty(rdfifo_empty), //output Empty
		.Full() //output Full
	);

	wr_data_fifo wr_data_fifo(
		.Data(wfifo_din), //input [15:0] Data
		.Reset(~wr_reset_n|wfifo_rst_h), //input Reset
		.WrClk(wr_clk), //input WrClk
		.RdClk(ui_clk), //input RdClk
		.WrEn(wfifo_wren), //input WrEn
		.RdEn(wfifo_rden), //input RdEn
		.Wnum(), //output [12:0] Wnum
		.Rnum(wfifo_rcount), //output [9:0] Rnum
		.Almost_Empty(), //output Almost_Empty
		.Almost_Full(), //output Almost_Full
		.Q(wfifo_dout), //output [127:0] Q
		.Empty(wfifo_empty), //output Empty
		.Full(wrfifo_full) //output Full
	);

endmodule
