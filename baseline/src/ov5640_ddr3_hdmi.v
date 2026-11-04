// Adapted from the supplied native GW5AT-60B ch47 project.
module camera_hdmi_core(
    input wire [3:0] osd_enc_a,osd_enc_b,
    //System clock reset
    input       clk50M        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入
    input       test_pattern  , // SW0 high: FPGA-generated HDMI color bars
    input osd_telemetry_tx, osd_host_rx, osd_imu_rx,
    input [3:0] osd_key_led,
    input osd_gpio_selected,
    output      hdmi_oe       , // HDMI1 +5V/DDC enable, H5

    //camera interface
    inout       camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    //led
    output [3:0]led           ,

    //hdmi output
	output       	tmds_clk_p ,
	output       	tmds_clk_n ,
	output 	[2:0]   tmds_data_p,    //rgb
	output 	[2:0]   tmds_data_n,     //rgb

    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

);
    //Set IMAGE Size  
    parameter IMAGE_WIDTH  = 1280;
    parameter IMAGE_HEIGHT = 720;

    wire          loc_clk24m;
    wire          g_rst_p;
    wire          loc_clk400m;
    wire          pll_lock;
    wire          loc_clk50m;
    wire          pll_locked;
    //camera
    wire          camera_init_done;
    wire          camera_failed;
    wire [15:0]   sensor_id, failed_register;
    wire          wr_load, wrfifo_wren;
    wire          wrfifo_full, rdfifo_empty;
    wire          system_reset_n, init_reset_n, capture_reset_n, display_reset_n;
    wire init_calib_complete;
    wire clk_vga; // Native IP: 75 MHz (not 74.25 MHz).
    reg [15:0]    powerup = 0;
    always @(posedge clk50M) if(!(&powerup)) powerup <= powerup + 1'b1;
    assign system_reset_n = reset_n && (&powerup);
    reset_release init_reset(.clk(clk50M), .async_n(system_reset_n && pll_locked), .reset_n(init_reset_n));
    reset_release capture_reset(.clk(camera_pclk),
        .async_n(system_reset_n && pll_locked && camera_init_done && init_calib_complete), .reset_n(capture_reset_n));
    reset_release display_reset(.clk(clk_vga), .async_n(system_reset_n && pll_locked), .reset_n(display_reset_n));
    assign hdmi_oe = system_reset_n;
    wire          pclk_bufg_o;

    //ddr
	wire fifo_aclr;
	wire fifo_wrreq;
	wire [15:0] fifo_wrdata;
    
    //hdmi
	wire clk_vgax5; // Native IP: 375 MHz.
    wire hdmi_pll_lock;

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;
 
	wire video_hs;
	wire video_vs;
	wire video_de;
    wire osd_frame_begin;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;


    assign  g_rst_p = ~pll_locked;

    // Active-high board LED4..7: PLL, DDR ready, sensor ready, latched error.
    // Verified against board schematic sheet 10, not old example comments.
    wire video_error;
    assign led = {(camera_failed | video_error),camera_init_done,init_calib_complete,pll_locked};
    

        camera_pll your_instance_name(
        .clkin(clk50M), //input  clkin
        .clkout0(clk_vgax5), //output  clkout0
        .clkout1(loc_clk24m), //output  clkout1
        .clkout2(loc_clk50m), //output  clkout2
        .lock(pll_locked), //output  lock
        .mdclk(clk50M), //input  mdclk
        .reset(~system_reset_n) //input  reset
    );


    CLKDIV u_clkdiv
    (.RESETN(pll_locked)
    ,.HCLKIN(clk_vgax5) //clk  x5
    ,.CLKOUT(clk_vga)    //clk  x1
    ,.CALIB (1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";


    assign camera_xclk = loc_clk24m;
    //摄像头
    camera_setup
    #(
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ),// 图片宽度
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ) // 图片高度
    )camera_init
    (
        .Clk         (clk50M          ),
        .Rst_n       (init_reset_n    ),
        .Init_Done   (camera_init_done ),
        .failed      (camera_failed),
        .sensor_id   (sensor_id),
        .failed_register(failed_register),
        .camera_rst_n(camera_rst_n     ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );
    
    assign pclk_bufg_o = camera_pclk;

    // Matched DVP input delays, identical to board-tested HD build E.
    wire [7:0] delayed_data;
    wire delayed_href,delayed_vsync;
    genvar data_bit;
    generate for(data_bit=0;data_bit<8;data_bit=data_bit+1)begin: dvp_delay
      IODELAY #(.C_STATIC_DLY(240),.DYN_DLY_EN("FALSE"),.ADAPT_EN("FALSE")) delay_cell(
        .DI(camera_data[data_bit]),.DO(delayed_data[data_bit]),.DF(),.SDTAP(1'b0),.VALUE(1'b0),.DLYSTEP(8'b0));
    end endgenerate
    IODELAY #(.C_STATIC_DLY(240),.DYN_DLY_EN("FALSE"),.ADAPT_EN("FALSE")) href_delay(
      .DI(camera_href),.DO(delayed_href),.DF(),.SDTAP(1'b0),.VALUE(1'b0),.DLYSTEP(8'b0));
    IODELAY #(.C_STATIC_DLY(240),.DYN_DLY_EN("FALSE"),.ADAPT_EN("FALSE")) vsync_delay(
      .DI(camera_vsync),.DO(delayed_vsync),.DF(),.SDTAP(1'b0),.VALUE(1'b0),.DLYSTEP(8'b0));

    DVP_Capture DVP_Capture(
        .Rst_n      (capture_reset_n ),//input
        .PCLK       (pclk_bufg_o      ),//input
        .Vsync      (delayed_vsync    ),//input
        .Href       (delayed_href     ),//input
        .Data       (delayed_data     ),//input     [7:0]

        .ImageState (fifo_aclr        ),//output reg
        .DataValid  (fifo_wrreq       ),//output
        .DataPixel  (fifo_wrdata      ),//output    [15:0]
        .DataHs     (                 ),//output
        .DataVs     (wr_load          ),//output
        .Xaddr      (                 ),//output    [11:0],start is 1
        .Yaddr      (                 )   //output    [11:0],start is 1
    );
    

    wire rdfifo_rden;
    wire wrfifo_clr;
    wire [15:0] wrfifo_din;
    wire wrfifo_clk;
    wire rdfifo_clr;
    wire rdfifo_clk;
    wire [15:0] rdfifo_dout;
    
    
    assign wrfifo_clr = fifo_aclr;
    assign wrfifo_clk = pclk_bufg_o;
    assign rdfifo_clr = 0;
    assign rdfifo_clk = clk_vga;
    assign wrfifo_wren = fifo_wrreq && !wrfifo_full && capture_reset_n;
    video_halfcycle_data #(.WIDTH(16)) capture_ram_retime(
        .clk(wrfifo_clk),.reset_n(capture_reset_n),
        .data_in(fifo_wrdata),.data_out(wrfifo_din));
    wire display_req;
    wire [23:0] camera_rgb = {rdfifo_dout[15:11],rdfifo_dout[15:13],
                             rdfifo_dout[10:5],rdfifo_dout[10:9],rdfifo_dout[4:0],rdfifo_dout[4:2]};
    video_select select_video(.pclk(camera_pclk),.pixel_clk(clk_vga),
        .capture_reset_n(capture_reset_n),.display_reset_n(display_reset_n),
        .cam_vsync(delayed_vsync),.write_valid(fifo_wrreq),.write_full(wrfifo_full),
        .test_pattern(test_pattern),.display_req(display_req),.read_empty(rdfifo_empty),
        .x(hcount),.camera_rgb(camera_rgb),.pixel_rgb(disp_data),
        .read_enable(rdfifo_rden),.error(video_error));

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0] burst_len = IMAGE_WIDTH[10:3];

    wire mdrp_inc;     
    wire [1:0] mdrp_op;     
    wire [7:0] mdrp_wdata;     
    wire [7:0] mdrp_rdata;     
    wire pll_stop;     
    reg pll_stop_r;
    reg wr;

    reg  pll_lock_r;
    reg  pll_lock_rr;

    always@(posedge clk50M or negedge system_reset_n)begin
        if(!system_reset_n) begin pll_lock_r<=0; pll_lock_rr<=0; end
        else begin pll_lock_r <= pll_lock; pll_lock_rr <= pll_lock_r; end
    end

    always@(posedge clk50M or negedge system_reset_n)
        if(!system_reset_n) pll_stop_r<=0; else pll_stop_r <= pll_stop;

    pll_mDRP_intf u_pll_mDRP_intf(
        .clk(clk50M),
        .rst_n(system_reset_n),
        .pll_lock(pll_lock_rr),
        .wr(wr),
        .mdrp_inc(mdrp_inc),
        .mdrp_op(mdrp_op),
        .mdrp_wdata(mdrp_wdata),
        .mdrp_rdata(mdrp_rdata)
    );    

    always@(posedge clk50M or negedge system_reset_n)
    if(!system_reset_n)
        wr <= 1'b0;
    else if(!pll_lock_rr)
        wr <= 1'b0;
    else if(~pll_stop & pll_stop_r)begin
        wr <= 1'b1;
    end
    else if(pll_stop & (~pll_stop_r))begin
        wr <= 1'b1;
    end
    else begin
        wr <= 1'b0;
    end

    ddr_pll ddr_pll(
        .clkin(clk50M), //input  clkin
        .clkout0(), //output  clkout0
        .clkout1(), //output  clkout1
        .clkout2(loc_clk400m), //output  clkout2
        .lock(pll_lock), //output  lock
        .mdopc(mdrp_op), //input [1:0] mdopc
        .mdainc(mdrp_inc), //input  mdainc
        .mdwdi(mdrp_wdata), //input [7:0] mdwdi
        .mdrdo(mdrp_rdata), //output [7:0] mdrdo
        .pll_init_bypass(1'b0), //input  pll_init_bypass
        .mdclk(clk50M), //input  mdclk
        .reset(~system_reset_n) //input  reset
    );

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(clk50M)                     ,      //50M board clock
        .pll_lock(pll_lock)            ,
        .pll_stop(pll_stop)                ,
        .clk_200m(loc_clk400m)            ,      //DDR3参考时钟信号
        .sys_rst_n(init_reset_n)      ,      // reset released after camera PLL lock
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(video_vs)             ,   //输出源更新信号
        .wr_load(wr_load)             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wrfifo_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .wrfifo_full(wrfifo_full),
        .rd_clk(rdfifo_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rdfifo_empty(rdfifo_empty),
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
   

    //HDMI显示
    disp_driver disp_driver(
		.ClkDisp(clk_vga),
		.Rst_n(display_reset_n),
		.Data(disp_data),
		.DataReq(display_req),
		.H_Addr(hcount),
		.V_Addr(vcount),
		.Disp_HS(video_hs),
		.Disp_VS(video_vs),
		.Disp_Red(video_r),
		.Disp_Green(video_g),
		.Disp_Blue(video_b),
        .Frame_Begin(osd_frame_begin),
		.Disp_DE(video_de),
		.Disp_PCLK()
	);
    // Read-only serial/status observer. Shared async reset restarts both sides
    // of the mailbox on S0 or PLL loss. RGB coordinates match disp_driver's FF.
    wire osd_reset_n = system_reset_n && pll_locked;
    wire osd_observer_reset_n;
    reset_release observer_reset(.clk(clk50M),.async_n(osd_reset_n),.reset_n(osd_observer_reset_n));
    wire [810:0] osd_source;
    wire [1130:0] osd_frame;
    wire [319:0] wheel_fields;
    osd_wheels wheel_observer(.clk(clk50M),.reset_n(osd_observer_reset_n),
      .phase_a(osd_enc_a),.phase_b(osd_enc_b),.ascii_fields(wheel_fields));
    wire osd_publish;
    wire [810:0] osd_degrees;
    wire degrees_done;
    osd_angles_deg degree_display(.clk(clk50M),.reset_n(osd_observer_reset_n),
      .publish(osd_publish),.source_data(osd_source),.converted(osd_degrees),.done(degrees_done),.busy());
    osd_observer observer(.clk(clk50M),.reset_n(osd_observer_reset_n),
        .telemetry_tx(osd_telemetry_tx),.host_rx(osd_host_rx),.imu_rx(osd_imu_rx),
        .key_led(osd_key_led),.gpio_selected(osd_gpio_selected),
        .snapshot(osd_source),.publish(osd_publish));
    osd_snapshot_cdc #(.WIDTH(1131)) osd_mailbox(.source_clk(clk50M),.pixel_clk(clk_vga),
        .reset_n(osd_reset_n),.publish(degrees_done),.frame_begin(osd_frame_begin),
        .source_data({wheel_fields,osd_degrees}),.frame_data(osd_frame),.busy());
    reg [11:0] osd_x,osd_y;
    reg [4:0] osd_status_meta,osd_status_sync,osd_status_frame;
    always @(posedge clk_vga or negedge display_reset_n)begin
        if(!display_reset_n)begin
            osd_x<=0;osd_y<=0;osd_status_meta<=0;osd_status_sync<=0;osd_status_frame<=0;
        end else begin
            osd_x<=hcount;osd_y<=vcount;
            osd_status_meta<={test_pattern,video_error,camera_failed,camera_init_done,init_calib_complete};
            osd_status_sync<=osd_status_meta;
            if(osd_frame_begin)osd_status_frame<=osd_status_sync;
        end
    end
    wire [23:0] osd_rgb;
    wire osd_de,osd_hs,osd_vs;
    osd_overlay overlay(.clk(clk_vga),.reset_n(display_reset_n),.x(osd_x),.y(osd_y),
        .rgb_in({video_r,video_g,video_b}),.de_in(video_de),.hs_in(video_hs),.vs_in(video_vs),
        .snapshot(osd_frame[810:0]),.wheel_fields(osd_frame[1130:811]),.video_status(osd_status_frame),
        .rgb_out(osd_rgb),.de_out(osd_de),.hs_out(osd_hs),.vs_out(osd_vs));

	dvi_encoder u_dvi_encoder(
		.pixelclk(clk_vga),// system clock
		.pixelclk5x(clk_vgax5),// system clock x5
		.rst_n(display_reset_n),// reset
		.blue_din(osd_rgb[7:0]),// Blue data in
		.green_din(osd_rgb[15:8]),// Green data in
		.red_din(osd_rgb[23:16]),// Red data in
		.hsync(osd_hs),// hsync data
		.vsync(osd_vs),// vsync data
		.de(osd_de),// data enable
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p),//rgb
		.tmds_data_n(tmds_data_n) //rgb
	);

//	DVI_TX_Top DVI_TX_Top(
//		.I_rst_n(reset_n), //input I_rst_n
//		.I_serial_clk(clk_vga * 5), //input I_serial_clk
//		.I_rgb_clk(clk_vga), //input I_rgb_clk
//		.I_rgb_vs(video_vs), //input I_rgb_vs
//		.I_rgb_hs(video_hs), //input I_rgb_hs
//		.I_rgb_de(video_de), //input I_rgb_de
//		.I_rgb_r(video_r), //input [7:0] I_rgb_r
//		.I_rgb_g(video_g), //input [7:0] I_rgb_g
//		.I_rgb_b(video_b), //input [7:0] I_rgb_b
//		.O_tmds_clk_p(tmds_clk_p), //output O_tmds_clk_p
//		.O_tmds_clk_n(tmds_clk_n), //output O_tmds_clk_n
//		.O_tmds_data_p(tmds_data_p), //output [2:0] O_tmds_data_p
//		.O_tmds_data_n(tmds_data_n) //output [2:0] O_tmds_data_n
//	);


endmodule
