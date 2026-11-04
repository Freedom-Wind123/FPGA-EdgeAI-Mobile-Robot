module ddr_pll(
    clkin,
    clkout0,
    clkout1,
    clkout2,
    lock,
    mdopc,
    mdainc,
    mdwdi,
    mdrdo,
    pll_init_bypass,
    mdclk,
    reset
);


input clkin;
output clkout0;
output clkout1;
output clkout2;
output lock;
input [1:0] mdopc;
input mdainc;
input [7:0] mdwdi;
output [7:0] mdrdo;
input pll_init_bypass;
input mdclk;
input reset;
wire [1:0] wMdOpc;
wire wMdAInc;
wire [7:0] wMdDIn;
wire [7:0] wMdQOut;
wire pll_lock;
wire pll_rst;

// Run vendor PLL startup once, then hand the dynamic register bus to the
// DDR calibration controller. Keeping PLL_INIT_BYPASS low forever discards
// MDOPC/MDAINC/MDWDI and returns constant MDRDO, so pll_stop cannot gate CLKOUT2.
reg init_handoff = 1'b0;
always @(posedge mdclk or posedge reset) begin
    if(reset) init_handoff <= 1'b0;
    else if(lock) init_handoff <= 1'b1;
end


    ddr_pll_MOD u_pll(
        .clkout1(clkout1),
        .clkout2(clkout2),
        .clkout0(clkout0),
        .lock(pll_lock),
        .mdrdo(wMdQOut),
        .clkin(clkin),
        .reset(pll_rst),
        .mdclk(mdclk),
        .mdopc(wMdOpc),
        .mdainc(wMdAInc),
        .mdwdi(wMdDIn)
    );


    PLL_INIT u_pll_init(
        .I_RST(reset),
        .O_RST(pll_rst),
        .I_LOCK(pll_lock),
        .O_LOCK(lock),
        .I_MD_CLK(mdclk),
        .O_MD_INC(wMdAInc),
        .O_MD_OPC(wMdOpc),
        .O_MD_WR_DATA(wMdDIn),
        .I_MD_RD_DATA(wMdQOut),
        .PLL_INIT_BYPASS(pll_init_bypass | init_handoff),
        .MDRDO(mdrdo),
        .MDOPC(mdopc),
        .MDAINC(mdainc),
        .MDWDI(mdwdi)
    );
    defparam u_pll_init.CLK_PERIOD = 20;
    defparam u_pll_init.MULTI_FAC = 16;


endmodule
