/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ov5640_init_table_rgb
// Description   : OV5640初始化寄存器表(RGB模式专用)
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module ov5640_init_table_rgb #(
  parameter DATA_WIDTH      = 24,
  parameter ADDR_WIDTH      = 8,
  parameter IMAGE_WIDTH     = 16'd640,
  parameter IMAGE_HEIGHT    = 16'd480,
  parameter IMAGE_FLIP_EN   = 1'b0,
  parameter IMAGE_MIRROR_EN = 1'b0
)
(
  clk,
  addr,
  q
);
  input clk;
  input [(ADDR_WIDTH-1):0] addr;
  output reg [(DATA_WIDTH-1):0] q /* synthesis syn_romstyle="logic" */;

  localparam IMAGE_FLIP_DAT   = IMAGE_FLIP_EN ? 8'h47 : 8'h40;
  localparam IMAGE_MIRROR_DAT = IMAGE_MIRROR_EN ? 4'h0 : 4'h7;

  // Registered case ROM: GW5AT-60B has no SSRAM. Values/order unchanged.
  always @(posedge clk) begin
    case (addr)
    8'd0: q <= 24'h3103_11; // system clock from pad, bit[1]
    8'd1: q <= 24'h3008_82; // software reset, bit[7]
    //delay 5ms
    8'd2: q <= 24'h3008_42; // software power down, bit[6]
    8'd3: q <= 24'h3103_03; // system clock from PLL, bit[1]
    8'd4: q <= 24'h3017_ff; // FREX, Vsync, HREF, PCLK, D[9:6] output enable
    8'd5: q <= 24'h3018_ff; // D[5:0], GPIO[1:0] output enable
    8'd6: q <= 24'h3034_1a; // MIPI 10-bit
    8'd7: q <= 24'h3037_13; // PLL root divider, bit[4], PLL pre-divider, bit[3:0]
    8'd8: q <= 24'h3108_01; // PCLK root divider, bit[5:4], SCLK2x root divider, bit[3:2]
    8'd9: q <= 24'h3630_36; // SCLK root divider, bit[1:0]
    8'd10: q <= 24'h3631_0e;
    8'd11: q <= 24'h3632_e2;
    8'd12: q <= 24'h3633_12;
    8'd13: q <= 24'h3621_e0;
    8'd14: q <= 24'h3704_a0;
    8'd15: q <= 24'h3703_5a;
    8'd16: q <= 24'h3715_78;
    8'd17: q <= 24'h3717_01;
    8'd18: q <= 24'h370b_60;
    8'd19: q <= 24'h3705_1a;
    8'd20: q <= 24'h3905_02;
    8'd21: q <= 24'h3906_10;
    8'd22: q <= 24'h3901_0a;
    8'd23: q <= 24'h3731_12;
    8'd24: q <= 24'h3600_08; // VCM control
    8'd25: q <= 24'h3601_33; // VCM control
    8'd26: q <= 24'h302d_60; // system control
    8'd27: q <= 24'h3620_52;
    8'd28: q <= 24'h371b_20;
    8'd29: q <= 24'h471c_50;
    8'd30: q <= 24'h3a13_43; // pre-gain = 1.047x
    8'd31: q <= 24'h3a18_00; // gain ceiling
    8'd32: q <= 24'h3a19_f8; // gain ceiling = 15.5x
    8'd33: q <= 24'h3635_13;
    8'd34: q <= 24'h3636_03;
    8'd35: q <= 24'h3634_40;
    8'd36: q <= 24'h3622_01;
    // 50/60Hz detection 50/60Hz 灯光条纹过滤
    8'd37: q <= 24'h3c01_34; // Band auto, bit[7]
    8'd38: q <= 24'h3c04_28; // threshold low sum
    8'd39: q <= 24'h3c05_98; // threshold high sum
    8'd40: q <= 24'h3c06_00; // light meter 1 threshold[15:8]
    8'd41: q <= 24'h3c07_08; // light meter 1 threshold[7:0]
    8'd42: q <= 24'h3c08_00; // light meter 2 threshold[15:8]
    8'd43: q <= 24'h3c09_1c; // light meter 2 threshold[7:0]
    8'd44: q <= 24'h3c0a_9c; // sample number[15:8]
    8'd45: q <= 24'h3c0b_40; // sample number[7:0]
    8'd46: q <= 24'h3810_00; // Timing Hoffset[11:8]
    8'd47: q <= 24'h3811_10; // Timing Hoffset[7:0]
    8'd48: q <= 24'h3812_00; // Timing Voffset[10:8]
    8'd49: q <= 24'h3708_64;
    8'd50: q <= 24'h4001_02; // BLC start from line 2
    8'd51: q <= 24'h4005_1a; // BLC always update
    8'd52: q <= 24'h3000_00; // enable blocks
    8'd53: q <= 24'h3004_ff; // enable clocks
    8'd54: q <= 24'h300e_58; // MIPI power down, DVP enable
    8'd55: q <= 24'h302e_00;
    8'd56: q <= 24'h4300_61; // RGB565   //h4300_6f(千兆网模式)--------------------------------------------------------
    8'd57: q <= 24'h501f_01; // RGB565
    8'd58: q <= 24'h440e_00;
    8'd59: q <= 24'h5000_a7; // Lenc on, raw gamma on, BPC on, WPC on, CIP on
    // AEC target 自动曝光控制
    8'd60: q <= 24'h3a0f_30; // stable range in high
    8'd61: q <= 24'h3a10_28; // stable range in low
    8'd62: q <= 24'h3a1b_30; // stable range out high
    8'd63: q <= 24'h3a1e_26; // stable range out low
    8'd64: q <= 24'h3a11_60; // fast zone high
    8'd65: q <= 24'h3a1f_14; // fast zone low
    // Lens correction for   镜头补偿
    8'd66: q <= 24'h5800_23;
    8'd67: q <= 24'h5801_14;
    8'd68: q <= 24'h5802_0f;
    8'd69: q <= 24'h5803_0f;
    8'd70: q <= 24'h5804_12;
    8'd71: q <= 24'h5805_26;
    8'd72: q <= 24'h5806_0c;
    8'd73: q <= 24'h5807_08;
    8'd74: q <= 24'h5808_05;
    8'd75: q <= 24'h5809_05;
    8'd76: q <= 24'h580a_08;
    8'd77: q <= 24'h580b_0d;
    8'd78: q <= 24'h580c_08;
    8'd79: q <= 24'h580d_03;
    8'd80: q <= 24'h580e_00;
    8'd81: q <= 24'h580f_00;
    8'd82: q <= 24'h5810_03;
    8'd83: q <= 24'h5811_09;
    8'd84: q <= 24'h5812_07;
    8'd85: q <= 24'h5813_03;
    8'd86: q <= 24'h5814_00;
    8'd87: q <= 24'h5815_01;
    8'd88: q <= 24'h5816_03;
    8'd89: q <= 24'h5817_08;
    8'd90: q <= 24'h5818_0d;
    8'd91: q <= 24'h5819_08;
    8'd92: q <= 24'h581a_05;
    8'd93: q <= 24'h581b_06;
    8'd94: q <= 24'h581c_08;
    8'd95: q <= 24'h581d_0e;
    8'd96: q <= 24'h581e_29;
    8'd97: q <= 24'h581f_17;
    8'd98: q <= 24'h5820_11;
    8'd99: q <= 24'h5821_11;
    8'd100: q <= 24'h5822_15;
    8'd101: q <= 24'h5823_28;
    8'd102: q <= 24'h5824_46;
    8'd103: q <= 24'h5825_26;
    8'd104: q <= 24'h5826_08;
    8'd105: q <= 24'h5827_26;
    8'd106: q <= 24'h5828_64;
    8'd107: q <= 24'h5829_26;
    8'd108: q <= 24'h582a_24;
    8'd109: q <= 24'h582b_22;
    8'd110: q <= 24'h582c_24;
    8'd111: q <= 24'h582d_24;
    8'd112: q <= 24'h582e_06;
    8'd113: q <= 24'h582f_22;
    8'd114: q <= 24'h5830_40;
    8'd115: q <= 24'h5831_42;
    8'd116: q <= 24'h5832_24;
    8'd117: q <= 24'h5833_26;
    8'd118: q <= 24'h5834_24;
    8'd119: q <= 24'h5835_22;
    8'd120: q <= 24'h5836_22;
    8'd121: q <= 24'h5837_26;
    8'd122: q <= 24'h5838_44;
    8'd123: q <= 24'h5839_24;
    8'd124: q <= 24'h583a_26;
    8'd125: q <= 24'h583b_28;
    8'd126: q <= 24'h583c_42;
    8'd127: q <= 24'h583d_ce; // lenc BR offset
    // AWB 自动白平衡
    8'd128: q <= 24'h5180_ff; // AWB B block
    8'd129: q <= 24'h5181_f2; // AWB control
    8'd130: q <= 24'h5182_00; // [7:4] max local counter, [3:0] max fast counter
    8'd131: q <= 24'h5183_14; // AWB advanced
    8'd132: q <= 24'h5184_25;
    8'd133: q <= 24'h5185_24;
    8'd134: q <= 24'h5186_09;
    8'd135: q <= 24'h5187_09;
    8'd136: q <= 24'h5188_09;
    8'd137: q <= 24'h5189_75;
    8'd138: q <= 24'h518a_54;
    8'd139: q <= 24'h518b_e0;
    8'd140: q <= 24'h518c_b2;
    8'd141: q <= 24'h518d_42;
    8'd142: q <= 24'h518e_3d;
    8'd143: q <= 24'h518f_56;
    8'd144: q <= 24'h5190_46;
    8'd145: q <= 24'h5191_f8; // AWB top limit
    8'd146: q <= 24'h5192_04; // AWB bottom limit
    8'd147: q <= 24'h5193_70; // red limit
    8'd148: q <= 24'h5194_f0; // green limit
    8'd149: q <= 24'h5195_f0; // blue limit
    8'd150: q <= 24'h5196_03; // AWB control
    8'd151: q <= 24'h5197_01; // local limit
    8'd152: q <= 24'h5198_04;
    8'd153: q <= 24'h5199_12;
    8'd154: q <= 24'h519a_04;
    8'd155: q <= 24'h519b_00;
    8'd156: q <= 24'h519c_06;
    8'd157: q <= 24'h519d_82;
    8'd158: q <= 24'h519e_38; // AWB control
    // Gamma 伽玛曲线
    8'd159: q <= 24'h5480_01; // Gamma bias plus on, bit[0]
    8'd160: q <= 24'h5481_08;
    8'd161: q <= 24'h5482_14;
    8'd162: q <= 24'h5483_28;
    8'd163: q <= 24'h5484_51;
    8'd164: q <= 24'h5485_65;
    8'd165: q <= 24'h5486_71;
    8'd166: q <= 24'h5487_7d;
    8'd167: q <= 24'h5488_87;
    8'd168: q <= 24'h5489_91;
    8'd169: q <= 24'h548a_9a;
    8'd170: q <= 24'h548b_aa;
    8'd171: q <= 24'h548c_b8;
    8'd172: q <= 24'h548d_cd;
    8'd173: q <= 24'h548e_dd;
    8'd174: q <= 24'h548f_ea;
    8'd175: q <= 24'h5490_1d;
    // color matrix 色彩矩阵
    8'd176: q <= 24'h5381_1e; // CMX1 for Y
    8'd177: q <= 24'h5382_5b; // CMX2 for Y
    8'd178: q <= 24'h5383_08; // CMX3 for Y
    8'd179: q <= 24'h5384_0a; // CMX4 for U
    8'd180: q <= 24'h5385_7e; // CMX5 for U
    8'd181: q <= 24'h5386_88; // CMX6 for U
    8'd182: q <= 24'h5387_7c; // CMX7 for V
    8'd183: q <= 24'h5388_6c; // CMX8 for V
    8'd184: q <= 24'h5389_10; // CMX9 for V
    8'd185: q <= 24'h538a_01; // sign[9]
    8'd186: q <= 24'h538b_98; // sign[8:1]
    // UV adjust UV 色彩饱和度调整
    8'd187: q <= 24'h5580_06; // saturation on, bit[1]
    8'd188: q <= 24'h5583_40;
    8'd189: q <= 24'h5584_10;
    8'd190: q <= 24'h5589_10;
    8'd191: q <= 24'h558a_00;
    8'd192: q <= 24'h558b_f8;
    8'd193: q <= 24'h501d_40; // enable manual offset of contrast
    // CIP 锐化和降噪
    8'd194: q <= 24'h5300_08; // CIP sharpen MT threshold 1
    8'd195: q <= 24'h5301_30; // CIP sharpen MT threshold 2
    8'd196: q <= 24'h5302_10; // CIP sharpen MT offset 1
    8'd197: q <= 24'h5303_00; // CIP sharpen MT offset 2
    8'd198: q <= 24'h5304_08; // CIP DNS threshold 1
    8'd199: q <= 24'h5305_30; // CIP DNS threshold 2
    8'd200: q <= 24'h5306_08; // CIP DNS offset 1
    8'd201: q <= 24'h5307_16; // CIP DNS offset 2
    8'd202: q <= 24'h5309_08; // CIP sharpen TH threshold 1
    8'd203: q <= 24'h530a_30; // CIP sharpen TH threshold 2
    8'd204: q <= 24'h530b_04; // CIP sharpen TH offset 1
    8'd205: q <= 24'h530c_06; // CIP sharpen TH offset 2
    8'd206: q <= 24'h5025_00;
    8'd207: q <= 24'h3008_02; // wake up from standby, bit[6]

    // 12824'h720, 30fps
    // input clock 24Mhz, PCLK 42Mhz
    8'd208: q <= 24'h3035_21; // PLL  21:30fps  41:15fps  81:7.5fps
    8'd209: q <= 24'h3036_69; // PLL
    8'd210: q <= 24'h3c07_07; // lightmeter 1 threshold[7:0]
    8'd211: q <= {16'h3820, IMAGE_FLIP_DAT}; // flip
    8'd212: q <= {20'h38210, IMAGE_MIRROR_DAT}; // no mirror
    8'd213: q <= 24'h3814_31; // timing X inc
    8'd214: q <= 24'h3815_31; // timing Y inc
    8'd215: q <= 24'h3800_00; // HS
    8'd216: q <= 24'h3801_00; // HS
    8'd217: q <= 24'h3802_00; // VS
    8'd218: q <= 24'h3803_fa; // VS
    8'd219: q <= 24'h3804_0a; // HW SET_OV5640 +  HE}
    8'd220: q <= 24'h3805_3f; // HW SET_OV5640 +  HE}
    8'd221: q <= 24'h3806_06; // VH SET_OV5640 +  VE}
    8'd222: q <= 24'h3807_a9; // VH SET_OV5640 +  VE}
    8'd223: q <= {16'h3808, IMAGE_WIDTH[15:8]}; // DVPHO  (<1280>500) (<640>280)IMAGE_WIDTH
    8'd224: q <= {16'h3809, IMAGE_WIDTH[ 7:0]}; // DVPHO
    8'd225: q <= {16'h380a, IMAGE_HEIGHT[15:8]}; // DVPVO  (<720>2d0)  (<480>1e0)IMAGE_HEIGHT
    8'd226: q <= {16'h380b, IMAGE_HEIGHT[ 7:0]}; // DVPHO
    8'd227: q <= 24'h380c_07; // HTS
    8'd228: q <= 24'h380d_64; // HTS
    8'd229: q <= 24'h380e_02; // VTS
    8'd230: q <= 24'h380f_e4; // VTS
    8'd231: q <= 24'h3813_04; // timing V offset
    8'd232: q <= 24'h3618_00;
    8'd233: q <= 24'h3612_29;
    8'd234: q <= 24'h3709_52;
    8'd235: q <= 24'h370c_03;
    8'd236: q <= 24'h3a02_02; // 60Hz max exposure
    8'd237: q <= 24'h3a03_e0; // 60Hz max exposure
    8'd238: q <= 24'h3a14_02; // 50Hz max exposure
    8'd239: q <= 24'h3a15_e0; // 50Hz max exposure
    8'd240: q <= 24'h4004_02; // BLC line number
    8'd241: q <= 24'h3002_1c; // reset JFIFO, SFIFO, JPG
    8'd242: q <= 24'h3006_c3; // disable clock of JPEG2x, JPEG
    8'd243: q <= 24'h4713_03; // JPEG mode 3
    8'd244: q <= 24'h4407_04; // Quantization scale
    8'd245: q <= 24'h460b_37;
    8'd246: q <= 24'h460c_20;
    8'd247: q <= 24'h4837_16; // MIPI global timing
    8'd248: q <= 24'h3824_04; // PCLK manual divider
    8'd249: q <= 24'h5001_83; // SDE on, CMX on, AWB on
    8'd250: q <= 24'h3503_00; // AEC/AGC on
    8'd251: q <= 24'h4740_20; // VS 1
    8'd252: q <= 24'h503D_00; // VS 1
      default: q <= 24'h000000;
    endcase
  end

endmodule
