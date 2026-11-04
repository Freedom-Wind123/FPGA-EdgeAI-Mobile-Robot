// 64 x 5 text cells, 16 x 16 pixels each; top-left at (16,16).
// Six-stage character/row/bit pipeline; RGB/DE/HS/VS have identical latency.
module osd_overlay(
 input wire clk,reset_n,
 input wire [11:0] x,y,
 input wire [23:0] rgb_in, input wire de_in,hs_in,vs_in,
 input wire [810:0] snapshot,
 input wire [4:0] video_status, // {test_pattern,error,camera_failed,camera_ready,ddr_ready}
 input wire [319:0] wheel_fields,
 output reg [23:0] rgb_out, output reg de_out,hs_out,vs_out
);
 function automatic [7:0] sc(input [511:0] text,input integer len,input integer pos);
  if(pos<0 || pos>=len) sc=" ";else sc=text[(len-1-pos)*8+:8];
 endfunction
 function automatic [7:0] state_char(input [1:0] state,input integer pos);
  case(state)
   1:state_char=sc("LIVE",4,pos);
   2:state_char=sc("OLD ",4,pos);
   default:state_char=sc("WAIT",4,pos);
  endcase
 endfunction
 function automatic [7:0] number_char(input integer field,input [3:0] pos);
  if(snapshot[810:809]==0)number_char="-";
 else begin
  case(pos)
   0:number_char=snapshot[(field*10+0)*8+:8];
   1:number_char=snapshot[(field*10+1)*8+:8];
   2:number_char=snapshot[(field*10+2)*8+:8];
   3:number_char=snapshot[(field*10+3)*8+:8];
   4:number_char=snapshot[(field*10+4)*8+:8];
   5:number_char=snapshot[(field*10+5)*8+:8];
   6:number_char=snapshot[(field*10+6)*8+:8];
   7:number_char=snapshot[(field*10+7)*8+:8];
   8:number_char=snapshot[(field*10+8)*8+:8];
   9:number_char=snapshot[(field*10+9)*8+:8];
   default:number_char=" ";
  endcase
 end
 endfunction
 function automatic [7:0] text_char(input [2:0] row,input [5:0] col);
  begin
   text_char=" ";
   if(row==0)begin
    if(col<4)text_char=sc("YAW ",4,col);
    else if(col<14)text_char=number_char(2,col-4);
    else if(col==15)text_char=8'hb0;
   end
   if(row==1)begin
    if(col<5)text_char=sc("MODE ",5,col);
    else if(col<13)case(snapshot[801:800])
     0:text_char=sc("STOP    ",8,col-5);
     1:text_char=sc("MANUAL  ",8,col-5);
     2:text_char=sc("MAP NAV ",8,col-5);
     3:text_char=sc("FOLLOW  ",8,col-5);
    endcase
    else if(col>=14 && col<23)text_char=sc(" DEV LOCK",9,col-14);
   end
  end
 endfunction
 // Hand-authored 5x7 font, top row first. Unlisted glyphs are blank.
 function automatic [34:0] glyph(input [7:0] c);
  case(c)
   8'hb0:glyph=35'b01110_01010_01110_00000_00000_00000_00000;
   "0":glyph=35'b01110_10001_10011_10101_11001_10001_01110;
   "1":glyph=35'b00100_01100_00100_00100_00100_00100_01110;
   "2":glyph=35'b01110_10001_00001_00010_00100_01000_11111;
   "3":glyph=35'b11110_00001_00001_01110_00001_00001_11110;
   "4":glyph=35'b00010_00110_01010_10010_11111_00010_00010;
   "5":glyph=35'b11111_10000_10000_11110_00001_00001_11110;
   "6":glyph=35'b01110_10000_10000_11110_10001_10001_01110;
   "7":glyph=35'b11111_00001_00010_00100_01000_01000_01000;
   "8":glyph=35'b01110_10001_10001_01110_10001_10001_01110;
   "9":glyph=35'b01110_10001_10001_01111_00001_00001_01110;
   "A":glyph=35'b01110_10001_10001_11111_10001_10001_10001;
   "B":glyph=35'b11110_10001_10001_11110_10001_10001_11110;
   "C":glyph=35'b01111_10000_10000_10000_10000_10000_01111;
   "D":glyph=35'b11110_10001_10001_10001_10001_10001_11110;
   "E":glyph=35'b11111_10000_10000_11110_10000_10000_11111;
   "F":glyph=35'b11111_10000_10000_11110_10000_10000_10000;
   "G":glyph=35'b01111_10000_10000_10111_10001_10001_01111;
   "H":glyph=35'b10001_10001_10001_11111_10001_10001_10001;
   "I":glyph=35'b01110_00100_00100_00100_00100_00100_01110;
   "J":glyph=35'b00111_00010_00010_00010_10010_10010_01100;
   "K":glyph=35'b10001_10010_10100_11000_10100_10010_10001;
   "L":glyph=35'b10000_10000_10000_10000_10000_10000_11111;
   "M":glyph=35'b10001_11011_10101_10101_10001_10001_10001;
   "N":glyph=35'b10001_11001_10101_10011_10001_10001_10001;
   "O":glyph=35'b01110_10001_10001_10001_10001_10001_01110;
   "P":glyph=35'b11110_10001_10001_11110_10000_10000_10000;
   "Q":glyph=35'b01110_10001_10001_10001_10101_10010_01101;
   "R":glyph=35'b11110_10001_10001_11110_10100_10010_10001;
   "S":glyph=35'b01111_10000_10000_01110_00001_00001_11110;
   "T":glyph=35'b11111_00100_00100_00100_00100_00100_00100;
   "U":glyph=35'b10001_10001_10001_10001_10001_10001_01110;
   "V":glyph=35'b10001_10001_10001_10001_10001_01010_00100;
   "W":glyph=35'b10001_10001_10001_10101_10101_10101_01010;
   "X":glyph=35'b10001_10001_01010_00100_01010_10001_10001;
   "Y":glyph=35'b10001_10001_01010_00100_00100_00100_00100;
   "Z":glyph=35'b11111_00001_00010_00100_01000_10000_11111;
   "-":glyph=35'b00000_00000_00000_11111_00000_00000_00000;
   ".":glyph=35'b00000_00000_00000_00000_00000_00110_00110;
   "/":glyph=35'b00001_00001_00010_00100_01000_10000_10000;
   default:glyph=0;
  endcase
 endfunction
 wire [11:0] px=x-12'd16,py=y-12'd16;
 wire area=de_in && x>=16 && x<1040 && y>=16 && y<96;
 reg [5:0] text_col0;
 reg [2:0] text_row0,text_row1;
 reg [7:0] row_char1[0:4];
 reg [7:0] char2;
 reg [34:0] glyph3;
 reg [4:0] glyph_row4;
 reg [2:0] col_pipe[0:4],row_pipe[0:4];
 reg [4:0] area_pipe;
 reg [23:0] rgb_pipe[0:4];
 reg [23:0] color_pipe[0:4];
 reg [2:0] sync_pipe[0:4];
 function automatic [23:0] text_color(input [2:0] row);
  case(row)
   0:text_color=24'hff8000; // angles: orange
   1:text_color=24'h00dfff; // velocity: cyan
   2:text_color=24'h00ff60; // mode/communication: green
   3:text_color=24'hff40d0; // video diagnostics: magenta
   4:text_color=24'h00dfff; // wheel speed: cyan
   default:text_color=24'hff8000;
  endcase
 endfunction
 reg ink;
 always @* begin
  case(col_pipe[4])
   0:ink=glyph_row4[4];1:ink=glyph_row4[3];2:ink=glyph_row4[2];
   3:ink=glyph_row4[1];4:ink=glyph_row4[0];default:ink=0;
  endcase
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   text_col0<=0;text_row0<=0;text_row1<=0;char2<=32;glyph3<=0;glyph_row4<=0;
   area_pipe<=0;rgb_out<=0;
   for(integer i=0;i<5;i=i+1)begin
    row_char1[i]<=32;col_pipe[i]<=0;row_pipe[i]<=0;rgb_pipe[i]<=0;color_pipe[i]<=0;sync_pipe[i]<=0;
   end
   de_out<=0;hs_out<=0;vs_out<=0;
  end else begin
   text_col0<=px[9:4];text_row0<=py[6:4];
   col_pipe[0]<=px[3:1];row_pipe[0]<=py[3:1];
   area_pipe[0]<=area;rgb_pipe[0]<=rgb_in;sync_pipe[0]<={de_in,hs_in,vs_in};
   color_pipe[0]<=text_color(py[6:4]);
   for(integer i=1;i<5;i=i+1)begin
    col_pipe[i]<=col_pipe[i-1];row_pipe[i]<=row_pipe[i-1];
    area_pipe[i]<=area_pipe[i-1];rgb_pipe[i]<=rgb_pipe[i-1];sync_pipe[i]<=sync_pipe[i-1];
    color_pipe[i]<=color_pipe[i-1];
   end
   // Parallel rows prevent a combined row/field/byte mux on one pixel path.
   for(integer i=0;i<5;i=i+1)row_char1[i]<=text_char(i,text_col0);
   text_row1<=text_row0;
   case(text_row1)
    0:char2<=row_char1[0];1:char2<=row_char1[1];2:char2<=row_char1[2];
    3:char2<=row_char1[3];4:char2<=row_char1[4];default:char2<=32;
   endcase
   glyph3<=glyph(char2);
   case(row_pipe[3])
    0:glyph_row4<=glyph3[34:30];1:glyph_row4<=glyph3[29:25];
    2:glyph_row4<=glyph3[24:20];3:glyph_row4<=glyph3[19:15];
    4:glyph_row4<=glyph3[14:10];5:glyph_row4<=glyph3[9:5];
    6:glyph_row4<=glyph3[4:0];default:glyph_row4<=0;
   endcase
   // Transparent overlay: only glyph strokes replace the camera pixel.
   rgb_out<=!sync_pipe[4][2]?24'b0:(area_pipe[4] && ink)?color_pipe[4]:rgb_pipe[4];
   {de_out,hs_out,vs_out}<=sync_pipe[4];
  end
 end
endmodule
