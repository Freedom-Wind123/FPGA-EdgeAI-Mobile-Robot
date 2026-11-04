// Read-only display converter: radian ASCII to degree ASCII, one decimal.
// Shift/add multiplication, restoring division and double-dabble BCD avoid
// combinational constant division/multiplication on the 50 MHz clock.
module osd_angles_deg(input wire clk,reset_n,publish,input wire [810:0] source_data,
 output reg [810:0] converted,output reg done,output wire busy);
 reg [4:0] state;
 reg [1:0] item;
 reg [3:0] pos,outpos,frac_digits;
 reg negative,dot_seen,started;
 reg [31:0] magnitude,decimal_value,bcd;
 reg [31:0] bcd_adjusted;
 reg [79:0] field_text;
 reg [51:0] dividend,quotient,multiplicand,product;
 reg [19:0] multiplier;
 reg [20:0] remainder;
 reg [5:0] bits_left;
 reg [2:0] digit_index;
 wire [7:0] ch=field_text[pos*8+:8];
 wire [3:0] digit=bcd[digit_index*4+:4];
 wire [20:0] rem_next={remainder[19:0],dividend[51]};
 assign busy=state!=0;
 always @* begin
  bcd_adjusted=bcd;
  for(integer j=0;j<8;j=j+1)
   if(bcd[j*4+:4]>=5)bcd_adjusted[j*4+:4]=bcd[j*4+:4]+4'd3;
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   state<=0;converted<=0;done<=0;item<=0;pos<=0;outpos<=0;field_text<=0;
   negative<=0;dot_seen<=0;started<=0;frac_digits<=0;magnitude<=0;
   decimal_value<=0;bcd<=0;dividend<=0;quotient<=0;remainder<=0;
   bits_left<=0;digit_index<=0;multiplicand<=0;multiplier<=0;product<=0;
  end else begin
   done<=0;
   case(state)
    0:if(publish)begin
     converted<=source_data;item<=0;
     state<=source_data[810:809]==0?14:1;
    end
    1:begin
     // Fixed slices keep the read/write mux local to one 80-bit field.
     case(item)
      0:field_text<=converted[160+:80];
      1:field_text<=converted[400+:80];
      2:field_text<=converted[480+:80];
      3:field_text<=converted[560+:80];
     endcase
     magnitude<=0;negative<=0;dot_seen<=0;frac_digits<=0;pos<=0;state<=2;
    end
    2:begin
     if(ch=="-")negative<=1;
     else if(ch==".")dot_seen<=1;
     else if(ch>="0" && ch<="9" && (!dot_seen || frac_digits<3))begin
      magnitude<=magnitude*10+(ch-8'h30);
      if(dot_seen)frac_digits<=frac_digits+1'b1;
     end
     if(pos==9)state<=3;else pos<=pos+1'b1;
    end
    3:begin
     if(frac_digits<3)begin magnitude<=magnitude*10;frac_digits<=frac_digits+1'b1;end
     else begin
      multiplicand<={20'b0,magnitude};multiplier<=20'd572958;
      product<=52'd500000;bits_left<=20;state<=4;
     end
    end
    4:begin
     if(multiplier[0])product<=product+multiplicand;
     multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
     bits_left<=bits_left-1'b1;
     if(bits_left==1)state<=5;
    end
    5:begin dividend<=product;quotient<=0;remainder<=0;bits_left<=52;state<=6;end
    6:begin
     dividend<={dividend[50:0],1'b0};bits_left<=bits_left-1'b1;
     if(rem_next>=1000000)begin remainder<=rem_next-1000000;quotient<={quotient[50:0],1'b1};end
     else begin remainder<=rem_next;quotient<={quotient[50:0],1'b0};end
     if(bits_left==1)state<=7;
    end
    7:begin
     field_text<={10{8'h20}};
     if(quotient>99999999)begin
      field_text[0+:24]<={"R","V","O"};state<=12;
     end else begin decimal_value<=quotient[31:0];bcd<=0;bits_left<=32;state<=8;end
    end
    8:begin
     bcd<={bcd_adjusted[30:0],decimal_value[31]};
     decimal_value<=decimal_value<<1;bits_left<=bits_left-1'b1;
     if(bits_left==1)begin digit_index<=7;outpos<=0;started<=0;state<=9;end
    end
    9:begin
     if(negative && quotient!=0)begin field_text[0+:8]<="-";outpos<=1;end
     state<=10;
    end
    10:begin
     if(started || digit!=0 || digit_index<=1)begin
      field_text[outpos*8+:8]<=8'h30+digit;
      outpos<=outpos+1'b1;started<=1;
      if(digit_index==1)begin field_text[(outpos+1)*8+:8]<=".";outpos<=outpos+2;end
     end
     if(digit_index==0)state<=12;else digit_index<=digit_index-1'b1;
    end
    12:begin
     case(item)
      0:converted[160+:80]<=field_text;
      1:converted[400+:80]<=field_text;
      2:converted[480+:80]<=field_text;
      3:converted[560+:80]<=field_text;
     endcase
     if(item==3)state<=14;else begin item<=item+1'b1;state<=1;end
    end
    14:begin done<=1;state<=0;end
    default:state<=0;
   endcase
  end
 end
endmodule
