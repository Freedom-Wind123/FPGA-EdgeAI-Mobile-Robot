`timescale 1ns/1ps

// Emits the original normal-mode telemetry contract:
// $x,y,th,vx,vy,wz,roll,pitch,wx,wy\r\n
module telemetry_formatter_ref (
    input  logic               clk,
    input  logic               rst,
    input  logic               start,
    input  logic signed [31:0] value0,
    input  logic signed [31:0] value1,
    input  logic signed [31:0] value2,
    input  logic signed [31:0] value3,
    input  logic signed [31:0] value4,
    input  logic signed [31:0] value5,
    input  logic signed [31:0] value6,
    input  logic signed [31:0] value7,
    input  logic signed [31:0] value8,
    input  logic signed [31:0] value9,
    input  logic               out_ready,
    output logic [7:0]         out_data,
    output logic               out_valid,
    output logic               busy
);
    typedef enum logic [4:0] {IDLE, PREFIX, SIGN, D5, D4, D3, D2, D1, DOT,
                              F3, F2, F1, SEP, CR, LF} state_t;
    state_t state;
    logic [3:0] field;
    logic signed [31:0] values [0:9];
    logic sign;
    logic [31:0] magnitude;
    logic [15:0] integer_part;
    logic [10:0] fraction;
    logic suppress;
    integer i;

    task automatic load_number(input logic [3:0] n);
        logic [47:0] frac_product;
        logic [31:0] mag_temp;
        logic [15:0] int_temp;
        logic [10:0] frac_temp;
        begin
            sign = values[n][31];
            mag_temp = values[n][31] ? -values[n] : values[n];
            int_temp = mag_temp[31:16];
            frac_product = mag_temp[15:0] * 1000 + 32768;
            frac_temp = frac_product >> 16;
            if (frac_temp >= 1000) begin
                frac_temp = 0;
                int_temp = int_temp + 1'b1;
            end
            magnitude    = mag_temp;
            integer_part = int_temp;
            fraction     = frac_temp;
            suppress     = 1'b1;
        end
    endtask

    function automatic [7:0] digit(input logic [15:0] number, input integer divisor);
        digit = "0" + ((number / divisor) % 10);
    endfunction

    always_comb begin
        out_valid = busy && (state != IDLE) && (state != SIGN || sign)
                    && (state != D5 || integer_part >= 10000)
                    && (state != D4 || integer_part >= 1000)
                    && (state != D3 || integer_part >= 100)
                    && (state != D2 || integer_part >= 10);
        case (state)
            PREFIX: out_data = "$";
            SIGN:   out_data = "-";
            D5:     out_data = digit(integer_part,10000);
            D4:     out_data = digit(integer_part,1000);
            D3:     out_data = digit(integer_part,100);
            D2:     out_data = digit(integer_part,10);
            D1:     out_data = digit(integer_part,1);
            DOT:    out_data = ".";
            F3:     out_data = "0" + ((fraction/100)%10);
            F2:     out_data = "0" + ((fraction/10)%10);
            F1:     out_data = "0" + (fraction%10);
            SEP:    out_data = ",";
            CR:     out_data = 8'h0d;
            LF:     out_data = 8'h0a;
            default:out_data = 0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state<=IDLE; field<=0; busy<=1'b0; sign<=0; magnitude<=0;
            integer_part<=0; fraction<=0; suppress<=1'b1;
            for (i=0;i<10;i=i+1) values[i]<=0;
        end else if (state == IDLE) begin
            busy <= 1'b0;
            if (start) begin
                values[0]<=value0; values[1]<=value1; values[2]<=value2; values[3]<=value3; values[4]<=value4;
                values[5]<=value5; values[6]<=value6; values[7]<=value7; values[8]<=value8; values[9]<=value9;
                field<=0; busy<=1'b1; state<=PREFIX;
            end
        end else if ((state == SIGN) && !sign) begin
            state <= D5;
        end else if ((state == D5) && (integer_part < 10000)) begin
            state <= D4;
        end else if ((state == D4) && (integer_part < 1000)) begin
            state <= D3;
        end else if ((state == D3) && (integer_part < 100)) begin
            state <= D2;
        end else if ((state == D2) && (integer_part < 10)) begin
            state <= D1;
        end else if (out_ready && out_valid) begin
            case(state)
                PREFIX: begin load_number(0); state<=SIGN; end
                SIGN: state<=D5;
                D5: begin suppress<=0; state<=D4; end
                D4: begin suppress<=0; state<=D3; end
                D3: begin suppress<=0; state<=D2; end
                D2: begin suppress<=0; state<=D1; end
                D1: state<=DOT;
                DOT: state<=F3;
                F3: state<=F2;
                F2: state<=F1;
                F1: begin
                    if (field == 9) state <= CR;
                    else state <= SEP;
                end
                SEP: begin field<=field+1'b1; load_number(field+1'b1); state<=SIGN; end
                CR: state<=LF;
                LF: begin state<=IDLE; busy<=1'b0; end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
