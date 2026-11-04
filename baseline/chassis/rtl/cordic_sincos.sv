`timescale 1ns/1ps

module cordic_sincos (
    input  logic               clk,
    input  logic               rst,
    input  logic               start,
    input  logic signed [31:0] angle_q16,
    output logic signed [31:0] sin_q16,
    output logic signed [31:0] cos_q16,
    output logic               valid,
    output logic               busy
);
    logic signed [31:0] x, y, z;
    logic [4:0] iteration;

    function automatic signed [31:0] atan_value(input logic [4:0] n);
        case(n)
          0:atan_value=51472; 1:atan_value=30386; 2:atan_value=16055; 3:atan_value=8150;
          4:atan_value=4091; 5:atan_value=2047; 6:atan_value=1024; 7:atan_value=512;
          8:atan_value=256; 9:atan_value=128; 10:atan_value=64; 11:atan_value=32;
          12:atan_value=16; 13:atan_value=8; 14:atan_value=4; default:atan_value=2;
        endcase
    endfunction

    logic signed [31:0] x_next, y_next, z_next;
    always_comb begin
        if (z >= 0) begin
            x_next = x - (y >>> iteration);
            y_next = y + (x >>> iteration);
            z_next = z - atan_value(iteration);
        end else begin
            x_next = x + (y >>> iteration);
            y_next = y - (x >>> iteration);
            z_next = z + atan_value(iteration);
        end
    end

    always_ff @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            x<=0; y<=0; z<=0; iteration<=0; sin_q16<=0; cos_q16<=65536; busy<=1'b0;
        end else if (start && !busy) begin
            y <= 0;
            iteration <= 0;
            busy <= 1'b1;
            if (angle_q16 > 32'sd102944) begin x<=-32'sd39797; z<=angle_q16-32'sd205887; end
            else if (angle_q16 < -32'sd102944) begin x<=-32'sd39797; z<=angle_q16+32'sd205887; end
            else begin x<=32'sd39797; z<=angle_q16; end
        end else if (busy) begin
            x <= x_next; y <= y_next; z <= z_next;
            if (iteration == 15) begin
                cos_q16 <= x_next;
                sin_q16 <= y_next;
                valid   <= 1'b1;
                busy    <= 1'b0;
            end else iteration <= iteration + 1'b1;
        end
    end
endmodule
