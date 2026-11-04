`timescale 1ns/1ps

// Parses the existing STM32 wire format: $vx,vy,wz\r\n.
// Values are emitted as signed Q16.16 numbers.
module ros_command_parser (
    input  logic               clk,
    input  logic               rst,
    input  logic [7:0]         rx_data,
    input  logic               rx_valid,
    output logic signed [31:0] vx_q16,
    output logic signed [31:0] vy_q16,
    output logic signed [31:0] wz_q16,
    output logic               command_valid
);
    logic active;
    logic negative;
    logic seen_dot;
    logic [1:0] field;
    logic [3:0] frac_digits;
    logic [31:0] integer_part;
    logic [31:0] fraction_part;

    // Restoring division emits the exact same truncated 16 fractional bits.
    // A UART byte takes >4300 clocks at 50MHz/115200; conversion takes 18.
    // Snapshot on delimiter; never divide by a variable in one clock.
    logic [4:0] convert_step;
    logic [19:0] remainder, divisor;
    logic [15:0] quotient, integer_hold;
    logic sign_hold, final_hold;
    logic [1:0] field_hold;
    wire [20:0] shifted_remainder = {remainder,1'b0};
    wire subtract_bit = shifted_remainder >= {1'b0,divisor};
    wire [31:0] magnitude = {integer_hold,quotient};
    function automatic [19:0] decimal_scale(input logic [3:0] digits);
        case(digits)
            0: decimal_scale=1; 1: decimal_scale=10;
            2: decimal_scale=100; 3: decimal_scale=1000;
            4: decimal_scale=10000; 5: decimal_scale=100000;
            default: decimal_scale=1000000;
        endcase
    endfunction

    task automatic clear_number;
        begin
            negative      <= 1'b0;
            seen_dot      <= 1'b0;
            frac_digits   <= '0;
            integer_part  <= '0;
            fraction_part <= '0;
        end
    endtask

    always_ff @(posedge clk) begin
        command_valid <= 1'b0;
        if (rst) begin
            active        <= 1'b0;
            field         <= '0;
            vx_q16        <= '0;
            vy_q16        <= '0;
            wz_q16        <= '0;
            negative      <= 1'b0;
            seen_dot      <= 1'b0;
            frac_digits   <= '0;
            integer_part  <= '0;
            fraction_part <= '0;
            convert_step <= 0; remainder<=0;divisor<=1;quotient<=0;
            integer_hold<=0;sign_hold<=0;final_hold<=0;field_hold<=0;
        end else begin
            if(convert_step != 0) begin
                if(convert_step <= 16) begin
                    remainder <= subtract_bit ? shifted_remainder-divisor : shifted_remainder;
                    quotient <= {quotient[14:0],subtract_bit};
                    convert_step <= convert_step+1'b1;
                end else begin
                    case(field_hold)
                        0: vx_q16 <= sign_hold ? -$signed(magnitude) : $signed(magnitude);
                        1: vy_q16 <= sign_hold ? -$signed(magnitude) : $signed(magnitude);
                        2: wz_q16 <= sign_hold ? -$signed(magnitude) : $signed(magnitude);
                    endcase
                    command_valid <= final_hold;
                    convert_step <= 0;
                end
            end
            if (rx_valid) begin
            if (rx_data == "$") begin
                active <= 1'b1;
                field  <= 0;
                clear_number();
            end else if (active) begin
                if ((rx_data == "-") && (integer_part == 0) && !seen_dot)
                    negative <= 1'b1;
                else if ((rx_data == "+") && (integer_part == 0) && !seen_dot)
                    negative <= 1'b0;
                else if (rx_data == ".")
                    seen_dot <= 1'b1;
                else if ((rx_data >= "0") && (rx_data <= "9")) begin
                    if (!seen_dot) begin
                        if (integer_part < 32'd100000)
                            integer_part <= integer_part * 10 + (rx_data - "0");
                    end else if (frac_digits < 6) begin
                        fraction_part <= fraction_part * 10 + (rx_data - "0");
                        frac_digits   <= frac_digits + 1'b1;
                    end
                end else if ((rx_data == ",") || (rx_data == 8'h0a)) begin
                    integer_hold<=integer_part[15:0];
                    remainder<=fraction_part[19:0]; divisor<=decimal_scale(frac_digits);
                    quotient<=0;sign_hold<=negative;field_hold<=field;
                    final_hold<=(rx_data==8'h0a && field==2);
                    convert_step<=1;
                    if (rx_data == 8'h0a) begin
                        active <= 1'b0;
                    end else if (field < 2) begin
                        field <= field + 1'b1;
                        clear_number();
                    end else begin
                        active <= 1'b0;
                    end
                end else if (rx_data != 8'h0d) begin
                    active <= 1'b0;
                end
            end
        end
        end
    end
endmodule
