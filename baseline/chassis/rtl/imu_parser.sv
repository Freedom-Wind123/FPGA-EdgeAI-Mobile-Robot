`timescale 1ns/1ps

module imu_parser (
    input  logic               clk,
    input  logic               rst,
    input  logic               tick_1ms,
    input  logic [7:0]         rx_data,
    input  logic               rx_valid,
    input  logic               zero_request,
    output logic signed [31:0] roll_q16,
    output logic signed [31:0] pitch_q16,
    output logic signed [31:0] yaw_q16,
    output logic signed [31:0] gyro_x_q16,
    output logic signed [31:0] gyro_y_q16,
    output logic signed [31:0] gyro_z_q16,
    output logic               attitude_updated,
    output logic [31:0]        checksum_errors
);
    logic [7:0] frame [0:9];
    logic [3:0] index;
    logic [7:0] checksum;
    logic active;
    logic [5:0] idle_ms;
    logic [15:0] angle_frames;
    logic yaw_calibrated;
    logic signed [31:0] yaw_offset_q16;
    integer i;

    function automatic signed [15:0] word_s16(input logic [7:0] lo, input logic [7:0] hi);
        word_s16 = $signed({hi, lo});
    endfunction
    function automatic signed [31:0] angle_convert(input logic signed [15:0] raw);
        logic signed [47:0] product;
        begin
            product = raw * 32'sd411775;
            angle_convert = product >>> 16;
        end
    endfunction
    function automatic signed [31:0] gyro_convert(input logic signed [15:0] raw);
        logic signed [63:0] product;
        begin
            product = raw * 32'sd4575276;
            gyro_convert = product >>> 16;
        end
    endfunction
    function automatic signed [31:0] normalize_angle(input logic signed [31:0] angle);
        begin
            if (angle > 32'sd205887)
                normalize_angle = angle - 32'sd411775;
            else if (angle < -32'sd205887)
                normalize_angle = angle + 32'sd411775;
            else
                normalize_angle = angle;
        end
    endfunction

    logic signed [15:0] raw0, raw1, raw2;
    logic signed [31:0] converted_yaw;
    always_comb begin
        raw0 = word_s16(frame[2], frame[3]);
        raw1 = word_s16(frame[4], frame[5]);
        raw2 = word_s16(frame[6], frame[7]);
        converted_yaw = angle_convert(raw2);
    end

    always_ff @(posedge clk) begin
        attitude_updated <= 1'b0;
        if (rst) begin
            index              <= 0;
            checksum           <= 0;
            active             <= 1'b0;
            idle_ms            <= 0;
            roll_q16           <= 0;
            pitch_q16          <= 0;
            yaw_q16            <= 0;
            gyro_x_q16         <= 0;
            gyro_y_q16         <= 0;
            gyro_z_q16         <= 0;
            angle_frames       <= 0;
            yaw_calibrated     <= 1'b0;
            yaw_offset_q16     <= 0;
            checksum_errors    <= 0;
            for (i=0; i<10; i=i+1) frame[i] <= 0;
        end else begin
            if (zero_request) begin
                yaw_calibrated <= 1'b0;
                yaw_offset_q16 <= 0;
                yaw_q16        <= 0;
            end
            if (tick_1ms && active) begin
                if (idle_ms >= 6'd49) begin
                    active  <= 1'b0;
                    index   <= 0;
                    idle_ms <= 0;
                end else begin
                    idle_ms <= idle_ms + 1'b1;
                end
            end
            if (rx_valid) begin
                idle_ms <= 0;
                if (!active) begin
                    if (rx_data == 8'h55) begin
                        frame[0] <= 8'h55;
                        checksum <= 8'h55;
                        index    <= 1;
                        active   <= 1'b1;
                    end
                end else if (index == 1) begin
                    if ((rx_data >= 8'h50) && (rx_data <= 8'h5f)) begin
                        frame[1] <= rx_data;
                        checksum <= checksum + rx_data;
                        index <= 2;
                    end else begin
                        active <= 1'b0;
                        index  <= 0;
                    end
                end else if (index < 10) begin
                    frame[index] <= rx_data;
                    checksum <= checksum + rx_data;
                    index <= index + 1'b1;
                end else begin
                    active <= 1'b0;
                    index  <= 0;
                    if (checksum != rx_data) begin
                        checksum_errors <= checksum_errors + 1'b1;
                    end else if (frame[1] == 8'h52) begin
                        gyro_x_q16 <= gyro_convert(raw0);
                        gyro_y_q16 <= gyro_convert(raw1);
                        gyro_z_q16 <= gyro_convert(raw2);
                    end else if (frame[1] == 8'h53) begin
                        roll_q16  <= angle_convert(raw0);
                        pitch_q16 <= angle_convert(raw1);
                        if (!yaw_calibrated && (angle_frames >= 10)) begin
                            yaw_offset_q16 <= converted_yaw;
                            yaw_calibrated <= 1'b1;
                            yaw_q16        <= 0;
                        end else begin
                            yaw_q16 <= normalize_angle(converted_yaw - yaw_offset_q16);
                        end
                        attitude_updated <= 1'b1;
                        angle_frames <= angle_frames + 1'b1;
                    end
                end
            end
        end
    end
endmodule
