`timescale 1ns/1ps

// Q16.16 implementation of the STM32 mecanum odometry path.
module odometry (
    input  logic               clk,
    input  logic               rst,
    input  logic               tick_20ms,
    input  logic signed [31:0] encoder_a,
    input  logic signed [31:0] encoder_b,
    input  logic signed [31:0] encoder_c,
    input  logic signed [31:0] encoder_d,
    input  logic signed [31:0] imu_roll_q16,
    input  logic signed [31:0] imu_pitch_q16,
    input  logic signed [31:0] imu_yaw_q16,
    input  logic signed [31:0] imu_gx_q16,
    input  logic signed [31:0] imu_gy_q16,
    input  logic signed [31:0] imu_gz_q16,
    input  logic               imu_updated,
    input  logic               zero_request,
    output logic signed [31:0] x_q16,
    output logic signed [31:0] y_q16,
    output logic signed [31:0] th_q16,
    output logic signed [31:0] vx_q16,
    output logic signed [31:0] vy_q16,
    output logic signed [31:0] wz_q16,
    output logic signed [31:0] roll_q16,
    output logic signed [31:0] pitch_q16,
    output logic signed [31:0] wx_q16,
    output logic signed [31:0] wy_q16,
    output logic               updated
);
    logic signed [31:0] last_a, last_b, last_c, last_d;
    logic signed [31:0] local_vx_hold, local_vy_hold;
    logic signed [31:0] da, db, dc, dd;
    logic signed [31:0] forward_sum, lateral_sum, rotate_sum;
    logic signed [63:0] vx_product, vy_product, wz_product;
    logic signed [31:0] local_vx_next, local_vy_next, local_wz_next;
    logic signed [31:0] angle_next, angle_hold;
    logic cordic_start, cordic_valid, cordic_busy;
    logic signed [31:0] sin_q16, cos_q16;
    logic signed [63:0] global_x_product, global_y_product;
    logic signed [31:0] global_vx, global_vy;
    logic [4:0] stage;
    logic signed [31:0] forward_ab,forward_cd,lateral_ab,lateral_cd,rotate_ab;
    logic signed [31:0] yaw_hold,gz_hold,roll_hold,pitch_hold,gx_hold,gy_hold,th_hold;
    logic imu_hold;
    logic signed [63:0] xc,ys,xs,yc;
    logic divide_angle_start,divide_position_start,angle_done,dx_done,dy_done;
    logic signed [31:0] angle_delta,dx,dy;

    signed_div50 angle_div(clk,rst,divide_angle_start,local_wz_next,angle_delta,angle_done);
    signed_div50 x_div(clk,rst,divide_position_start,global_vx,dx,dx_done);
    signed_div50 y_div(clk,rst,divide_position_start,global_vy,dy,dy_done);

    function automatic signed [31:0] normalize_angle(input logic signed [31:0] angle);
        begin
            if (angle > 32'sd205887) normalize_angle = angle - 32'sd411775;
            else if (angle < -32'sd205887) normalize_angle = angle + 32'sd411775;
            else normalize_angle = angle;
        end
    endfunction

    cordic_sincos cordic(
        .clk, .rst, .start(cordic_start), .angle_q16(angle_hold),
        .sin_q16, .cos_q16, .valid(cordic_valid), .busy(cordic_busy)
    );

    always_ff @(posedge clk) begin
        cordic_start <= 1'b0;
        divide_angle_start<=0;divide_position_start<=0;
        updated      <= 1'b0;
        if (rst) begin
            last_a<=encoder_a; last_b<=encoder_b; last_c<=encoder_c; last_d<=encoder_d;
            local_vx_hold<=0; local_vy_hold<=0; angle_hold<=0;
            x_q16<=0; y_q16<=0; th_q16<=0; vx_q16<=0; vy_q16<=0; wz_q16<=0;
            roll_q16<=0; pitch_q16<=0; wx_q16<=0; wy_q16<=0;
            stage<=0;da<=0;db<=0;dc<=0;dd<=0;
            forward_ab<=0;forward_cd<=0;lateral_ab<=0;lateral_cd<=0;rotate_ab<=0;
            forward_sum<=0;lateral_sum<=0;rotate_sum<=0;
            vx_product<=0;vy_product<=0;wz_product<=0;
            local_vx_next<=0;local_vy_next<=0;local_wz_next<=0;
            yaw_hold<=0;gz_hold<=0;roll_hold<=0;pitch_hold<=0;gx_hold<=0;gy_hold<=0;th_hold<=0;imu_hold<=0;
            angle_next<=0;xc<=0;ys<=0;xs<=0;yc<=0;
            global_x_product<=0;global_y_product<=0;global_vx<=0;global_vy<=0;
        end else begin
            if (zero_request) begin
                x_q16  <= 0;
                y_q16  <= 0;
                th_q16 <= 0;
            end
            case(stage)
            0: if (tick_20ms) begin
                // Sampling stays exactly at the 20ms tick; only arithmetic is delayed.
                da<=encoder_a-last_a;db<=encoder_b-last_b;dc<=encoder_c-last_c;dd<=encoder_d-last_d;
                last_a <= encoder_a;
                last_b <= encoder_b;
                last_c <= encoder_c;
                last_d <= encoder_d;
                imu_hold<=imu_updated;yaw_hold<=imu_yaw_q16;gz_hold<=imu_gz_q16;
                roll_hold<=imu_roll_q16;pitch_hold<=imu_pitch_q16;gx_hold<=imu_gx_q16;gy_hold<=imu_gy_q16;
                th_hold<=th_q16;stage<=1;
            end
            1: begin
                forward_ab<=da+db;forward_cd<=dc+dd;
                lateral_ab<=db-da;lateral_cd<=dc-dd;rotate_ab<=-da-db;stage<=2;
            end
            2: begin forward_sum<=forward_ab+forward_cd;lateral_sum<=lateral_ab+lateral_cd;rotate_sum<=rotate_ab+forward_cd;stage<=3;end
            3: begin vx_product<=forward_sum*32'sd40122;vy_product<=lateral_sum*32'sd40122;wz_product<=rotate_sum*32'sd211167;stage<=4;end
            4: begin local_vx_next<=vx_product>>>8;local_vy_next<=vy_product>>>8;local_wz_next<=wz_product>>>8;stage<=5;end
            5: begin divide_angle_start<=1;stage<=6;end
            6: if(angle_done)begin angle_next<=th_hold+angle_delta;stage<=7;end
            7: begin
                angle_hold<=imu_hold?yaw_hold:normalize_angle(angle_next);
                th_q16<=imu_hold?yaw_hold:normalize_angle(angle_next);
                local_vx_hold<=local_vx_next;local_vy_hold<=local_vy_next;
                vx_q16<=local_vx_next;vy_q16<=local_vy_next;wz_q16<=imu_hold?gz_hold:local_wz_next;
                if(imu_hold)begin roll_q16<=roll_hold;pitch_q16<=pitch_hold;wx_q16<=gx_hold;wy_q16<=gy_hold;end
                cordic_start<=1;stage<=8;
            end
            8: if(cordic_valid)begin
                xc<=$signed(local_vx_hold)*$signed(cos_q16);ys<=$signed(local_vy_hold)*$signed(sin_q16);
                xs<=$signed(local_vx_hold)*$signed(sin_q16);yc<=$signed(local_vy_hold)*$signed(cos_q16);stage<=9;
            end
            9: begin global_x_product<=xc-ys;global_y_product<=xs+yc;stage<=10;end
            10: begin global_vx<=global_x_product>>>16;global_vy<=global_y_product>>>16;stage<=11;end
            11: begin divide_position_start<=1;stage<=12;end
            12: if(dx_done && dy_done)begin x_q16<=x_q16+dx;y_q16<=y_q16+dy;updated<=1;stage<=0;end
            default:stage<=0;
            endcase
        end
    end
endmodule

// Exact signed /50, truncating toward zero including INT32_MIN.
module signed_div50(input logic clk,rst,start,input logic signed [31:0] dividend,
                    output logic signed [31:0] quotient,output logic valid);
    logic [5:0] step,rem_bits;
    logic [31:0] work;
    logic sign_bit;
    wire [6:0] trial={rem_bits,work[31]};
    wire take_bit=trial>=7'd50;
    always_ff @(posedge clk)begin
        valid<=0;
        if(rst)begin step<=0;rem_bits<=0;work<=0;sign_bit<=0;quotient<=0;end
        else if(step==0)begin
            if(start)begin work<=dividend[31]?-dividend:dividend;sign_bit<=dividend[31];rem_bits<=0;step<=1;end
        end else if(step<=32)begin
            rem_bits<=take_bit?trial-7'd50:trial;
            work<={work[30:0],take_bit};step<=step+1'b1;
        end else begin quotient<=sign_bit?-$signed(work):$signed(work);valid<=1;step<=0;end
    end
endmodule
