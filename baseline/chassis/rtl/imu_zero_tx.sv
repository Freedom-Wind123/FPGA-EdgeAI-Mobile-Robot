`timescale 1ns/1ps

module imu_zero_tx (
    input  logic       clk,
    input  logic       rst,
    input  logic       tick_1ms,
    input  logic       request,
    input  logic       uart_busy,
    output logic [7:0] uart_data,
    output logic       uart_start,
    output logic       busy
);
    typedef enum logic [3:0] {IDLE, SEND_UNLOCK, UNLOCK_ACCEPT, UNLOCK_DONE,
                             WAIT_UNLOCK, SEND_ZERO, ZERO_ACCEPT, ZERO_DONE, WAIT_ZERO} state_t;
    state_t state;
    logic [2:0] index;
    logic [5:0] wait_ms;

    function automatic [7:0] unlock_byte(input logic [2:0] n);
        case(n) 0:unlock_byte=8'hff; 1:unlock_byte=8'haa; 2:unlock_byte=8'h69; 3:unlock_byte=8'h88; default:unlock_byte=8'hb5; endcase
    endfunction
    function automatic [7:0] zero_byte(input logic [2:0] n);
        case(n) 0:zero_byte=8'hff; 1:zero_byte=8'haa; 2:zero_byte=8'h01; 3:zero_byte=8'h04; default:zero_byte=8'h00; endcase
    endfunction

    always_ff @(posedge clk) begin
        uart_start <= 1'b0;
        if (rst) begin
            state     <= IDLE;
            index     <= 0;
            wait_ms   <= 0;
            uart_data <= 0;
            busy      <= 1'b0;
        end else begin
            case (state)
                IDLE: if (request) begin
                    busy  <= 1'b1;
                    index <= 0;
                    state <= SEND_UNLOCK;
                end
                SEND_UNLOCK: if (!uart_busy) begin
                    uart_data  <= unlock_byte(index);
                    uart_start <= 1'b1;
                    state <= UNLOCK_ACCEPT;
                end
                // uart_start is registered: wait for the transmitter to accept
                // it before advancing. Otherwise alternate bytes are skipped.
                UNLOCK_ACCEPT: if (uart_busy) state <= UNLOCK_DONE;
                UNLOCK_DONE: if (!uart_busy) begin
                    if (index == 4) begin index<=0; wait_ms<=0; state<=WAIT_UNLOCK; end
                    else begin index <= index + 1'b1; state <= SEND_UNLOCK; end
                end
                WAIT_UNLOCK: if (tick_1ms) begin
                    if (wait_ms == 49) begin wait_ms<=0; state<=SEND_ZERO; end
                    else wait_ms <= wait_ms + 1'b1;
                end
                SEND_ZERO: if (!uart_busy) begin
                    uart_data  <= zero_byte(index);
                    uart_start <= 1'b1;
                    state <= ZERO_ACCEPT;
                end
                ZERO_ACCEPT: if (uart_busy) state <= ZERO_DONE;
                ZERO_DONE: if (!uart_busy) begin
                    if (index == 4) begin index<=0; wait_ms<=0; state<=WAIT_ZERO; end
                    else begin index <= index + 1'b1; state <= SEND_ZERO; end
                end
                WAIT_ZERO: if (tick_1ms) begin
                    if (wait_ms == 49) begin wait_ms<=0; busy<=1'b0; state<=IDLE; end
                    else wait_ms <= wait_ms + 1'b1;
                end
            endcase
        end
    end
endmodule
