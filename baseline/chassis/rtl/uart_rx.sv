`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       rx,
    output logic [7:0] data,
    output logic       valid
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;
    localparam integer CW           = $clog2(CLKS_PER_BIT+1);

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;
    state_t state;
    logic [CW-1:0] count;
    logic [2:0] rx_sync;
    logic [2:0] bit_index;
    logic [7:0] shift;

    always_ff @(posedge clk) begin
        rx_sync <= {rx_sync[1:0], rx};
        valid   <= 1'b0;
        if (rst) begin
            state     <= IDLE;
            count     <= '0;
            bit_index <= '0;
            shift     <= '0;
            data      <= '0;
            rx_sync   <= 3'b111;
        end else begin
            case (state)
                IDLE: if (!rx_sync[2]) begin
                    count <= HALF_BIT-1;
                    state <= START;
                end
                START: if (count == 0) begin
                    if (!rx_sync[2]) begin
                        count     <= CLKS_PER_BIT-1;
                        bit_index <= 0;
                        state     <= DATA;
                    end else begin
                        state <= IDLE;
                    end
                end else count <= count - 1'b1;
                DATA: if (count == 0) begin
                    shift[bit_index] <= rx_sync[2];
                    count <= CLKS_PER_BIT-1;
                    if (bit_index == 3'd7)
                        state <= STOP;
                    else
                        bit_index <= bit_index + 1'b1;
                end else count <= count - 1'b1;
                STOP: if (count == 0) begin
                    if (rx_sync[2]) begin
                        data  <= shift;
                        valid <= 1'b1;
                    end
                    state <= IDLE;
                end else count <= count - 1'b1;
            endcase
        end
    end
endmodule
