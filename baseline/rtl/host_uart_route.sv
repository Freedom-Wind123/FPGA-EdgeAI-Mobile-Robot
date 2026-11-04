`timescale 1ns/1ps
// Sample SW1 only during startup/S0 reset. Never switch a live UART mid-frame.
module host_uart_route (
    input wire clk, key_stop_n, select_gpio,
    input wire usb_rx, gpio_rx, core_tx,
    output wire core_rx, usb_tx, gpio_tx, chassis_stop_n,
    output wire gpio_selected
);
    reg selected = 0;
    assign gpio_selected = selected;
    reg [4:0] powerup = 0;
    (* syn_preserve=1 *) reg [1:0] select_sync = 0;
    always @(posedge clk) begin
        if (!(&powerup)) powerup <= powerup + 1'b1;
        select_sync <= {select_sync[0],select_gpio};
        if (!chassis_stop_n) selected <= select_sync[1];
    end
    assign chassis_stop_n = key_stop_n && (&powerup);
    assign core_rx = gpio_selected ? gpio_rx : usb_rx;
    assign usb_tx = gpio_selected ? 1'b1 : core_tx;
    assign gpio_tx = gpio_selected ? core_tx : 1'b1;
endmodule
