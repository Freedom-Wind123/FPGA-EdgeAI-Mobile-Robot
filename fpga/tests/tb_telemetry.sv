`timescale 1ns/1ps
module tb_telemetry;
    reg clk=0, rst=1, start=0;
    always #5 clk=~clk;
    wire [7:0] data, received;
    wire valid, busy, tx_busy, tx, rx_valid;
    string expected = {"$0.000,-1.500,12.250,0.000,0.000,0.000,0.000,0.000,0.000,0.000",8'h0d,8'h0a};
    integer n=0;
    telemetry_formatter dut(.clk(clk),.rst(rst),.start(start),
        .value0(32'sd0),.value1(-32'sd98304),.value2(32'sd802816),
        .value3(32'sd0),.value4(32'sd0),.value5(32'sd0),.value6(32'sd0),
        .value7(32'sd0),.value8(32'sd0),.value9(32'sd0),
        .out_ready(!tx_busy),.out_data(data),.out_valid(valid),.busy(busy));
    uart_tx #(.CLK_HZ(1000000),.BAUD(100000)) sender(
        .clk(clk),.rst(rst),.data(data),.start(valid && !tx_busy),.tx(tx),.busy(tx_busy));
    uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) receiver(
        .clk(clk),.rst(rst),.rx(tx),.data(received),.valid(rx_valid));
    always @(posedge clk) if(rx_valid) begin
        if (n >= expected.len() || received !== expected[n])
            $fatal(1,"telemetry byte %0d got %02x",n,received);
        n=n+1;
    end
    initial begin
        repeat(4) @(negedge clk); rst=0;
        @(negedge clk); start=1;
        @(negedge clk); start=0;
        wait(n==expected.len()); repeat(200) @(negedge clk);
        if(busy || tx_busy) $fatal(1,"telemetry failed to finish");
        $display("PASS tb_telemetry: complete UART frame including CRLF"); $finish;
    end
    initial begin #2000000; $fatal(1,"telemetry timeout"); end
endmodule
