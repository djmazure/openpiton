// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// Self-checking testbench for kv260_ctrl: reset values, the ID, RUN set and
// cleared, byte strobes, writes to other offsets ignored, and AW and W
// arriving in different cycles. Run:
//   $ verilator --binary --timing -Wall -Wno-UNUSEDSIGNAL kv260_ctrl_tb.v ../rtl/kv260_ctrl.v --top-module kv260_ctrl_tb
//   $ obj_dir/Vkv260_ctrl_tb
// Prints "kv260_ctrl_tb PASS" and exits 0, or "FAIL ..." and exits 1.

`timescale 1ns/1ps

module kv260_ctrl_tb;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [5:0]  awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 0;
    wire        awready, wready, bvalid, arready, rvalid, run;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;
    integer errors = 0;

    kv260_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .run(run));

    task check(input [255:0] what, input [31:0] got, input [31:0] exp);
        if (got !== exp) begin
            $display("FAIL %0s: got %h, expected %h", what, got, exp);
            errors = errors + 1;
        end
    endtask

    // write; gap > 0 presents W that many cycles after AW
    task axi_write(input [5:0] a, input [31:0] d, input [3:0] s, input integer gap);
        integer n;
        begin
            @(negedge clk); awaddr = a; awvalid = 1;
            for (n = 0; n < gap; n = n + 1) begin
                @(posedge clk); #1;
                if (awready) begin $display("FAIL AW accepted without W"); errors = errors + 1; end
                @(negedge clk);
            end
            wdata = d; wstrb = s; wvalid = 1;
            @(posedge clk); while (!(awready && wready)) @(posedge clk);
            @(negedge clk); awvalid = 0; wvalid = 0; bready = 1;
            while (!bvalid) @(negedge clk);
            check("bresp", {30'd0, bresp}, 0);
            @(negedge clk); bready = 0;
        end
    endtask

    task axi_read(input [5:0] a, output [31:0] d);
        begin
            @(negedge clk); araddr = a; arvalid = 1;
            @(posedge clk); while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0; rready = 1;
            while (!rvalid) @(negedge clk);
            d = rdata;
            check("rresp", {30'd0, rresp}, 0);
            @(negedge clk); rready = 0;
        end
    endtask

    reg [31:0] v;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        check("run after reset", {31'd0, run}, 0);
        axi_read(6'h4, v);  check("ID", v, 32'h4F504E31);
        axi_read(6'h0, v);  check("CONTROL after reset", v, 0);
        axi_read(6'h8, v);  check("unmapped 0x8", v, 0);
        axi_write(6'h0, 32'h1, 4'hF, 0);
        check("run after RUN=1", {31'd0, run}, 1);
        axi_read(6'h0, v);  check("CONTROL after RUN=1", v, 1);
        axi_write(6'h4, 32'h0, 4'hF, 0);
        axi_read(6'h4, v);  check("ID is read-only", v, 32'h4F504E31);
        axi_write(6'h8, 32'h0, 4'hF, 0);
        check("write to 0x8 leaves RUN", {31'd0, run}, 1);
        axi_write(6'h0, 32'h0, 4'h2, 0);
        check("strobe 0x2 leaves byte 0", {31'd0, run}, 1);
        axi_write(6'h0, 32'h0, 4'h1, 3);
        check("RUN=0 with W 3 cycles after AW", {31'd0, run}, 0);
        rst_n = 0; @(posedge clk); @(posedge clk); rst_n = 1;
        axi_write(6'h0, 32'h1, 4'hF, 0);
        rst_n = 0; @(posedge clk); #1;
        check("reset clears RUN", {31'd0, run}, 0);
        if (errors == 0) begin $display("kv260_ctrl_tb PASS"); $finish; end
        else begin $display("kv260_ctrl_tb FAIL (%0d)", errors); $fatal(1); end
    end
    initial begin #100000; $display("FAIL timeout"); $fatal(1); end
endmodule
