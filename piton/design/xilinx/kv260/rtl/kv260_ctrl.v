// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// kv260_ctrl - the KV260 run-control registers, an AXI4-Lite slave on the
// PS's M_AXI_HPM0 (0xA000_0000 in the PS map). The PS loads a program into
// the DDR carve-out while the system is held in reset, checks the ID, and
// sets RUN.
//
//   0x0  MANYCORE_CONTROL  rw  bit 0 RUN: 1 releases the system from reset;
//                              0 (the reset value) holds it
//   0x4  MANYCORE_ID       ro  0x4F504E31 ("OPN1"); a loader must read this
//                              before it touches memory
//
// Other offsets read 0 and ignore writes. Every response is OKAY. One read
// and one write are handled at a time; a write waits for both AW and W.

`timescale 1ps/1ps

module kv260_ctrl (
    input             clk,
    input             rst_n,

    input      [5:0]  s_axi_awaddr,
    input      [2:0]  s_axi_awprot,
    input             s_axi_awvalid,
    output            s_axi_awready,
    input      [31:0] s_axi_wdata,
    input      [3:0]  s_axi_wstrb,
    input             s_axi_wvalid,
    output            s_axi_wready,
    output     [1:0]  s_axi_bresp,
    output reg        s_axi_bvalid,
    input             s_axi_bready,

    input      [5:0]  s_axi_araddr,
    input      [2:0]  s_axi_arprot,
    input             s_axi_arvalid,
    output            s_axi_arready,
    output reg [31:0] s_axi_rdata,
    output     [1:0]  s_axi_rresp,
    output reg        s_axi_rvalid,
    input             s_axi_rready,

    output            run
);
    localparam [31:0] MAGIC = 32'h4F50_4E31;

    reg [31:0] control;

    // write: accept AW and W in the same cycle, when no response is pending
    wire wr = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid;
    assign s_axi_awready = wr;
    assign s_axi_wready  = wr;
    assign s_axi_bresp   = 2'b00;

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            control      <= 32'd0;
            s_axi_bvalid <= 1'b0;
        end else begin
            if (wr) begin
                if (s_axi_awaddr[5:2] == 4'd0)
                    for (i = 0; i < 4; i = i + 1)
                        if (s_axi_wstrb[i])
                            control[8*i +: 8] <= s_axi_wdata[8*i +: 8];
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // read: accept AR when no read data is pending
    wire rd = s_axi_arvalid & ~s_axi_rvalid;
    assign s_axi_arready = rd;
    assign s_axi_rresp   = 2'b00;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
        end else begin
            if (rd) begin
                case (s_axi_araddr[5:2])
                    4'd0:    s_axi_rdata <= control;
                    4'd1:    s_axi_rdata <= MAGIC;
                    default: s_axi_rdata <= 32'd0;
                endcase
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    assign run = control[0];
endmodule
