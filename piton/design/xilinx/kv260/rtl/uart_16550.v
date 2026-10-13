// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// uart_16550 — open RTL in OpenPiton's Xilinx-IP slot (OPN-P2.16).
//
// OpenPiton's chipset UART (io_ctrl/rtl/uart_top.v, PITON_UART16550) instantiates
// a module named uart_16550 with the port list of Xilinx's AXI UART16550 IP.
// That IP is encrypted: Verilator cannot run it, so the board image would not
// be the design the simulation checked. This module fills the same slot with
// the OpenCores 16550 register file Microwatt ships (ext/microwatt/uart16550,
// uart_regs + FIFOs; its Wishbone uart_top is not used, which also avoids the
// name clash with OpenPiton's uart_top).
//
// Register map, as uart_top presents it: 16550 register N at byte address
// 0x1000 + 4*N (uart_top remaps the core's byte offset N there), the byte in
// wdata[7:0] / rdata[7:0]. So the core sees a plain 8-bit 16550 at reg-shift 0,
// which is what the Microwatt DTS (powerlib) and Linux's 8250 driver expect.
//
// One AXI-lite transaction at a time (uart_top's noc_axilite_bridge issues one).
// A read strobes wb_re_i for one cycle, which is when the 16550's read side
// effects happen (RBR pop, LSR/IIR clear), and captures the byte that cycle.
//
// TODO(rr UART IP, owner request 2026-09-24): replace with the rr 16550-family
// UART IP once it exists; this module is its slot.

`timescale 1ps/1ps

module uart_16550 (
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    output wire        ip2intc_irpt,
    input  wire        freeze,

    input  wire [12:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [12:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    output wire        baudoutn,
    input  wire        ctsn,
    input  wire        dcdn,
    output wire        ddis,
    input  wire        dsrn,
    output wire        dtrn,
    output wire        out1n,
    output wire        out2n,
    input  wire        rin,
    output wire        rtsn,
    output wire        rxrdyn,
    input  wire        sin,
    output wire        sout,
    output wire        txrdyn
);
    wire clk = s_axi_aclk;
    wire rst = ~s_axi_aresetn;

    // Write: take AW and W together, one write per response.
    wire do_wr = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid & ~s_axi_rvalid;
    // Read: only when no write is being taken this cycle and no response is pending.
    wire do_rd = s_axi_arvalid & ~do_wr & ~s_axi_bvalid & ~s_axi_rvalid;

    assign s_axi_awready = do_wr;
    assign s_axi_wready  = do_wr;
    assign s_axi_arready = do_rd;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_rresp   = 2'b00;

    wire [2:0] wb_addr = do_wr ? s_axi_awaddr[4:2] : s_axi_araddr[4:2];
    wire [7:0] wb_dat_o;
    wire       rts_pad, dtr_pad, int_o;

    uart_regs #(.SIM(0)) regs (
        .clk          (clk),
        .wb_rst_i     (rst),
        .wb_addr_i    (wb_addr),
        .wb_dat_i     (s_axi_wdata[7:0]),
        .wb_dat_o     (wb_dat_o),
        .wb_we_i      (do_wr & s_axi_wstrb[0]),
        .wb_re_i      (do_rd),
        // {cts, dsr, ri, dcd}, active high here; the IP's pins are active low
        .modem_inputs ({~ctsn, ~dsrn, ~rin, ~dcdn}),
        .stx_pad_o    (sout),
        .srx_pad_i    (sin),
        .rts_pad_o    (rts_pad),
        .dtr_pad_o    (dtr_pad),
        .int_o        (int_o)
    );

    always @(posedge clk) begin
        if (rst) begin
            s_axi_bvalid <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
        end else begin
            if (do_wr)
                s_axi_bvalid <= 1'b1;
            else if (s_axi_bready)
                s_axi_bvalid <= 1'b0;
            if (do_rd) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rdata  <= {24'd0, wb_dat_o};
            end else if (s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end

    assign ip2intc_irpt = int_o;
    assign rtsn     = ~rts_pad;
    assign dtrn     = ~dtr_pad;
    assign baudoutn = 1'b1;
    assign ddis     = 1'b0;
    assign out1n    = 1'b1;
    assign out2n    = 1'b1;
    assign rxrdyn   = 1'b1;
    assign txrdyn   = 1'b1;

    // freeze (IP debug input) has no meaning here
    wire unused = freeze;
endmodule
