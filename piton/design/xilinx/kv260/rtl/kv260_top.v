// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// kv260_top - the Kria KV260 board top (protosyn -b kv260).
//
//   kv260_ps     the Zynq UltraScale+ PS, a block design built by
//                tools/src/proto/kv260/kv260_ps.tcl (wrapper kv260_ps_wrapper)
//     fab_clk      pl_clk0, 50 MHz  -> everything (one clock domain)
//     fab_aresetn  pl_resetn0       -> kv260_ctrl; AND RUN -> the system
//     S_AXI_HP0    <- kv260_shell's pmesh_hp_shim (PS DDR carve-out
//                     0x4000_0000 + 512 MB; the address map allows only that)
//     M_AXI_GP0    -> kv260_ctrl at 0xA000_0000 (RUN and the ID)
//   kv260_shell  the OpenPiton system + the shim
//
// Console: the system's 16550 is null-modemed inside the fabric to PS UART0
// on EMIO: system TX -> PS RXD, PS TXD -> system RX. Linux on the PS sees it
// as a non-console ttyPS; UART1 (MIO) stays the board console. No package pins.

`timescale 1ps/1ps

module kv260_top ();
  wire mw_uart_txd;   // PS UART0 TX -> core RX
  wire mw_uart_rxd;   // core TX     -> PS UART0 RX

    wire        fab_clk;
    wire [0:0]  fab_aresetn;
    wire        run;
    wire [7:0]  leds;
  wire [5:0] hp_axi_awid;
  wire [48:0] hp_axi_awaddr;
  wire [7:0] hp_axi_awlen;
  wire [2:0] hp_axi_awsize;
  wire [1:0] hp_axi_awburst;
  wire hp_axi_awlock;
  wire [3:0] hp_axi_awcache;
  wire [2:0] hp_axi_awprot;
  wire [3:0] hp_axi_awqos;
  wire hp_axi_awvalid;
  wire hp_axi_awready;
  wire [127:0] hp_axi_wdata;
  wire [15:0] hp_axi_wstrb;
  wire hp_axi_wlast;
  wire hp_axi_wvalid;
  wire hp_axi_wready;
  wire [5:0] hp_axi_bid;
  wire [1:0] hp_axi_bresp;
  wire hp_axi_bvalid;
  wire hp_axi_bready;
  wire [5:0] hp_axi_arid;
  wire [48:0] hp_axi_araddr;
  wire [7:0] hp_axi_arlen;
  wire [2:0] hp_axi_arsize;
  wire [1:0] hp_axi_arburst;
  wire hp_axi_arlock;
  wire [3:0] hp_axi_arcache;
  wire [2:0] hp_axi_arprot;
  wire [3:0] hp_axi_arqos;
  wire hp_axi_arvalid;
  wire hp_axi_arready;
  wire [5:0] hp_axi_rid;
  wire [127:0] hp_axi_rdata;
  wire [1:0] hp_axi_rresp;
  wire hp_axi_rlast;
  wire hp_axi_rvalid;
  wire hp_axi_rready;
  wire [31:0] gp_awaddr;
  wire [2:0] gp_awprot;
  wire gp_awvalid;
  wire gp_awready;
  wire [31:0] gp_wdata;
  wire [3:0] gp_wstrb;
  wire gp_wvalid;
  wire gp_wready;
  wire [1:0] gp_bresp;
  wire gp_bvalid;
  wire gp_bready;
  wire [31:0] gp_araddr;
  wire [2:0] gp_arprot;
  wire gp_arvalid;
  wire gp_arready;
  wire [31:0] gp_rdata;
  wire [1:0] gp_rresp;
  wire gp_rvalid;
  wire gp_rready;

    kv260_ps_wrapper ps (
        .S_AXI_HP0_awid     (hp_axi_awid),
        .S_AXI_HP0_awaddr   (hp_axi_awaddr[31:0]),   // clamp keeps it inside the carve-out
        .S_AXI_HP0_awlen    (hp_axi_awlen),
        .S_AXI_HP0_awsize   (hp_axi_awsize),
        .S_AXI_HP0_awburst  (hp_axi_awburst),
        .S_AXI_HP0_awlock   (hp_axi_awlock),
        .S_AXI_HP0_awcache  (hp_axi_awcache),
        .S_AXI_HP0_awprot   (hp_axi_awprot),
        .S_AXI_HP0_awqos    (hp_axi_awqos),
        .S_AXI_HP0_awvalid  (hp_axi_awvalid),
        .S_AXI_HP0_awready  (hp_axi_awready),
        .S_AXI_HP0_wdata    (hp_axi_wdata),
        .S_AXI_HP0_wstrb    (hp_axi_wstrb),
        .S_AXI_HP0_wlast    (hp_axi_wlast),
        .S_AXI_HP0_wvalid   (hp_axi_wvalid),
        .S_AXI_HP0_wready   (hp_axi_wready),
        .S_AXI_HP0_bid      (hp_axi_bid),
        .S_AXI_HP0_bresp    (hp_axi_bresp),
        .S_AXI_HP0_bvalid   (hp_axi_bvalid),
        .S_AXI_HP0_bready   (hp_axi_bready),
        .S_AXI_HP0_arid     (hp_axi_arid),
        .S_AXI_HP0_araddr   (hp_axi_araddr[31:0]),   // clamp keeps it inside the carve-out
        .S_AXI_HP0_arlen    (hp_axi_arlen),
        .S_AXI_HP0_arsize   (hp_axi_arsize),
        .S_AXI_HP0_arburst  (hp_axi_arburst),
        .S_AXI_HP0_arlock   (hp_axi_arlock),
        .S_AXI_HP0_arcache  (hp_axi_arcache),
        .S_AXI_HP0_arprot   (hp_axi_arprot),
        .S_AXI_HP0_arqos    (hp_axi_arqos),
        .S_AXI_HP0_arvalid  (hp_axi_arvalid),
        .S_AXI_HP0_arready  (hp_axi_arready),
        .S_AXI_HP0_rid      (hp_axi_rid),
        .S_AXI_HP0_rdata    (hp_axi_rdata),
        .S_AXI_HP0_rresp    (hp_axi_rresp),
        .S_AXI_HP0_rlast    (hp_axi_rlast),
        .S_AXI_HP0_rvalid   (hp_axi_rvalid),
        .S_AXI_HP0_rready   (hp_axi_rready),
        .M_AXI_GP0_awaddr   (gp_awaddr),
        .M_AXI_GP0_awprot   (gp_awprot),
        .M_AXI_GP0_awvalid  (gp_awvalid),
        .M_AXI_GP0_awready  (gp_awready),
        .M_AXI_GP0_wdata    (gp_wdata),
        .M_AXI_GP0_wstrb    (gp_wstrb),
        .M_AXI_GP0_wvalid   (gp_wvalid),
        .M_AXI_GP0_wready   (gp_wready),
        .M_AXI_GP0_bresp    (gp_bresp),
        .M_AXI_GP0_bvalid   (gp_bvalid),
        .M_AXI_GP0_bready   (gp_bready),
        .M_AXI_GP0_araddr   (gp_araddr),
        .M_AXI_GP0_arprot   (gp_arprot),
        .M_AXI_GP0_arvalid  (gp_arvalid),
        .M_AXI_GP0_arready  (gp_arready),
        .M_AXI_GP0_rdata    (gp_rdata),
        .M_AXI_GP0_rresp    (gp_rresp),
        .M_AXI_GP0_rvalid   (gp_rvalid),
        .M_AXI_GP0_rready   (gp_rready),
        .fab_clk          (fab_clk),
        .fab_aresetn      (fab_aresetn),
        .mw_uart_txd      (mw_uart_txd),
        .mw_uart_rxd      (mw_uart_rxd)
    );

    kv260_ctrl ctrl (
        .clk              (fab_clk),
        .rst_n            (fab_aresetn[0]),
        .s_axi_awaddr   (gp_awaddr[5:0]),
        .s_axi_awprot   (gp_awprot),
        .s_axi_awvalid  (gp_awvalid),
        .s_axi_awready  (gp_awready),
        .s_axi_wdata    (gp_wdata),
        .s_axi_wstrb    (gp_wstrb),
        .s_axi_wvalid   (gp_wvalid),
        .s_axi_wready   (gp_wready),
        .s_axi_bresp    (gp_bresp),
        .s_axi_bvalid   (gp_bvalid),
        .s_axi_bready   (gp_bready),
        .s_axi_araddr   (gp_araddr[5:0]),
        .s_axi_arprot   (gp_arprot),
        .s_axi_arvalid  (gp_arvalid),
        .s_axi_arready  (gp_arready),
        .s_axi_rdata    (gp_rdata),
        .s_axi_rresp    (gp_rresp),
        .s_axi_rvalid   (gp_rvalid),
        .s_axi_rready   (gp_rready),
        .run              (run)
    );

    kv260_shell shell (
        .clk              (fab_clk),
        .rst_n            (fab_aresetn[0] & run),
        .uart_tx          (mw_uart_rxd),
        .uart_rx          (mw_uart_txd),
        .leds             (leds),
        .hp_axi_awid     (hp_axi_awid),
        .hp_axi_awaddr   (hp_axi_awaddr),
        .hp_axi_awlen    (hp_axi_awlen),
        .hp_axi_awsize   (hp_axi_awsize),
        .hp_axi_awburst  (hp_axi_awburst),
        .hp_axi_awlock   (hp_axi_awlock),
        .hp_axi_awcache  (hp_axi_awcache),
        .hp_axi_awprot   (hp_axi_awprot),
        .hp_axi_awqos    (hp_axi_awqos),
        .hp_axi_awvalid  (hp_axi_awvalid),
        .hp_axi_awready  (hp_axi_awready),
        .hp_axi_wdata    (hp_axi_wdata),
        .hp_axi_wstrb    (hp_axi_wstrb),
        .hp_axi_wlast    (hp_axi_wlast),
        .hp_axi_wvalid   (hp_axi_wvalid),
        .hp_axi_wready   (hp_axi_wready),
        .hp_axi_bid      (hp_axi_bid),
        .hp_axi_bresp    (hp_axi_bresp),
        .hp_axi_bvalid   (hp_axi_bvalid),
        .hp_axi_bready   (hp_axi_bready),
        .hp_axi_arid     (hp_axi_arid),
        .hp_axi_araddr   (hp_axi_araddr),
        .hp_axi_arlen    (hp_axi_arlen),
        .hp_axi_arsize   (hp_axi_arsize),
        .hp_axi_arburst  (hp_axi_arburst),
        .hp_axi_arlock   (hp_axi_arlock),
        .hp_axi_arcache  (hp_axi_arcache),
        .hp_axi_arprot   (hp_axi_arprot),
        .hp_axi_arqos    (hp_axi_arqos),
        .hp_axi_arvalid  (hp_axi_arvalid),
        .hp_axi_arready  (hp_axi_arready),
        .hp_axi_rid      (hp_axi_rid),
        .hp_axi_rdata    (hp_axi_rdata),
        .hp_axi_rresp    (hp_axi_rresp),
        .hp_axi_rlast    (hp_axi_rlast),
        .hp_axi_rvalid   (hp_axi_rvalid),
        .hp_axi_rready   (hp_axi_rready)
    );
endmodule
