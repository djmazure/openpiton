// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// kv260_shell - the OpenPiton system for the Kria KV260 (KV260_BOARD, which
// defines PITONSYS_SHELL): `system` plus pmesh_hp_shim onto the PS's
// S_AXI_HP0. kv260_top wraps it with the PS and the run-control register.
//
// One clock: under PITONSYS_SHELL + PITON_FULL_SYSTEM + PITON_NO_CHIP_BRIDGE,
// chipset_clk = sys_clk and core_ref_clk/io_clk come from the chipset
// (chipset.v), and the chip/chipset NoCs are synchronous; mc_clk must be the
// same clock too (the behavioural afifo_w64_d128_std used for this board is
// only safe with equal clocks). rst_n is PS pl_resetn0 AND the RUN bit.
//
// Wake-up needs no external help: ciop_iob sends tile 0 its wake interrupt
// ~40000 cycles after reset, once ddr_ready is high (io_ctrl_rst_n).

`timescale 1ps/1ps

module kv260_shell (
    input             clk,
    input             rst_n,
    output            uart_tx,
    input             uart_rx,
    output [7:0]      leds,
    output [5:0]      hp_axi_awid,
    output [48:0]     hp_axi_awaddr,
    output [7:0]      hp_axi_awlen,
    output [2:0]      hp_axi_awsize,
    output [1:0]      hp_axi_awburst,
    output            hp_axi_awlock,
    output [3:0]      hp_axi_awcache,
    output [2:0]      hp_axi_awprot,
    output [3:0]      hp_axi_awqos,
    output            hp_axi_awvalid,
    input             hp_axi_awready,
    output [127:0]    hp_axi_wdata,
    output [15:0]     hp_axi_wstrb,
    output            hp_axi_wlast,
    output            hp_axi_wvalid,
    input             hp_axi_wready,
    input  [5:0]      hp_axi_bid,
    input  [1:0]      hp_axi_bresp,
    input             hp_axi_bvalid,
    output            hp_axi_bready,
    output [5:0]      hp_axi_arid,
    output [48:0]     hp_axi_araddr,
    output [7:0]      hp_axi_arlen,
    output [2:0]      hp_axi_arsize,
    output [1:0]      hp_axi_arburst,
    output            hp_axi_arlock,
    output [3:0]      hp_axi_arcache,
    output [2:0]      hp_axi_arprot,
    output [3:0]      hp_axi_arqos,
    output            hp_axi_arvalid,
    input             hp_axi_arready,
    input  [5:0]      hp_axi_rid,
    input  [127:0]    hp_axi_rdata,
    input  [1:0]      hp_axi_rresp,
    input             hp_axi_rlast,
    input             hp_axi_rvalid,
    output            hp_axi_rready
);
    wire [`AXI4_ID_WIDTH     -1:0]    m_axi_awid;
    wire [`AXI4_ADDR_WIDTH   -1:0]    m_axi_awaddr;
    wire [`AXI4_LEN_WIDTH    -1:0]    m_axi_awlen;
    wire [`AXI4_SIZE_WIDTH   -1:0]    m_axi_awsize;
    wire [`AXI4_BURST_WIDTH  -1:0]    m_axi_awburst;
    wire                              m_axi_awlock;
    wire [`AXI4_CACHE_WIDTH  -1:0]    m_axi_awcache;
    wire [`AXI4_PROT_WIDTH   -1:0]    m_axi_awprot;
    wire [`AXI4_QOS_WIDTH    -1:0]    m_axi_awqos;
    wire [`AXI4_REGION_WIDTH -1:0]    m_axi_awregion;
    wire [`AXI4_USER_WIDTH   -1:0]    m_axi_awuser;
    wire                              m_axi_awvalid;
    wire                              m_axi_awready;
    wire [`AXI4_ID_WIDTH     -1:0]    m_axi_wid;
    wire [`AXI4_DATA_WIDTH   -1:0]    m_axi_wdata;
    wire [`AXI4_STRB_WIDTH   -1:0]    m_axi_wstrb;
    wire                              m_axi_wlast;
    wire [`AXI4_USER_WIDTH   -1:0]    m_axi_wuser;
    wire                              m_axi_wvalid;
    wire                              m_axi_wready;
    wire [`AXI4_ID_WIDTH     -1:0]    m_axi_arid;
    wire [`AXI4_ADDR_WIDTH   -1:0]    m_axi_araddr;
    wire [`AXI4_LEN_WIDTH    -1:0]    m_axi_arlen;
    wire [`AXI4_SIZE_WIDTH   -1:0]    m_axi_arsize;
    wire [`AXI4_BURST_WIDTH  -1:0]    m_axi_arburst;
    wire                              m_axi_arlock;
    wire [`AXI4_CACHE_WIDTH  -1:0]    m_axi_arcache;
    wire [`AXI4_PROT_WIDTH   -1:0]    m_axi_arprot;
    wire [`AXI4_QOS_WIDTH    -1:0]    m_axi_arqos;
    wire [`AXI4_REGION_WIDTH -1:0]    m_axi_arregion;
    wire [`AXI4_USER_WIDTH   -1:0]    m_axi_aruser;
    wire                              m_axi_arvalid;
    wire                              m_axi_arready;
    wire [`AXI4_ID_WIDTH     -1:0]    m_axi_rid;
    wire [`AXI4_DATA_WIDTH   -1:0]    m_axi_rdata;
    wire [`AXI4_RESP_WIDTH   -1:0]    m_axi_rresp;
    wire                              m_axi_rlast;
    wire [`AXI4_USER_WIDTH   -1:0]    m_axi_ruser;
    wire                              m_axi_rvalid;
    wire                              m_axi_rready;
    wire [`AXI4_ID_WIDTH     -1:0]    m_axi_bid;
    wire [`AXI4_RESP_WIDTH   -1:0]    m_axi_bresp;
    wire [`AXI4_USER_WIDTH   -1:0]    m_axi_buser;
    wire                              m_axi_bvalid;
    wire                              m_axi_bready;

    system system (
        .sys_clk         (clk),
        .mc_clk          (clk),
        .sys_rst_n       (rst_n),
        .m_axi_awid      (m_axi_awid),
        .m_axi_awaddr    (m_axi_awaddr),
        .m_axi_awlen     (m_axi_awlen),
        .m_axi_awsize    (m_axi_awsize),
        .m_axi_awburst   (m_axi_awburst),
        .m_axi_awlock    (m_axi_awlock),
        .m_axi_awcache   (m_axi_awcache),
        .m_axi_awprot    (m_axi_awprot),
        .m_axi_awqos     (m_axi_awqos),
        .m_axi_awregion  (m_axi_awregion),
        .m_axi_awuser    (m_axi_awuser),
        .m_axi_awvalid   (m_axi_awvalid),
        .m_axi_awready   (m_axi_awready),
        .m_axi_wid       (m_axi_wid),
        .m_axi_wdata     (m_axi_wdata),
        .m_axi_wstrb     (m_axi_wstrb),
        .m_axi_wlast     (m_axi_wlast),
        .m_axi_wuser     (m_axi_wuser),
        .m_axi_wvalid    (m_axi_wvalid),
        .m_axi_wready    (m_axi_wready),
        .m_axi_arid      (m_axi_arid),
        .m_axi_araddr    (m_axi_araddr),
        .m_axi_arlen     (m_axi_arlen),
        .m_axi_arsize    (m_axi_arsize),
        .m_axi_arburst   (m_axi_arburst),
        .m_axi_arlock    (m_axi_arlock),
        .m_axi_arcache   (m_axi_arcache),
        .m_axi_arprot    (m_axi_arprot),
        .m_axi_arqos     (m_axi_arqos),
        .m_axi_arregion  (m_axi_arregion),
        .m_axi_aruser    (m_axi_aruser),
        .m_axi_arvalid   (m_axi_arvalid),
        .m_axi_arready   (m_axi_arready),
        .m_axi_rid       (m_axi_rid),
        .m_axi_rdata     (m_axi_rdata),
        .m_axi_rresp     (m_axi_rresp),
        .m_axi_rlast     (m_axi_rlast),
        .m_axi_ruser     (m_axi_ruser),
        .m_axi_rvalid    (m_axi_rvalid),
        .m_axi_rready    (m_axi_rready),
        .m_axi_bid       (m_axi_bid),
        .m_axi_bresp     (m_axi_bresp),
        .m_axi_buser     (m_axi_buser),
        .m_axi_bvalid    (m_axi_bvalid),
        .m_axi_bready    (m_axi_bready),
        .ddr_ready       (1'b1),        // PS DDR is up before RUN releases the PL
        .uart_tx         (uart_tx),
        .uart_rx         (uart_rx),
        .sw              (8'd0),
        .leds            (leds)
    );

    assign m_axi_ruser = {`AXI4_USER_WIDTH{1'b0}};
    assign m_axi_buser = {`AXI4_USER_WIDTH{1'b0}};

    // KV260 carve-out: P-Mesh PA A -> PS 0x4000_0000 + A, 512 MB
    pmesh_hp_shim #(
        .OFFSET   (64'h0000_0000_4000_0000),
        .WINDOW   (64'h0000_0000_2000_0000),
        .M_ADDR_W (49)
    ) hp_shim (
        .clk              (clk),
        .rst_n            (rst_n),
        .s_axi_awid      (m_axi_awid),
        .s_axi_awaddr    (m_axi_awaddr),
        .s_axi_awlen     (m_axi_awlen),
        .s_axi_awsize    (m_axi_awsize),
        .s_axi_awburst   (m_axi_awburst),
        .s_axi_awvalid   (m_axi_awvalid),
        .s_axi_awready   (m_axi_awready),
        .s_axi_wdata     (m_axi_wdata),
        .s_axi_wstrb     (m_axi_wstrb),
        .s_axi_wlast     (m_axi_wlast),
        .s_axi_wvalid    (m_axi_wvalid),
        .s_axi_wready    (m_axi_wready),
        .s_axi_bid       (m_axi_bid),
        .s_axi_bresp     (m_axi_bresp),
        .s_axi_bvalid    (m_axi_bvalid),
        .s_axi_bready    (m_axi_bready),
        .s_axi_arid      (m_axi_arid),
        .s_axi_araddr    (m_axi_araddr),
        .s_axi_arlen     (m_axi_arlen),
        .s_axi_arsize    (m_axi_arsize),
        .s_axi_arburst   (m_axi_arburst),
        .s_axi_arvalid   (m_axi_arvalid),
        .s_axi_arready   (m_axi_arready),
        .s_axi_rid       (m_axi_rid),
        .s_axi_rdata     (m_axi_rdata),
        .s_axi_rresp     (m_axi_rresp),
        .s_axi_rlast     (m_axi_rlast),
        .s_axi_rvalid    (m_axi_rvalid),
        .s_axi_rready    (m_axi_rready),
        .m_axi_awid      (hp_axi_awid),
        .m_axi_awaddr    (hp_axi_awaddr),
        .m_axi_awlen     (hp_axi_awlen),
        .m_axi_awsize    (hp_axi_awsize),
        .m_axi_awburst   (hp_axi_awburst),
        .m_axi_awlock    (hp_axi_awlock),
        .m_axi_awcache   (hp_axi_awcache),
        .m_axi_awprot    (hp_axi_awprot),
        .m_axi_awqos     (hp_axi_awqos),
        .m_axi_awvalid   (hp_axi_awvalid),
        .m_axi_awready   (hp_axi_awready),
        .m_axi_wdata     (hp_axi_wdata),
        .m_axi_wstrb     (hp_axi_wstrb),
        .m_axi_wlast     (hp_axi_wlast),
        .m_axi_wvalid    (hp_axi_wvalid),
        .m_axi_wready    (hp_axi_wready),
        .m_axi_bid       (hp_axi_bid),
        .m_axi_bresp     (hp_axi_bresp),
        .m_axi_bvalid    (hp_axi_bvalid),
        .m_axi_bready    (hp_axi_bready),
        .m_axi_arid      (hp_axi_arid),
        .m_axi_araddr    (hp_axi_araddr),
        .m_axi_arlen     (hp_axi_arlen),
        .m_axi_arsize    (hp_axi_arsize),
        .m_axi_arburst   (hp_axi_arburst),
        .m_axi_arlock    (hp_axi_arlock),
        .m_axi_arcache   (hp_axi_arcache),
        .m_axi_arprot    (hp_axi_arprot),
        .m_axi_arqos     (hp_axi_arqos),
        .m_axi_arvalid   (hp_axi_arvalid),
        .m_axi_arready   (hp_axi_arready),
        .m_axi_rid       (hp_axi_rid),
        .m_axi_rdata     (hp_axi_rdata),
        .m_axi_rresp     (hp_axi_rresp),
        .m_axi_rlast     (hp_axi_rlast),
        .m_axi_rvalid    (hp_axi_rvalid),
        .m_axi_rready    (hp_axi_rready)
    );

endmodule
