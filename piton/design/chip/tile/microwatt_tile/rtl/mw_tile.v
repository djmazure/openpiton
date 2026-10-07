// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// mw_tile — converted Microwatt core + tri_wb_bridge, TRI ports at the top (OPN-P2.3)
//
// The TRI ports keep tri_wb_bridge's exact names, so the same L1.5 model and
// always-on checker (hw/sim/cocotb/tests/tri_harness.py) drive this top
// unchanged. The snoop port and wbd_stall are also brought out, read-only, so
// the harness can check that every invalidation reaches the core (TWB-NEG-004)
// and is presented while the data master is stalled (TWB-COH-002).
//
// mw_core_flat is generated from Microwatt VHDL by hw/scripts/convert_microwatt.sh
// (rr hooks.pre_sim); it is never edited by hand.

`default_nettype none

module mw_tile (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        core_hold,       // 1: core held in reset, bridge live (OPN-P2.4 wakeup)
    input  wire [7:0]  cpu_index,       // PIR of this core: the tile's flat id (OPN-P2.15)
    input  wire        ext_irq,         // XICS level interrupt, synchronised to clk (OPN-P2.15)

    // TRI: bridge -> L1.5
    output wire        transducer_l15_val,
    output wire [4:0]  transducer_l15_rqtype,
    output wire [3:0]  transducer_l15_amo_op,
    output wire        transducer_l15_nc,
    output wire [2:0]  transducer_l15_size,
    output wire [0:0]  transducer_l15_threadid,
    output wire        transducer_l15_prefetch,
    output wire        transducer_l15_invalidate_cacheline,
    output wire        transducer_l15_blockstore,
    output wire        transducer_l15_blockinitstore,
    output wire [1:0]  transducer_l15_l1rplway,
    output wire [39:0] transducer_l15_address,
    output wire [63:0] transducer_l15_data,
    output wire [63:0] transducer_l15_data_next_entry,
    output wire [32:0] transducer_l15_csm_data,
    output wire        transducer_l15_req_ack,

    // TRI: L1.5 -> bridge
    input  wire        l15_transducer_ack,
    input  wire        l15_transducer_header_ack,
    input  wire        l15_transducer_val,
    input  wire [3:0]  l15_transducer_returntype,
    input  wire        l15_transducer_l2miss,
    input  wire [1:0]  l15_transducer_error,
    input  wire        l15_transducer_noncacheable,
    input  wire        l15_transducer_atomic,
    input  wire [0:0]  l15_transducer_threadid,
    input  wire        l15_transducer_prefetch,
    input  wire        l15_transducer_f4b,
    input  wire [63:0] l15_transducer_data_0,
    input  wire [63:0] l15_transducer_data_1,
    input  wire [63:0] l15_transducer_data_2,
    input  wire [63:0] l15_transducer_data_3,
    input  wire        l15_transducer_inval_icache_all_way,
    input  wire        l15_transducer_inval_dcache_all_way,
    input  wire [15:4] l15_transducer_inval_address_15_4,
    input  wire        l15_transducer_cross_invalidate,
    input  wire [1:0]  l15_transducer_cross_invalidate_way,
    input  wire        l15_transducer_inval_dcache_inval,
    input  wire        l15_transducer_inval_icache_inval,
    input  wire [1:0]  l15_transducer_inval_way,
    input  wire        l15_transducer_blockinitstore,

    // observation only
    output wire        snp_cyc,
    output wire        snp_stb,
    output wire        snp_we,
    output wire [28:0] snp_adr,
    output wire        icinv_stb,     // I-cache index invalidation (TWB-COH-003), observable
    output wire [15:4] icinv_adr,
    output wire        wbd_stall,
    output wire        terminated
);

    wire        wbi_cyc, wbi_stb, wbi_ack, wbi_stall;
    wire [28:0] wbi_adr;
    wire [7:0]  wbi_sel;
    wire [63:0] wbi_dat_r;

    wire        wbd_cyc, wbd_stb, wbd_we, wbd_nc, wbd_reserve, wbd_ack, wbd_sc_fail;
    wire [28:0] wbd_adr;
    wire [7:0]  wbd_sel;
    wire [63:0] wbd_dat_w, wbd_dat_r;

    // MW_MUTANT_NO_SNOOP (test-only mutation, OPN-P2.5): the bridge still
    // produces every snoop, but the core never sees one, so its L1D keeps
    // lines the L1.5 has invalidated. The coherence test must go red with it.
    wire core_snp_stb;
`ifdef MW_MUTANT_NO_SNOOP
    assign core_snp_stb = 1'b0;
`else
    assign core_snp_stb = snp_stb;
`endif

    // MW_MUTANT_NO_ICINV (test-only mutation, OPN-P2.13): the bridge still
    // forwards every L1.5 I-side invalidation, but the core never sees one, so
    // its icache keeps code another tile rewrote. test_piton_mw_icoh must go
    // red with it.
    wire core_icinv_stb;
`ifdef MW_MUTANT_NO_ICINV
    assign core_icinv_stb = 1'b0;
`else
    assign core_icinv_stb = icinv_stb;
`endif

    // Test-only mutations (OPN-P2.15), each must turn test_piton_mw_smp red:
    //   MW_MUTANT_TB_PER_WAKE - the timebase restarts when THIS tile is woken
    //                           (the pre-P2.15 wiring), so tiles disagree;
    //   MW_MUTANT_PIR0        - every core reads PIR 0 (the baked-in CPU_INDEX).
    wire       core_tb_rst;
    wire [7:0] core_cpu_index;
`ifdef MW_MUTANT_TB_PER_WAKE
    assign core_tb_rst = ~rst_n | core_hold;
`else
    assign core_tb_rst = ~rst_n;
`endif
`ifdef MW_MUTANT_PIR0
    assign core_cpu_index = 8'd0;
`else
    assign core_cpu_index = cpu_index;
`endif

    mw_core_flat u_core (
        .clk(clk), .rst(~rst_n | core_hold),
        // timebase: system reset only, so every tile's timebase agrees (OPN-P2.15)
        .tb_rst(core_tb_rst), .cpu_index(core_cpu_index),
        .wbi_cyc(wbi_cyc), .wbi_stb(wbi_stb), .wbi_adr(wbi_adr), .wbi_sel(wbi_sel),
        .wbi_dat_r(wbi_dat_r), .wbi_ack(wbi_ack), .wbi_stall(wbi_stall),
        .wbd_cyc(wbd_cyc), .wbd_stb(wbd_stb), .wbd_we(wbd_we), .wbd_adr(wbd_adr),
        .wbd_sel(wbd_sel), .wbd_dat_w(wbd_dat_w), .wbd_nc(wbd_nc), .wbd_reserve(wbd_reserve),
        .wbd_dat_r(wbd_dat_r), .wbd_ack(wbd_ack), .wbd_stall(wbd_stall), .wbd_sc_fail(wbd_sc_fail),
        .snp_cyc(snp_cyc), .snp_stb(core_snp_stb), .snp_we(snp_we), .snp_adr(snp_adr),
        .icinv_stb(core_icinv_stb), .icinv_adr(icinv_adr),
        .ext_irq(ext_irq),
        .terminated(terminated)
    );

    tri_wb_bridge #(.L1D_INDEX_BITS(6)) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .wbi_cyc(wbi_cyc), .wbi_stb(wbi_stb), .wbi_adr(wbi_adr), .wbi_sel(wbi_sel),
        .wbi_dat_r(wbi_dat_r), .wbi_ack(wbi_ack), .wbi_stall(wbi_stall),
        .wbd_cyc(wbd_cyc), .wbd_stb(wbd_stb), .wbd_we(wbd_we), .wbd_adr(wbd_adr),
        .wbd_sel(wbd_sel), .wbd_dat_w(wbd_dat_w), .wbd_nc(wbd_nc), .wbd_reserve(wbd_reserve),
        .wbd_dat_r(wbd_dat_r), .wbd_ack(wbd_ack), .wbd_stall(wbd_stall), .wbd_sc_fail(wbd_sc_fail),
        .snp_cyc(snp_cyc), .snp_stb(snp_stb), .snp_we(snp_we), .snp_adr(snp_adr),
        .icinv_stb(icinv_stb), .icinv_adr(icinv_adr),
        .transducer_l15_val(transducer_l15_val),
        .transducer_l15_rqtype(transducer_l15_rqtype),
        .transducer_l15_amo_op(transducer_l15_amo_op),
        .transducer_l15_nc(transducer_l15_nc),
        .transducer_l15_size(transducer_l15_size),
        .transducer_l15_threadid(transducer_l15_threadid),
        .transducer_l15_prefetch(transducer_l15_prefetch),
        .transducer_l15_invalidate_cacheline(transducer_l15_invalidate_cacheline),
        .transducer_l15_blockstore(transducer_l15_blockstore),
        .transducer_l15_blockinitstore(transducer_l15_blockinitstore),
        .transducer_l15_l1rplway(transducer_l15_l1rplway),
        .transducer_l15_address(transducer_l15_address),
        .transducer_l15_data(transducer_l15_data),
        .transducer_l15_data_next_entry(transducer_l15_data_next_entry),
        .transducer_l15_csm_data(transducer_l15_csm_data),
        .transducer_l15_req_ack(transducer_l15_req_ack),
        .l15_transducer_ack(l15_transducer_ack),
        .l15_transducer_header_ack(l15_transducer_header_ack),
        .l15_transducer_val(l15_transducer_val),
        .l15_transducer_returntype(l15_transducer_returntype),
        .l15_transducer_l2miss(l15_transducer_l2miss),
        .l15_transducer_error(l15_transducer_error),
        .l15_transducer_noncacheable(l15_transducer_noncacheable),
        .l15_transducer_atomic(l15_transducer_atomic),
        .l15_transducer_threadid(l15_transducer_threadid),
        .l15_transducer_prefetch(l15_transducer_prefetch),
        .l15_transducer_f4b(l15_transducer_f4b),
        .l15_transducer_data_0(l15_transducer_data_0),
        .l15_transducer_data_1(l15_transducer_data_1),
        .l15_transducer_data_2(l15_transducer_data_2),
        .l15_transducer_data_3(l15_transducer_data_3),
        .l15_transducer_inval_icache_all_way(l15_transducer_inval_icache_all_way),
        .l15_transducer_inval_dcache_all_way(l15_transducer_inval_dcache_all_way),
        .l15_transducer_inval_address_15_4(l15_transducer_inval_address_15_4),
        .l15_transducer_cross_invalidate(l15_transducer_cross_invalidate),
        .l15_transducer_cross_invalidate_way(l15_transducer_cross_invalidate_way),
        .l15_transducer_inval_dcache_inval(l15_transducer_inval_dcache_inval),
        .l15_transducer_inval_icache_inval(l15_transducer_inval_icache_inval),
        .l15_transducer_inval_way(l15_transducer_inval_way),
        .l15_transducer_blockinitstore(l15_transducer_blockinitstore)
    );

endmodule

`default_nettype wire
