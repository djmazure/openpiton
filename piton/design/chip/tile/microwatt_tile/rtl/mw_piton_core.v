// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// mw_piton_core — the Microwatt core as an OpenPiton tile core
//
// What OpenPiton's tile.v instantiates for TILE_TYPE == MICROWATT_TILE. Its
// ports are exactly the tile's TRI wires, so the tile block is a plain
// name-for-name hookup (as PicoRV32's transducer is).
//
// Two things an OpenPiton tile core must do that the bare mw_tile does not:
//
// 1. WAIT FOR THE WAKEUP. OpenPiton starts a core with an interrupt packet
//    from the IOB (L1.5 returntype CPX_RESTYPE_INTERRUPT, l15.h.pyv 4'b0111);
//    PicoRV32 waits on pico_int, Balkind's 2020 Microwatt tile on
//    rst_pkt_rcv. The core is held in reset until that packet has been seen,
//    so it never fetches before the system is ready. The bridge stays out of
//    reset meanwhile: it must consume the wakeup message.
//
// 2. MAP MICROWATT'S 32-BIT BUS ONTO THE 40-BIT PHYSICAL SPACE. Microwatt's
//    I/O region is 0xC000_0000-0xCFFF_FFFF (256 MiB; loadstore1's real-mode
//    cache-inhibit rule and microwatt_soc.h), OpenPiton's device space is 0xFF_F000_0000-
//    0xFF_FFFF_FFFF (devices.xml: UART 0xFF_F0C2_C000, and the RV64
//    platform's CLINT/PLIC/bootrom at 0xFF_F1xx_xxxx) - also 256 MiB. So
//      core 0xCFxx_xxxx  ->  PA 0x98_00xx_xxxx   (see below)
//      core 0xCxxx_xxxx  ->  PA 0xFF_Fxxx_xxxx   (every other 0xC)
//      core anything else ->  PA zero-extended (DRAM from 0)
//    and the OpenPiton UART is at core address 0xC0C2_C000.
//
//    The 0xCF window reaches OpenPiton's inter-processor interrupt register,
//    L15_INT_VEC_DIS = PA 0x98_0000_0800 (l15.h.pyv). A non-cacheable 8-byte
//    store there becomes an INTERRUPT_FWD to the L2 of the tile named in the
//    data (noc1encoder.v: data[63] = 1 selects x = data[25:18], y = [33:26]),
//    which sends that tile an INTERRUPT: the same packet the fake IOB uses to
//    wake tile 0, so it is also how one Microwatt tile wakes another. Nothing
//    in devices.xml lives at 0xFF_FFxx_xxxx, so the carve-out hides no device.
//    This is only the address map. Whether an access is non-cacheable comes
//    from the core itself (dcache ext_nc -> wbd_nc), which also
//    covers ci-instructions and no-cache pages outside this window.

`default_nettype none

module mw_piton_core (
    input  wire        clk,
    input  wire        rst_n,
    // XICS level interrupt for this tile from the chipset; tied 0
    // by the tile when the system has no XICS. Asynchronous to clk in general
    // (chipset clock), so it is synchronised here before the core samples it.
    input  wire        ext_irq_async,
    // Microwatt syscon CPU_CTRL enable bit for this tile (chipset mw_syscon),
    // asynchronous. It starts a secondary tile the way a native
    // Microwatt SoC releases a core's reset: at address 0. Ignored on tile 0
    // (cpu_index 0), which the IOB wake-up packet starts.
    input  wire        run_async,
    // This tile's flat id (tile.v flat_tileid): the core's PIR, and the XICS
    // interrupt server whose output drives ext_irq_async.
    input  wire [7:0]  cpu_index,

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
    input  wire        l15_transducer_blockinitstore
);

    localparam [3:0] INT_RET = 4'b0111;         // l15.h.pyv CPX_RESTYPE_INTERRUPT

    (* ASYNC_REG = "TRUE" *) reg [1:0] run_sync;
    always @(posedge clk) run_sync <= {run_sync[0], run_async};
`ifdef MW_MUTANT_NO_RUN
    wire run = 1'b0;     // mutant: the syscon run wire is cut
`else
    wire run = run_sync[1] && cpu_index != 8'd0;
`endif

    // Sticky: a started tile keeps running if its run bit is cleared again
    // (Linux writes CPU_CTRL once).
    reg woken;
    always @(posedge clk) begin
        if (!rst_n)
            woken <= 1'b0;
        else if ((l15_transducer_val && l15_transducer_returntype == INT_RET) || run)
            woken <= 1'b1;
    end

    wire [39:0] core_address;
    assign transducer_l15_address = (core_address[31:24] == 8'hCF) ? {16'h9800, core_address[23:0]}
                                  : (core_address[31:28] == 4'hC)  ? {12'hFFF, core_address[27:0]}
                                  :                                  {8'h00, core_address[31:0]};

    wire        snp_cyc, snp_stb, snp_we, wbd_stall, terminated;
    wire [28:0] snp_adr;
    wire        icinv_stb;             // observable in sims
    wire [15:4] icinv_adr;

    (* ASYNC_REG = "TRUE" *) reg [1:0] ext_irq_sync;
    always @(posedge clk) ext_irq_sync <= {ext_irq_sync[0], ext_irq_async};

    mw_tile u_mw (
        .clk(clk), .rst_n(rst_n), .core_hold(~woken), .cpu_index(cpu_index),
        .ext_irq(ext_irq_sync[1]),
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
        .transducer_l15_address(core_address),
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
        .l15_transducer_blockinitstore(l15_transducer_blockinitstore),
        .snp_cyc(snp_cyc), .snp_stb(snp_stb), .snp_we(snp_we), .snp_adr(snp_adr),
        .icinv_stb(icinv_stb), .icinv_adr(icinv_adr),
        .wbd_stall(wbd_stall), .terminated(terminated)
    );

endmodule

`default_nettype wire
