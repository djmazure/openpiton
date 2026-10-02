// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// mw_syscon — Microwatt syscon as an OpenPiton chipset device (OPN-P2.24).
// Contract: hw/ip/mw_syscon/requirements.yml.
//
//   NoC2 in / NoC3 out (device port "mw_syscon", 256-byte window at
//   PA 0xFF_F000_0000 = core 0xC000_0000, where Linux's microwatt smp.c
//   looks for it)
//     -> mw_noc_wb (hw/ip/mw_noc_wb: noc_axilite_bridge, AXI-lite -> one-beat
//        Wishbone I/O, 32-bit, little-endian registers as readl/writel expect)
//     -> mw_syscon_flat: Microwatt's own syscon.vhdl, NCPUS = the tile count
//
// run[i] = ~syscon.core_reset[i], i.e. CPU_CTRL enable bit i. It goes to tile i
// as a sideband wire (PITON_MW_SYSCON), where it starts the core the way
// Microwatt's core reset release does: "they start at location 0" (smp.c).
// run[0] is produced but the system does not route it: tile 0 is started by
// OpenPiton's IOB wake-up packet (SYSCON-DIV-001). A run bit that is cleared
// again does not stop a started tile (SYSCON-DIV-002, a property of the
// tile's wake latch, not of this device).
`include "define.tmp.h"

module mw_syscon #(
    parameter NCPUS = 1
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        noc2_valid,
    input  wire [`NOC_DATA_WIDTH-1:0]  noc2_data,
    output wire                        noc2_ready,
    output wire                        noc3_valid,
    output wire [`NOC_DATA_WIDTH-1:0]  noc3_data,
    input  wire                        noc3_ready,

    output wire [NCPUS-1:0]            run_o
);
    wire             wb_cyc, wb_stb, wb_we, wb_ack;
    wire [5:0]       wb_adr;
    wire [3:0]       wb_sel;
    wire [31:0]      wb_dat_w, wb_dat_r;
    wire [NCPUS-1:0] core_reset;

    assign run_o = ~core_reset;

    mw_noc_wb #(.ADDR_W(8)) noc_wb (
        .clk        (clk),        .rst_n      (rst_n),
        .noc2_valid (noc2_valid), .noc2_data  (noc2_data),  .noc2_ready (noc2_ready),
        .noc3_valid (noc3_valid), .noc3_data  (noc3_data),  .noc3_ready (noc3_ready),
        .wb_cyc     (wb_cyc),     .wb_stb     (wb_stb),     .wb_we      (wb_we),
        .wb_adr     (wb_adr),     .wb_sel     (wb_sel),
        .wb_dat_w   (wb_dat_w),   .wb_dat_r   (wb_dat_r),   .wb_ack     (wb_ack)
    );

    // GHDL fixes generics at conversion, so each supported NCPUS is its own
    // converted netlist: mw_syscon_flat (1 CPU, hook convert_mw_syscon) and
    // mw_syscon_flat_n2 (2 CPUs, convert_mw_syscon2), both in Flist.syscon.
    // Any other count is refused (SYSCON-CFG-001) rather than reporting the
    // wrong NCPUS to Linux.
    generate if (NCPUS == 1) begin : g_n1
        mw_syscon_flat syscon (
            .clk (clk), .rst (~rst_n),
            .wb_cyc (wb_cyc), .wb_stb (wb_stb), .wb_we (wb_we), .wb_adr (wb_adr),
            .wb_sel (wb_sel), .wb_dat_w (wb_dat_w), .wb_dat_r (wb_dat_r), .wb_ack (wb_ack),
            .core_reset (core_reset)
        );
    end else if (NCPUS == 2) begin : g_n2
        mw_syscon_flat_n2 syscon (
            .clk (clk), .rst (~rst_n),
            .wb_cyc (wb_cyc), .wb_stb (wb_stb), .wb_we (wb_we), .wb_adr (wb_adr),
            .wb_sel (wb_sel), .wb_dat_w (wb_dat_w), .wb_dat_r (wb_dat_r), .wb_ack (wb_ack),
            .core_reset (core_reset)
        );
    end else begin : g_ncpus_unsupported
        initial $fatal(1, "mw_syscon: NCPUS=%0d has no converted netlist (1 and 2 do)", NCPUS);
    end endgenerate
endmodule
