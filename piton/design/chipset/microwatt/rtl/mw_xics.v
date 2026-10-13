// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// mw_xics — Microwatt XICS as an OpenPiton chipset device.
//
//   NoC2 in / NoC3 out (device port "mw_xics", 8 KB window)
//     -> mw_noc_wb (mw_noc_wb.v: noc_axilite_bridge, AXI-lite -> one-beat
//        Wishbone I/O, 32-bit; the byte-lane mapping is documented there)
//     -> mw_xics_flat: xics_icp at +0x0000, xics_ics at +0x1000
//
// Byte lanes: see mw_noc_wb.v.
`include "define.tmp.h"

module mw_xics #(
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

    input  wire [15:0]                 int_level,   // [0] = XICS source 0x10
    output wire [NCPUS-1:0]            core_irq
);
    wire        wb_cyc, wb_stb, wb_we, wb_ack;
    wire [10:0] wb_adr;
    wire [3:0]  wb_sel;
    wire [31:0] wb_dat_w, wb_dat_r;

    mw_noc_wb #(.ADDR_W(13)) noc_wb (
        .clk        (clk),        .rst_n      (rst_n),
        .noc2_valid (noc2_valid), .noc2_data  (noc2_data),  .noc2_ready (noc2_ready),
        .noc3_valid (noc3_valid), .noc3_data  (noc3_data),  .noc3_ready (noc3_ready),
        .wb_cyc     (wb_cyc),     .wb_stb     (wb_stb),     .wb_we      (wb_we),
        .wb_adr     (wb_adr),     .wb_sel     (wb_sel),
        .wb_dat_w   (wb_dat_w),   .wb_dat_r   (wb_dat_r),   .wb_ack     (wb_ack)
    );

    // GHDL fixes generics at conversion, so each supported NCPUS is its own
    // converted netlist: mw_xics_flat (1 CPU, convert_microwatt xics)
    // and mw_xics_flat_n2 (2 CPUs, convert_microwatt xics2), both in Flist.mw_xics.
    // Any other width is refused rather than silently driving the wrong tiles;
    // add a conversion hook to support it.
    generate if (NCPUS == 1) begin : g_n1
        mw_xics_flat xics (
            .clk       (clk),
            .rst       (~rst_n),
            .wb_cyc    (wb_cyc),
            .wb_stb    (wb_stb),
            .wb_we     (wb_we),
            .wb_adr    (wb_adr),
            .wb_sel    (wb_sel),
            .wb_dat_w  (wb_dat_w),
            .wb_dat_r  (wb_dat_r),
            .wb_ack    (wb_ack),
            .int_level (int_level),
            .core_irq  (core_irq)
        );
    end else if (NCPUS == 2) begin : g_n2
        mw_xics_flat_n2 xics (
            .clk       (clk),
            .rst       (~rst_n),
            .wb_cyc    (wb_cyc),
            .wb_stb    (wb_stb),
            .wb_we     (wb_we),
            .wb_adr    (wb_adr),
            .wb_sel    (wb_sel),
            .wb_dat_w  (wb_dat_w),
            .wb_dat_r  (wb_dat_r),
            .wb_ack    (wb_ack),
            .int_level (int_level),
            .core_irq  (core_irq)
        );
    end else begin : g_ncpus_unsupported
        initial $fatal(1, "mw_xics: NCPUS=%0d has no converted netlist (1 and 2 do)", NCPUS);
    end endgenerate
endmodule
