// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// mw_noc_wb — a P-Mesh chipset device port to Microwatt's Wishbone I/O bus,
// shared by the Microwatt-derived chipset devices (mw_xics, mw_syscon).
// Moved here unchanged from mw_xics.v; ADDR_W is the
// byte-offset width of the device window (13 = 8 KB for mw_xics, 8 = 256 B
// for mw_syscon).
//
//   NoC2 in / NoC3 out
//     -> noc_axilite_bridge  SWAP_ENDIANESS=1, SLAVE_RESP_BYTEWIDTH=4
//     -> AXI-lite (64-bit) -> one-beat Wishbone I/O (32-bit), one access at
//        a time: cyc is held for the access, stb is a one-cycle pulse, the
//        slave's ack ends it.
//
// Byte lanes. The bridge does not place write data by address:
// wstrb is a size mask from lane 0 (1/3/F/FF) and the offset is in awaddr
// [BRG :375-395]. Store data arrives REPLICATED across the doubleword
// (tri_wb_bridge.v:295, pico_decoder.v:134) and SWAP_ENDIANESS turns the
// big-endian NoC word into little-endian lanes [BRG :408-415], so register
// byte (A mod 4) sits on lane (A mod 8) and the Wishbone word is
// wdata[32*A[2] +: 32], with sel = size mask << A[1:0]. On reads the bridge
// takes rdata[31:0] (SLAVE_RESP_BYTEWIDTH=4, [BRG :522-524]) and swaps it
// back, so the word is returned in little-endian lanes, replicated.
// The result on the Wishbone side is Microwatt's native I/O bus.
`include "define.tmp.h"

module mw_noc_wb #(
    parameter ADDR_W = 13
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        noc2_valid,
    input  wire [`NOC_DATA_WIDTH-1:0]  noc2_data,
    output wire                        noc2_ready,
    output wire                        noc3_valid,
    output wire [`NOC_DATA_WIDTH-1:0]  noc3_data,
    input  wire                        noc3_ready,

    output wire                        wb_cyc,
    output reg                         wb_stb,
    output wire                        wb_we,
    output wire [ADDR_W-3:0]           wb_adr,     // word address in the window
    output wire [3:0]                  wb_sel,
    output wire [31:0]                 wb_dat_w,
    input  wire [31:0]                 wb_dat_r,
    input  wire                        wb_ack
);
    wire [63:0] awaddr, araddr, wdata;
    wire [7:0]  wstrb;
    wire        awvalid, wvalid, arvalid, bready, rready;
    reg         awready, wready, arready, bvalid, rvalid;
    reg  [63:0] rdata;

    noc_axilite_bridge #(
        .SLAVE_RESP_BYTEWIDTH (4),
        .SWAP_ENDIANESS       (1)
    ) bridge (
        .clk                  (clk),
        .rst                  (~rst_n),
        .splitter_bridge_val  (noc2_valid),
        .splitter_bridge_data (noc2_data),
        .bridge_splitter_rdy  (noc2_ready),
        .bridge_splitter_val  (noc3_valid),
        .bridge_splitter_data (noc3_data),
        .splitter_bridge_rdy  (noc3_ready),
        .m_axi_awaddr         (awaddr),
        .m_axi_awvalid        (awvalid),
        .m_axi_awready        (awready),
        .m_axi_wdata          (wdata),
        .m_axi_wstrb          (wstrb),
        .m_axi_wvalid         (wvalid),
        .m_axi_wready         (wready),
        .m_axi_araddr         (araddr),
        .m_axi_arvalid        (arvalid),
        .m_axi_arready        (arready),
        .m_axi_rdata          (rdata),
        .m_axi_rresp          (2'b00),
        .m_axi_rvalid         (rvalid),
        .m_axi_rready         (rready),
        .m_axi_bresp          (2'b00),
        .m_axi_bvalid         (bvalid),
        .m_axi_bready         (bready),
        .w_reqbuf_size        (),
        .r_reqbuf_size        ()
    );

    // ---- AXI-lite slave -> one Wishbone access at a time ------------------
    localparam S_IDLE = 2'd0, S_WB = 2'd1, S_RSP = 2'd2;
    reg  [1:0]  state;
    reg         have_aw, have_w, is_wr;
    reg  [ADDR_W-1:0] a_off;   // byte offset inside the window
    reg  [63:0] w_data;
    reg  [3:0]  w_mask;         // wstrb[3:0]: the access size as a lane-0 mask

    assign wb_sel   = is_wr ? (w_mask << a_off[1:0]) : 4'hf;
    assign wb_dat_w = a_off[2] ? w_data[63:32] : w_data[31:0];
    assign wb_cyc   = (state == S_WB);
    assign wb_we    = is_wr;
    assign wb_adr   = a_off[ADDR_W-1:2];

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE; have_aw <= 1'b0; have_w <= 1'b0; is_wr <= 1'b0;
            awready <= 1'b0; wready <= 1'b0; arready <= 1'b0;
            bvalid <= 1'b0; rvalid <= 1'b0; wb_stb <= 1'b0;
            a_off <= {ADDR_W{1'b0}}; w_data <= 64'd0; w_mask <= 4'd0; rdata <= 64'd0;
        end else begin
            awready <= 1'b0; wready <= 1'b0; arready <= 1'b0;
            case (state)
            S_IDLE: begin
                if (awvalid && !have_aw && !awready) begin
                    have_aw <= 1'b1; awready <= 1'b1; a_off <= awaddr[ADDR_W-1:0];
                end
                if (wvalid && !have_w && !wready) begin
                    have_w <= 1'b1; wready <= 1'b1;
                    w_data <= wdata; w_mask <= wstrb[3:0];
                end
                if (have_aw && have_w) begin
                    is_wr <= 1'b1; wb_stb <= 1'b1; state <= S_WB;
                end else if (!have_aw && !have_w && !awvalid && !wvalid &&
                             arvalid && !arready) begin
                    arready <= 1'b1; a_off <= araddr[ADDR_W-1:0];
                    is_wr <= 1'b0; wb_stb <= 1'b1; state <= S_WB;
                end
            end
            S_WB: begin
                wb_stb <= 1'b0;                 // one-cycle strobe; cyc held
                if (wb_ack) begin
                    if (is_wr) begin
                        bvalid <= 1'b1; have_aw <= 1'b0; have_w <= 1'b0;
                    end else begin
                        rvalid <= 1'b1; rdata <= {2{wb_dat_r}};
                    end
                    state <= S_RSP;
                end
            end
            S_RSP: begin
                if (bvalid && bready) begin bvalid <= 1'b0; state <= S_IDLE; end
                if (rvalid && rready) begin rvalid <= 1'b0; state <= S_IDLE; end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
