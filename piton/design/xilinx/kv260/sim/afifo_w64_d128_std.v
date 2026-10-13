// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// SIMULATION MODEL of OpenPiton's `afifo_w64_d128_std` - a Xilinx FIFO
// Generator core (design/chipset/xilinx/*/ip_cores/afifo_w64_d128_std.xci:
// Independent_Clocks_Block_RAM, 64 x 128, standard read mode,
// Full_Flags_Reset_Value 1, Dout_Reset_Value 0). The KV260 memory path needs it because
// noc_bidir_afifo (f1_mc_top's clock crossing) instantiates it and only the
// .xci exists; a synthesis build gets the real core from the IP flow, never
// from this file.
//
// The interface behaviour noc_bidir_afifo relies on, modelled exactly:
//   * standard (not first-word-fall-through) read: `dout` updates on the
//     rd_clk edge that accepts rd_en, i.e. one cycle after rd_en;
//   * rd_en while empty and wr_en while full are ignored (the core's
//     underflow/overflow protection) - noc_bidir_afifo drives rd_en = rdy
//     whatever `empty` says;
//   * `full` reads 1 and `empty` 1 while rst is high.
// Flag latency is NOT idealised: each side sees the other's pointer through a
// two-flop synchroniser, as the core's gray-code crossing does, so `empty`
// falls >= 2 rd_clk edges after a write and `full` clears >= 2 wr_clk edges
// after a read. Depth is the configured 128 (the core's real usable depth is
// 127; the model never lets that difference matter because it reports full
// conservatively through the same delayed pointers).
module afifo_w64_d128_std (
    input  wire        rst,
    input  wire        wr_clk,
    input  wire        rd_clk,
    input  wire [63:0] din,
    input  wire        wr_en,
    input  wire        rd_en,
    output reg  [63:0] dout,
    output wire        full,
    output wire        empty
);
    reg [63:0] mem [0:127];
    reg [7:0]  wptr, rptr;              // one wrap bit + 7 index bits
    reg [7:0]  wptr_rd1, wptr_rd2;      // write pointer, seen in rd_clk
    reg [7:0]  rptr_wr1, rptr_wr2;      // read pointer, seen in wr_clk

    assign empty = rst | (rptr == wptr_rd2);
    assign full  = rst | (wptr == {~rptr_wr2[7], rptr_wr2[6:0]});

    always @(posedge wr_clk or posedge rst) begin
        if (rst) begin
            wptr <= 8'd0;
            rptr_wr1 <= 8'd0;
            rptr_wr2 <= 8'd0;
        end else begin
            rptr_wr1 <= rptr;
            rptr_wr2 <= rptr_wr1;
            if (wr_en && !full) begin
                mem[wptr[6:0]] <= din;
                wptr <= wptr + 8'd1;
            end
        end
    end

    always @(posedge rd_clk or posedge rst) begin
        if (rst) begin
            rptr <= 8'd0;
            wptr_rd1 <= 8'd0;
            wptr_rd2 <= 8'd0;
            dout <= 64'd0;
        end else begin
            wptr_rd1 <= wptr;
            wptr_rd2 <= wptr_rd1;
            if (rd_en && !empty) begin
                dout <= mem[rptr[6:0]];
                rptr <= rptr + 8'd1;
            end
        end
    end
endmodule
