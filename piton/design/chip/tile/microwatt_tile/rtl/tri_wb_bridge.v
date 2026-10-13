// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// tri_wb_bridge — Microwatt (pipelined Wishbone B4) <-> OpenPiton TRI (L1.5)
//
// Connects an unmodified-datapath Microwatt core to OpenPiton/BYOC's
// Transaction-Response Interface. Contract: requirements.yml (TWB-*).
// Design: tich-super docs/strategy/openpiton-microwatt-RTL-P3.1747.md sec 3.3.
//
// Architecture (deliberately simple; correctness first):
//   * ONE TRI transaction in flight. A Wishbone transfer is accepted only when
//     the bridge is idle; it becomes one TRI request (reads, LR) or one TRI
//     request per naturally-aligned chunk (writes, TWB-MAP-001), and the
//     Wishbone ack is returned once the last TRI response has arrived. Program
//     order is therefore preserved by construction (TWB-ORD-001).
//   * Round-robin arbitration between the instruction and data masters
//     (TWB-ARB-001).
//   * Coherence (TWB-COH-001/002): a direct-mapped shadow tag per L1D index,
//     written on every cacheable data fill. Any L1.5 message carrying a dcache
//     invalidation (EVICT_REQ or ST_ACK with inval_dcache_inval/all_way, or a
//     LOAD_RET with cross_invalidate) clears the shadow entry and emits ONE
//     snoop write on snp_* with wbd_stall held high in that cycle, so
//     Microwatt's no-handshake snoop port cannot mask it.
//   * I-side coherence (TWB-COH-003, OPN-P2.13): any L1.5 message carrying
//     inval_icache_inval or inval_icache_all_way is forwarded, in the cycle it
//     is consumed, as an icache INDEX invalidation (icinv_stb/icinv_adr = the
//     message's address bits [15:4]); the core clears every way of that index.
//     Power ISA icbi acts on all processors and Linux relies on it, but
//     Microwatt's icbi reaches only its own icache: without this, code another
//     tile rewrites runs stale here (test_piton_mw_icoh).
//   * Every L1.5 message is consumed (transducer_l15_req_ack) (TWB-HS-002).
//
// Byte order: Wishbone lane k = byte at dword*8+k (little-endian lanes);
// TRI words are big-endian (pico_decoder.v:142-144). Store data is REPLICATED
// across the 64-bit word for sub-doubleword sizes (as pico_decoder.v:134 does),
// so it is correct whether the L1.5 selects bytes by address or by position.
//
// Scope assumptions carried by the contract, not re-checked here:
//   * direct-mapped Microwatt L1D with 16 B lines (TWB-IN-003);
//   * lwarx reads (wbd_reserve) do NOT allocate in the Microwatt dcache, since
//     the bridge does not shadow-tag them. Guaranteed by the Microwatt dcache's
//     EXT_ATOMICS mode (OPN-P3.1) and pinned by ext/microwatt
//     dcache_ext_atomics_tb.vhdl check EXT2 (mutation "lwarx allocates" is killed).

`default_nettype none

module tri_wb_bridge #(
    parameter L1D_INDEX_BITS = 6          // Microwatt dcache lines = 2**L1D_INDEX_BITS (16 B each)
) (
    input  wire        clk,
    input  wire        rst_n,

    // Microwatt instruction master (pipelined Wishbone B4, read-only)
    input  wire        wbi_cyc,
    input  wire        wbi_stb,
    input  wire [28:0] wbi_adr,
    input  wire [7:0]  wbi_sel,
    output reg  [63:0] wbi_dat_r,
    output reg         wbi_ack,
    output wire        wbi_stall,

    // Microwatt data master (pipelined Wishbone B4) + atomic sideband
    input  wire        wbd_cyc,
    input  wire        wbd_stb,
    input  wire        wbd_we,
    input  wire [28:0] wbd_adr,
    input  wire [7:0]  wbd_sel,
    input  wire [63:0] wbd_dat_w,
    input  wire        wbd_nc,
    input  wire        wbd_reserve,
    output reg  [63:0] wbd_dat_r,
    output reg         wbd_ack,
    output wire        wbd_stall,
    output reg         wbd_sc_fail,

    // Snoop into the Microwatt dcache (core.wb_snoop_in)
    output wire        snp_cyc,
    output wire        snp_stb,
    output wire        snp_we,
    output wire [28:0] snp_adr,

    // I-cache index invalidation into the Microwatt core (TWB-COH-003)
    output wire        icinv_stb,
    output wire [15:4] icinv_adr,

    // TRI: bridge -> L1.5
    output reg         transducer_l15_val,
    output reg  [4:0]  transducer_l15_rqtype,
    output reg  [3:0]  transducer_l15_amo_op,
    output reg         transducer_l15_nc,
    output reg  [2:0]  transducer_l15_size,
    output wire [0:0]  transducer_l15_threadid,
    output wire        transducer_l15_prefetch,
    output wire        transducer_l15_invalidate_cacheline,
    output wire        transducer_l15_blockstore,
    output wire        transducer_l15_blockinitstore,
    output wire [1:0]  transducer_l15_l1rplway,
    output reg  [39:0] transducer_l15_address,
    output reg  [63:0] transducer_l15_data,
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
    input  wire        l15_transducer_blockinitstore
);

    // ------------------------------------------------------------------
    // Encodings (ext/openpiton piton/design/include)
    // ------------------------------------------------------------------
    localparam [4:0] LOAD_RQ  = 5'b00000;   // iop.h
    localparam [4:0] IMISS_RQ = 5'b10000;   // iop.h
    localparam [4:0] STORE_RQ = 5'b00001;   // iop.h
    localparam [4:0] AMO_RQ   = 5'b00110;   // l15.h.pyv PCX_REQTYPE_AMO = SWAP_RQ

    localparam [3:0] LOAD_RET   = 4'b0000;  // iop.h
    localparam [3:0] IFILL_RET  = 4'b0001;  // iop.h
    localparam [3:0] ST_ACK     = 4'b0100;  // iop.h
    localparam [3:0] ATOMIC_RES = 4'b1110;  // l15.h.pyv CPX_RESTYPE_ATOMIC_RES

    localparam [3:0] AMO_NONE = 4'b0000;
    localparam [3:0] AMO_LR   = 4'b0001;    // l15.h.pyv:318
    localparam [3:0] AMO_SC   = 4'b0010;    // l15.h.pyv:319

    localparam [2:0] SZ_1B  = 3'b001;       // define.h.pyv MSG_DATA_SIZE_*
    localparam [2:0] SZ_2B  = 3'b010;
    localparam [2:0] SZ_4B  = 3'b011;
    localparam [2:0] SZ_8B  = 3'b100;
    localparam [2:0] SZ_32B = 3'b110;

    // ------------------------------------------------------------------
    // Constant tie-offs (TWB-NEG-005, TWB-ORD-001 single memory thread)
    // ------------------------------------------------------------------
    assign transducer_l15_threadid             = 1'b0;
    assign transducer_l15_prefetch             = 1'b0;
    assign transducer_l15_invalidate_cacheline = 1'b0;
    assign transducer_l15_blockstore           = 1'b0;
    assign transducer_l15_blockinitstore       = 1'b0;
    assign transducer_l15_l1rplway             = 2'b00;   // direct-mapped L1D (TWB-IN-003)
    assign transducer_l15_data_next_entry      = 64'd0;
    assign transducer_l15_csm_data             = 33'd0;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    function [63:0] bswap64(input [63:0] x);
        bswap64 = {x[7:0], x[15:8], x[23:16], x[31:24], x[39:32], x[47:40], x[55:48], x[63:56]};
    endfunction

    // First set lane at or above 0 in a byte-select mask.
    function [2:0] first_lane(input [7:0] m);
        integer k;
        begin
            first_lane = 3'd0;
            for (k = 7; k >= 0; k = k - 1)
                if (m[k]) first_lane = k[2:0];
        end
    endfunction

    // Largest naturally-aligned chunk (bytes) starting at lane p fully inside m.
    function [3:0] chunk_bytes(input [7:0] m, input [2:0] p);
        begin
            if (p == 3'd0 && m == 8'hFF)                               chunk_bytes = 4'd8;
            else if (p[1:0] == 2'd0 && ((m >> p) & 8'h0F) == 8'h0F)    chunk_bytes = 4'd4;
            else if (p[0] == 1'b0 && ((m >> p) & 8'h03) == 8'h03)      chunk_bytes = 4'd2;
            else                                                       chunk_bytes = 4'd1;
        end
    endfunction

    // Byte-lane mask covered by a chunk.
    function [7:0] chunk_mask(input [2:0] p, input [3:0] n);
        chunk_mask = ((8'h01 << n) - 8'h01) << p;
    endfunction

    // byte-lane mask -> bit mask (lane k = bits 8k+7..8k)
    function [63:0] lane_bits(input [7:0] m);
        integer b;
        begin
            for (b = 0; b < 8; b = b + 1)
                lane_bits[8*b +: 8] = {8{m[b]}};
        end
    endfunction

    function [2:0] size_enc(input [3:0] n);
        case (n)
            4'd8:    size_enc = SZ_8B;
            4'd4:    size_enc = SZ_4B;
            4'd2:    size_enc = SZ_2B;
            default: size_enc = SZ_1B;
        endcase
    endfunction

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0, S_PREP = 3'd1, S_REQ = 3'd2, S_RSP = 3'd3, S_DONE = 3'd4;
    reg [2:0]  state;

    reg        cur_i;            // 1 = instruction master, 0 = data master
    reg        cur_we, cur_nc, cur_res;
    reg [28:0] cur_adr;
    reg [7:0]  cur_sel;
    reg [63:0] cur_be;           // bswap64(wbd_dat_w): big-endian view of the store data
    reg [7:0]  rem_sel;          // write lanes still to be stored
    reg [7:0]  chunk_m;          // lanes of the chunk in flight
    reg [2:0]  chunk_p;          // lane of the chunk in flight
    reg [3:0]  expect_rt;        // return type the in-flight request expects
    reg        rr_pri_i;         // round-robin: 1 = instruction master has priority

    reg [63:0] rsp_dat;
    reg        rsp_scfail;

    // shadow tags: line address bits [31:4] per L1D index
    localparam NIDX = (1 << L1D_INDEX_BITS);
    reg [27:0] sh_line  [0:NIDX-1];
    reg [NIDX-1:0] sh_valid;

    reg        snp_pend;
    reg [27:0] snp_line;

    // ------------------------------------------------------------------
    // Arbitration + Wishbone acceptance (combinational stall)
    // ------------------------------------------------------------------
    wire req_i = wbi_cyc & wbi_stb;
    wire req_d = wbd_cyc & wbd_stb;
    wire can_accept = (state == S_IDLE) & ~snp_pend;
    wire grant_i = can_accept & req_i & (rr_pri_i | ~req_d);
    wire grant_d = can_accept & req_d & ~grant_i;

    assign wbi_stall = ~grant_i;
    // TWB-HS-003: wbd_stall says whether a data request presented NOW would be
    // accepted, independently of wbd_stb - Microwatt's DO_STCX waits for
    // stall=0 before raising stb, so a stall that falls only in answer to stb
    // deadlocks every stcx. With stb high this is exactly ~grant_d.
    // TWB-COH-002: wbd_stall is forced high in any cycle a snoop is presented.
    wire i_wins = req_i & (rr_pri_i | ~req_d);
    assign wbd_stall = ~can_accept | i_wins | snp_pend;

    // ------------------------------------------------------------------
    // Snoop output (one cycle per invalidation)
    // ------------------------------------------------------------------
    assign snp_cyc = snp_pend;
    assign snp_stb = snp_pend;
    assign snp_we  = snp_pend;
    assign snp_adr = {snp_line, 1'b0};      // doubleword address of the line

    // ------------------------------------------------------------------
    // L1.5 message intake. Every message is consumed in the cycle it is
    // presented (TWB-HS-002, NEG-009): a refused message stalls the L1.5 at
    // S3, which mis-counts NoC1 credits. A snoop is presented from registers
    // (snp_pend/snp_line), so a message taken while one is on snp_* only
    // loads the NEXT snoop and cannot overwrite it (TWB-NEG-004).
    // ------------------------------------------------------------------
    wire take = l15_transducer_val;
    assign transducer_l15_req_ack = take;

    wire msg_inval_d = l15_transducer_inval_dcache_inval | l15_transducer_inval_dcache_all_way;
    // I-side: the L1.5 names the index (address bits [15:4]), not the line, and
    // a single-way inval_icache_inval is widened to all ways (conservative).
    assign icinv_stb = take & (l15_transducer_inval_icache_inval | l15_transducer_inval_icache_all_way);
    assign icinv_adr = l15_transducer_inval_address_15_4;
    wire [L1D_INDEX_BITS-1:0] inval_idx = l15_transducer_inval_address_15_4[4 +: L1D_INDEX_BITS];
    wire cur_is_fill = ~cur_i & ~cur_we & ~cur_nc & ~cur_res;
    wire [L1D_INDEX_BITS-1:0] cur_idx = cur_adr[1 +: L1D_INDEX_BITS];   // dword adr -> line index
    wire msg_cross = (l15_transducer_returntype == LOAD_RET) & l15_transducer_cross_invalidate;
    // The response may come in the ack cycle itself (INT_VEC_DIS: acked in S3
    // with its ST_ACK) or later, never before (TWB-HS-001).
    wire msg_is_rsp = ((state == S_RSP) | ((state == S_REQ) & l15_transducer_ack))
                    & (l15_transducer_returntype == expect_rt);

    // Data word for the addressed doubleword (TWB-MAP-002)
    reg [63:0] sel_word;
    always @(*) begin
        if (cur_i) begin
            case (cur_adr[1:0])
                2'd0: sel_word = l15_transducer_data_0;
                2'd1: sel_word = l15_transducer_data_1;
                2'd2: sel_word = l15_transducer_data_2;
                default: sel_word = l15_transducer_data_3;
            endcase
        end else begin
            sel_word = cur_adr[0] ? l15_transducer_data_1 : l15_transducer_data_0;
        end
    end

    // ------------------------------------------------------------------
    // Next TRI chunk (combinational). Writes are split into naturally-aligned
    // chunks covering exactly the selected lanes (TWB-MAP-001, NEG-001/002);
    // sub-doubleword data is REPLICATED across the word (pico_decoder.v:134).
    // be_sh brings lane p to the top byte, so no part-select can go out of range.
    // ------------------------------------------------------------------
    // Writes and cache-inhibited reads are chunked over exactly the selected
    // lanes (TWB-MAP-001, TWB-NEG-010: a device read must not touch its
    // neighbours - OPN-T1.2); cacheable reads fetch whole lines.
    wire        cur_chunked = cur_we | (cur_nc & ~cur_res);
    wire [7:0]  cur_mask = cur_chunked ? rem_sel : cur_sel;
    wire [2:0]  nxt_p    = first_lane(cur_mask);
    wire [3:0]  nxt_n    = chunk_bytes(cur_mask, nxt_p);
    wire [63:0] be_sh    = cur_be << (8 * nxt_p);
    reg  [63:0] nxt_data;
    always @(*) begin
        case (nxt_n)
            4'd8:    nxt_data = cur_be;
            4'd4:    nxt_data = {2{be_sh[63:32]}};
            4'd2:    nxt_data = {4{be_sh[63:48]}};
            default: nxt_data = {8{be_sh[63:56]}};
        endcase
    end

    // ------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------
    integer j;
    always @(posedge clk) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            rr_pri_i           <= 1'b0;
            transducer_l15_val <= 1'b0;
            wbi_ack            <= 1'b0;
            wbd_ack            <= 1'b0;
            wbd_sc_fail        <= 1'b0;
            snp_pend           <= 1'b0;
            sh_valid           <= {NIDX{1'b0}};
            wbi_dat_r          <= 64'd0;
            wbd_dat_r          <= 64'd0;
        end else begin
            wbi_ack  <= 1'b0;
            wbd_ack  <= 1'b0;
            snp_pend <= 1'b0;

            // ---------------- message intake ----------------
            if (take) begin
                if (msg_inval_d && sh_valid[inval_idx]) begin
                    snp_pend            <= 1'b1;
                    snp_line            <= sh_line[inval_idx];
                    sh_valid[inval_idx] <= 1'b0;
                end
                if (msg_is_rsp) begin
                    // chunked nc read: the first response supplies the whole word, later
                    // ones only their chunk's lanes - every byte is from a TRI response
                    // for this doubleword, never a previous beat (TWB-NEG-006)
                    if (expect_rt == LOAD_RET && cur_chunked && rem_sel != cur_sel)
                        rsp_dat <= (rsp_dat & ~lane_bits(chunk_m)) | (bswap64(sel_word) & lane_bits(chunk_m));
                    else
                        rsp_dat <= bswap64(sel_word);
                    // SC outcome is DATA: first byte of the addressed word = ~lrsc_flag
                    // (l15_pipeline.v.pyv:4076-4082) -> its LSB is bit 56-8*p.
                    rsp_scfail <= sel_word[56 - 8*chunk_p];
                    if (expect_rt == ST_ACK || (expect_rt == LOAD_RET && cur_chunked)) begin
                        rem_sel <= rem_sel & ~chunk_m;
                        state   <= ((rem_sel & ~chunk_m) == 8'd0) ? S_DONE : S_PREP;
                    end else begin
                        state <= S_DONE;
                    end
                    if (expect_rt == LOAD_RET && cur_is_fill) begin
                        // cross_invalidate on this LOAD_RET names the line being replaced
                        if (msg_cross && sh_valid[cur_idx] && !(msg_inval_d && sh_valid[inval_idx])) begin
                            snp_pend <= 1'b1;
                            snp_line <= sh_line[cur_idx];
                        end
                        sh_line[cur_idx]  <= cur_adr[28:1];
                        sh_valid[cur_idx] <= 1'b1;
                    end
                end
            end

            // ---------------- request side ----------------
            case (state)
                S_IDLE: begin
                    if (grant_i || grant_d) begin
                        cur_i    <= grant_i;
                        cur_we   <= grant_i ? 1'b0 : wbd_we;
                        cur_nc   <= grant_i ? 1'b0 : wbd_nc;
                        cur_res  <= grant_i ? 1'b0 : wbd_reserve;
                        cur_adr  <= grant_i ? wbi_adr : wbd_adr;
                        cur_sel  <= grant_i ? wbi_sel : wbd_sel;
                        rem_sel  <= grant_i ? 8'd0 : wbd_sel;
                        cur_be   <= bswap64(wbd_dat_w);
                        rr_pri_i <= ~grant_i;
                        state    <= S_PREP;
                    end
                end

                S_PREP: begin
                    chunk_p <= nxt_p;
                    chunk_m <= chunk_mask(nxt_p, nxt_n);
                    transducer_l15_amo_op <= AMO_NONE;
                    transducer_l15_nc     <= cur_nc;
                    transducer_l15_data   <= 64'd0;
                    if (cur_i) begin
                        // I-fill: 32 B block, naturally aligned (TWB-MAP-002, NEG-002)
                        transducer_l15_rqtype  <= IMISS_RQ;
                        transducer_l15_size    <= SZ_32B;
                        transducer_l15_address <= {8'd0, cur_adr[28:2], 5'd0};
                        expect_rt              <= IFILL_RET;
                    end else if (cur_we) begin
                        transducer_l15_rqtype  <= cur_res ? AMO_RQ : STORE_RQ;
                        transducer_l15_amo_op  <= cur_res ? AMO_SC : AMO_NONE;
                        transducer_l15_size    <= size_enc(nxt_n);
                        transducer_l15_address <= {8'd0, cur_adr, nxt_p};
                        transducer_l15_data    <= nxt_data;
                        expect_rt              <= cur_res ? ATOMIC_RES : ST_ACK;
                    end else if (cur_res) begin
                        transducer_l15_rqtype  <= AMO_RQ;
                        transducer_l15_amo_op  <= AMO_LR;
                        transducer_l15_size    <= size_enc(nxt_n);
                        transducer_l15_address <= {8'd0, cur_adr, nxt_p};
                        expect_rt              <= ATOMIC_RES;
                    end else if (cur_nc) begin
                        // cache-inhibited: this chunk's bytes only (TWB-NEG-010)
                        transducer_l15_rqtype  <= LOAD_RQ;
                        transducer_l15_size    <= size_enc(nxt_n);
                        transducer_l15_address <= {8'd0, cur_adr, nxt_p};
                        expect_rt              <= LOAD_RET;
                    end else begin
                        transducer_l15_rqtype  <= LOAD_RQ;
                        transducer_l15_size    <= SZ_8B;
                        transducer_l15_address <= {8'd0, cur_adr, 3'd0};
                        expect_rt              <= LOAD_RET;
                    end
                    transducer_l15_val <= 1'b1;
                    state              <= S_REQ;
                end

                // The L1.5 takes the header with header_ack (S1) and completes
                // the request with ack (S1..S3). val drops after the header is
                // taken - held, it would be taken again as a second request
                // (TWB-NEG-008) - while every field stays put until ack, since
                // the L1.5 reads the data up to S3 (TWB-HS-001, NEG-003).
                S_REQ: begin
                    if (l15_transducer_header_ack)
                        transducer_l15_val <= 1'b0;
                    if (l15_transducer_ack) begin
                        transducer_l15_val <= 1'b0;
                        if (!(take && msg_is_rsp))     // else intake already moved on
                            state          <= S_RSP;
                    end
                end

                S_RSP: ;   // advanced by message intake above

                S_DONE: begin
                    if (cur_i) begin
                        wbi_ack   <= 1'b1;
                        wbi_dat_r <= rsp_dat;
                    end else begin
                        wbd_ack     <= 1'b1;
                        wbd_dat_r   <= rsp_dat;
                        wbd_sc_fail <= cur_res & cur_we & rsp_scfail;
                    end
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
