// SPDX-FileCopyrightText: 2026 Daniel J. Mazure
// SPDX-License-Identifier: MIT
//
// pmesh_hp_shim - OpenPiton chipset memory master -> Zynq UltraScale+ PS HP port
//
// Slave side: noc_axi4_bridge (512-bit). It only issues single-beat, 64 B,
// 64 B-aligned transfers, so each maps to ONE 4-beat, 16 B INCR
// burst on the 128-bit PS port:
//   write: AW + 1 W beat  ->  AW(len 3, size 4) + W beats 0..3 (bytes 16k..16k+15)
//   read:  AR             ->  AR(len 3, size 4), 4 R beats assembled in address order
// Address: PS = OFFSET + A, only for A + 64 <= WINDOW. Everything else is
// answered here - DECERR outside the carve-out, SLVERR outside the input
// domain - and never reaches m_axi_*: the PS DDR outside the
// carve-out is the host's own memory.
//
// One write and one read are in flight at a time (the bridge allows two of
// each; this halves its peak rate, which the P-Mesh rarely approaches).
// Byte order: slave lane i <-> PS byte OFFSET + A + (i XOR 7),
// i.e. every 64-bit doubleword byte-reversed, strobes with their bytes. The
// bridge puts P-Mesh byte A on lane A XOR 7 (big-endian flits moved whole), so
// this makes the carve-out address-invariant: P-Mesh byte A = PS byte
// OFFSET + A, and PS software loads an image with a plain file copy. A pure
// wire permutation, checked end to end in simulation.
`default_nettype none
module pmesh_hp_shim #(
    parameter [63:0] OFFSET   = 64'h0000_0000_4000_0000,
    parameter [63:0] WINDOW   = 64'h0000_0000_2000_0000,
    parameter        M_ADDR_W = 49
) (
    input  wire          clk,
    input  wire          rst_n,
    // slave: noc_axi4_bridge (512-bit)
    input  wire [5:0]    s_axi_awid,
    input  wire [63:0]   s_axi_awaddr,
    input  wire [7:0]    s_axi_awlen,
    input  wire [2:0]    s_axi_awsize,
    input  wire [1:0]    s_axi_awburst,
    input  wire          s_axi_awvalid,
    output wire          s_axi_awready,
    input  wire [511:0]  s_axi_wdata,
    input  wire [63:0]   s_axi_wstrb,
    input  wire          s_axi_wlast,
    input  wire          s_axi_wvalid,
    output wire          s_axi_wready,
    output reg  [5:0]    s_axi_bid,
    output reg  [1:0]    s_axi_bresp,
    output reg           s_axi_bvalid,
    input  wire          s_axi_bready,
    input  wire [5:0]    s_axi_arid,
    input  wire [63:0]   s_axi_araddr,
    input  wire [7:0]    s_axi_arlen,
    input  wire [2:0]    s_axi_arsize,
    input  wire [1:0]    s_axi_arburst,
    input  wire          s_axi_arvalid,
    output wire          s_axi_arready,
    output reg  [5:0]    s_axi_rid,
    output reg  [511:0]  s_axi_rdata,
    output reg  [1:0]    s_axi_rresp,
    output reg           s_axi_rlast,
    output reg           s_axi_rvalid,
    input  wire          s_axi_rready,
    // master: PS S_AXI_HP0_FPD (128-bit)
    output reg  [5:0]          m_axi_awid,
    output reg  [M_ADDR_W-1:0] m_axi_awaddr,
    output wire [7:0]          m_axi_awlen,
    output wire [2:0]          m_axi_awsize,
    output wire [1:0]          m_axi_awburst,
    output wire                m_axi_awlock,
    output wire [3:0]          m_axi_awcache,
    output wire [2:0]          m_axi_awprot,
    output wire [3:0]          m_axi_awqos,
    output reg                 m_axi_awvalid,
    input  wire                m_axi_awready,
    output wire [127:0]        m_axi_wdata,
    output wire [15:0]         m_axi_wstrb,
    output wire                m_axi_wlast,
    output reg                 m_axi_wvalid,
    input  wire                m_axi_wready,
    input  wire [5:0]          m_axi_bid,
    input  wire [1:0]          m_axi_bresp,
    input  wire                m_axi_bvalid,
    output wire                m_axi_bready,
    output reg  [5:0]          m_axi_arid,
    output reg  [M_ADDR_W-1:0] m_axi_araddr,
    output wire [7:0]          m_axi_arlen,
    output wire [2:0]          m_axi_arsize,
    output wire [1:0]          m_axi_arburst,
    output wire                m_axi_arlock,
    output wire [3:0]          m_axi_arcache,
    output wire [2:0]          m_axi_arprot,
    output wire [3:0]          m_axi_arqos,
    output reg                 m_axi_arvalid,
    input  wire                m_axi_arready,
    input  wire [5:0]          m_axi_rid,
    input  wire [127:0]        m_axi_rdata,
    input  wire [1:0]          m_axi_rresp,
    input  wire                m_axi_rlast,
    input  wire                m_axi_rvalid,
    output wire                m_axi_rready
);
    localparam [1:0] OKAY = 2'b00, SLVERR = 2'b10, DECERR = 2'b11;

    // reverse the bytes of each 64-bit doubleword (and the
    // strobe bits with them). A 16 B beat holds two whole doublewords, so the
    // permutation is beat-local.
    function [127:0] dw_rev128(input [127:0] d);
        integer b;
        begin
            for (b = 0; b < 16; b = b + 1)
                dw_rev128[8 * b +: 8] = d[8 * (b ^ 7) +: 8];
        end
    endfunction
    function [15:0] dw_rev16(input [15:0] m);
        integer b;
        begin
            for (b = 0; b < 16; b = b + 1)
                dw_rev16[b] = m[b ^ 7];
        end
    endfunction

    // Fixed burst shape on the PS port
    assign m_axi_awlen   = 8'd3;
    assign m_axi_awsize  = 3'd4;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;         // bufferable, modifiable; as the bridge
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'd0;
    assign m_axi_arlen   = 8'd3;
    assign m_axi_arsize  = 3'd4;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'd0;

    // A request is served only if it has the bridge's shape and fits the window
    function [1:0] verdict(input [63:0] a, input [7:0] len, input [2:0] size);
        if (len != 8'd0 || size != 3'd6 || a[5:0] != 6'd0)
            verdict = SLVERR;                                   // outside the input domain
        else if (WINDOW < 64'd64 || a > WINDOW - 64'd64)
            verdict = DECERR;                                   // outside the carve-out
        else
            verdict = OKAY;
    endfunction

    // ------------------------------------------------------------------
    // Write path
    // ------------------------------------------------------------------
    localparam [2:0] W_IDLE = 3'd0, W_ADDR = 3'd1, W_DATA = 3'd2, W_B = 3'd3, W_RESP = 3'd4;
    reg [2:0]   wst;
    reg         aw_have, w_have, w_first;
    reg [5:0]   aw_id;
    reg [63:0]  aw_a;
    reg [7:0]   aw_len;
    reg [2:0]   aw_size;
    reg [511:0] w_data;
    reg [63:0]  w_strb;
    reg [1:0]   w_beat;

    assign s_axi_awready = (wst == W_IDLE) & ~aw_have;
    assign s_axi_wready  = (wst == W_IDLE) & ~w_have;
    assign m_axi_wdata   = dw_rev128(w_data[128 * w_beat +: 128]);
    assign m_axi_wstrb   = dw_rev16(w_strb[16 * w_beat +: 16]);
    assign m_axi_wlast   = (w_beat == 2'd3);
    assign m_axi_bready  = (wst == W_B);

    wire [1:0] w_verdict = verdict(aw_a, aw_len, aw_size);

    always @(posedge clk) begin
        if (!rst_n) begin
            wst           <= W_IDLE;
            aw_have       <= 1'b0;
            w_have        <= 1'b0;
            w_first       <= 1'b1;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            w_beat        <= 2'd0;
        end else begin
            case (wst)
                W_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready) begin
                        aw_have <= 1'b1;
                        aw_id   <= s_axi_awid;
                        aw_a    <= s_axi_awaddr;
                        aw_len  <= s_axi_awlen;
                        aw_size <= s_axi_awsize;
                    end
                    // take W beats up to wlast; keep the first (the only one in domain)
                    if (s_axi_wvalid && s_axi_wready) begin
                        if (w_first) begin
                            w_data <= s_axi_wdata;
                            w_strb <= s_axi_wstrb;
                        end
                        w_first <= s_axi_wlast;
                        if (s_axi_wlast)
                            w_have <= 1'b1;
                    end
                    if (aw_have && w_have) begin
                        aw_have <= 1'b0;
                        w_have  <= 1'b0;
                        s_axi_bid <= aw_id;
                        if (w_verdict != OKAY) begin
                            s_axi_bresp  <= w_verdict;
                            s_axi_bvalid <= 1'b1;
                            wst          <= W_RESP;
                        end else begin
                            m_axi_awid    <= aw_id;
                            m_axi_awaddr  <= OFFSET[M_ADDR_W-1:0] + aw_a[M_ADDR_W-1:0];
                            m_axi_awvalid <= 1'b1;
                            wst           <= W_ADDR;
                        end
                    end
                end
                W_ADDR: if (m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b1;
                    w_beat        <= 2'd0;
                    wst           <= W_DATA;
                end
                W_DATA: if (m_axi_wready) begin
                    if (w_beat == 2'd3) begin
                        m_axi_wvalid <= 1'b0;
                        wst          <= W_B;
                    end
                    w_beat <= w_beat + 2'd1;
                end
                W_B: if (m_axi_bvalid) begin
                    s_axi_bresp  <= m_axi_bresp;
                    s_axi_bvalid <= 1'b1;
                    wst          <= W_RESP;
                end
                W_RESP: if (s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                    wst          <= W_IDLE;
                end
                default: wst <= W_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Read path
    // ------------------------------------------------------------------
    localparam [2:0] R_IDLE = 3'd0, R_ADDR = 3'd1, R_DATA = 3'd2, R_RESP = 3'd3, R_ERR = 3'd4;
    reg [2:0]   rst_q;
    reg [1:0]   r_beat;
    reg [7:0]   r_left;                     // error beats still owed (len + 1 in total)

    assign s_axi_arready = (rst_q == R_IDLE);
    assign m_axi_rready  = (rst_q == R_DATA);

    wire [1:0] r_verdict = verdict(s_axi_araddr, s_axi_arlen, s_axi_arsize);

    always @(posedge clk) begin
        if (!rst_n) begin
            rst_q         <= R_IDLE;
            m_axi_arvalid <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rlast   <= 1'b0;
        end else begin
            case (rst_q)
                R_IDLE: if (s_axi_arvalid) begin
                    s_axi_rid <= s_axi_arid;
                    if (r_verdict != OKAY) begin
                        s_axi_rresp  <= r_verdict;
                        s_axi_rdata  <= 512'd0;
                        s_axi_rvalid <= 1'b1;
                        s_axi_rlast  <= (s_axi_arlen == 8'd0);
                        r_left       <= s_axi_arlen;
                        rst_q        <= R_ERR;
                    end else begin
                        m_axi_arid    <= s_axi_arid;
                        m_axi_araddr  <= OFFSET[M_ADDR_W-1:0] + s_axi_araddr[M_ADDR_W-1:0];
                        m_axi_arvalid <= 1'b1;
                        s_axi_rresp   <= OKAY;
                        rst_q         <= R_ADDR;
                    end
                end
                R_ADDR: if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    r_beat        <= 2'd0;
                    rst_q         <= R_DATA;
                end
                R_DATA: if (m_axi_rvalid) begin
                    s_axi_rdata[128 * r_beat +: 128] <= dw_rev128(m_axi_rdata);
                    if (m_axi_rresp > s_axi_rresp)
                        s_axi_rresp <= m_axi_rresp;          // worst of the four
                    r_beat <= r_beat + 2'd1;
                    if (r_beat == 2'd3) begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rlast  <= 1'b1;
                        rst_q        <= R_RESP;
                    end
                end
                R_RESP: if (s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                    s_axi_rlast  <= 1'b0;
                    rst_q        <= R_IDLE;
                end
                R_ERR: if (s_axi_rready) begin            // len + 1 error beats (AXI A3.4.4)
                    if (r_left == 8'd0) begin
                        s_axi_rvalid <= 1'b0;
                        s_axi_rlast  <= 1'b0;
                        rst_q        <= R_IDLE;
                    end else begin
                        r_left      <= r_left - 8'd1;
                        s_axi_rlast <= (r_left == 8'd1);
                    end
                end
                default: rst_q <= R_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
