-- mw_core_flat — Microwatt core with flat, record-free ports (OPN-P2.3)
--
-- The GHDL->Verilog conversion (hw/scripts/convert_microwatt.sh) elaborates
-- THIS entity as its top, so the generated Verilog module has plain port names
-- that match tri_wb_bridge one-to-one instead of escaped record fields.
--
-- Configuration is fixed here to what P-Mesh requires
-- (docs/microarchitecture.md sec 3.2, sec 4):
--   * direct-mapped L1D, 16 B lines, 64 lines  -> tri_wb_bridge L1D_INDEX_BITS = 6
--   * 32 B I-cache lines (P-Mesh L1I line)
--   * EXT_ATOMICS: lwarx/stcx. verdict comes from the L1.5 (wbd_reserve/wbd_sc_fail)
--   * DCBZ_ALLOCATE = false: the dcache allocates a line ONLY through a cacheable
--     read fill - the one allocation tri_wb_bridge's shadow tags and the L1.5
--     way-map can see. A dcbz-allocated line would never be invalidated
--     (found by test_mw_tile: head.S zeroes .bss with dcbz).
--
-- wbd_nc is the dcache's own storage attribute for each data beat (core
-- wb_data_nc = dcache ext_nc, OPN-P3.3): real-mode I/O space (0xC...),
-- cache-inhibited instructions (ldcix/stbcix...) at ANY address, and no-cache
-- pages in virtual mode. It is 0 for an EXT_ATOMICS lwarx to memory, which is
-- non-allocating but not cache-inhibited.
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.common.all;
use work.wishbone_types.all;
use work.utils.all;

entity mw_core_flat is
    generic (
        DCACHE_NUM_LINES : natural := 64;
        -- false drops the FPU (OPN-P2.9 fit trim; FP ops then need the
        -- kernel's MATH_EMULATION, which relies on them trapping)
        HAS_FPU          : boolean := true
        );
    port (
        clk          : in  std_ulogic;
        rst          : in  std_ulogic;
        -- Timebase reset (OPN-P2.15). The SYSTEM reset, not rst: rst also
        -- holds a sleeping tile (core_hold), and a timebase that restarts at
        -- each tile's wake differs across cores. Every tile's timebase then
        -- counts the same clock from the same reset release.
        tb_rst       : in  std_ulogic;
        -- PIR of this core (OPN-P2.15): one netlist serves every tile, so the
        -- CPU index is a port, driven with the tile's flat id.
        cpu_index    : in  std_ulogic_vector(7 downto 0);

        -- instruction master (read-only)
        wbi_cyc      : out std_ulogic;
        wbi_stb      : out std_ulogic;
        wbi_adr      : out std_ulogic_vector(28 downto 0);
        wbi_sel      : out std_ulogic_vector(7 downto 0);
        wbi_dat_r    : in  std_ulogic_vector(63 downto 0);
        wbi_ack      : in  std_ulogic;
        wbi_stall    : in  std_ulogic;

        -- data master + EXT_ATOMICS sideband
        wbd_cyc      : out std_ulogic;
        wbd_stb      : out std_ulogic;
        wbd_we       : out std_ulogic;
        wbd_adr      : out std_ulogic_vector(28 downto 0);
        wbd_sel      : out std_ulogic_vector(7 downto 0);
        wbd_dat_w    : out std_ulogic_vector(63 downto 0);
        wbd_nc       : out std_ulogic;
        wbd_reserve  : out std_ulogic;
        wbd_dat_r    : in  std_ulogic_vector(63 downto 0);
        wbd_ack      : in  std_ulogic;
        wbd_stall    : in  std_ulogic;
        wbd_sc_fail  : in  std_ulogic;

        -- snoop (write cycles invalidate matching dcache lines)
        snp_cyc      : in  std_ulogic;
        snp_stb      : in  std_ulogic;
        snp_we       : in  std_ulogic;
        snp_adr      : in  std_ulogic_vector(28 downto 0);

        -- I-cache index invalidation (OPN-P2.13, TWB-COH-003): invalidate
        -- every way of the icache index that byte address icinv_adr names.
        -- The L1.5 names only address bits [15:4] (inval_address_15_4), so
        -- the icache index must lie inside them (asserted below).
        icinv_stb    : in  std_ulogic;
        icinv_adr    : in  std_ulogic_vector(15 downto 4);

        -- external interrupt (level, this core's XICS ICP output; OPN-P2.15).
        -- Already synchronised to clk by the caller.
        ext_irq      : in  std_ulogic;

        terminated   : out std_ulogic
        );
end entity mw_core_flat;

architecture rtl of mw_core_flat is
    constant ICACHE_LINE_SIZE : natural := 32;
    constant ICACHE_NUM_LINES : natural := 64;
    signal icinv_addr : real_addr_t;
    signal insn_out  : wishbone_master_out;
    signal insn_in   : wishbone_slave_out;
    signal data_out  : wishbone_master_out;
    signal data_in   : wishbone_slave_out;
    signal snoop     : wishbone_master_out;
    signal tb_ctrl   : timebase_ctrl;
    signal term      : std_logic;
begin
    core0: entity work.core
        generic map (
            SIM => false,
            HAS_FPU => HAS_FPU,
            HAS_BTC => true,
            ICACHE_LINE_SIZE => ICACHE_LINE_SIZE,
            ICACHE_NUM_LINES => ICACHE_NUM_LINES,
            DCACHE_LINE_SIZE => 16,
            DCACHE_NUM_LINES => DCACHE_NUM_LINES,
            DCACHE_NUM_WAYS => 1,
            EXT_ATOMICS => true,
            DCBZ_ALLOCATE => false
            )
        port map (
            clk => clk,
            rst => rst,
            alt_reset => '0',
            tb_ctrl => tb_ctrl,
            pir_in => cpu_index,
            wishbone_insn_in => insn_in,
            wishbone_insn_out => insn_out,
            wishbone_data_in => data_in,
            wishbone_data_out => data_out,
            wb_snoop_in => snoop,
            icache_inval_idx => icinv_stb,
            icache_inval_addr => icinv_addr,
            wb_data_reserve => wbd_reserve,
            wb_data_sc_fail => wbd_sc_fail,
            wb_data_nc => wbd_nc,
            dmi_addr => (others => '0'),
            dmi_din => (others => '0'),
            dmi_dout => open,
            dmi_req => '0',
            dmi_wr => '0',
            dmi_ack => open,
            ext_irq => ext_irq,
            msg_in => '0',
            msg_out => open,
            run_out => open,
            terminated_out => term
            );

    assert log2(ICACHE_LINE_SIZE) + log2(ICACHE_NUM_LINES) <= 16
        report "icache index not inside addr[15:4]: the L1.5's I-invalidation cannot name it"
        severity failure;
    icinv_addr(REAL_ADDR_BITS - 1 downto 16) <= (others => '0');
    icinv_addr(15 downto 4) <= icinv_adr;
    icinv_addr(3 downto 0) <= (others => '0');

    tb_ctrl.reset <= tb_rst;
    tb_ctrl.rd_prot <= '0';
    tb_ctrl.freeze <= '0';

    wbi_cyc <= insn_out.cyc;
    wbi_stb <= insn_out.stb;
    wbi_adr <= insn_out.adr;
    wbi_sel <= insn_out.sel;
    insn_in.dat <= wbi_dat_r;
    insn_in.ack <= wbi_ack;
    insn_in.stall <= wbi_stall;

    wbd_cyc <= data_out.cyc;
    wbd_stb <= data_out.stb;
    wbd_we <= data_out.we;
    wbd_adr <= data_out.adr;
    wbd_sel <= data_out.sel;
    wbd_dat_w <= data_out.dat;
    data_in.dat <= wbd_dat_r;
    data_in.ack <= wbd_ack;
    data_in.stall <= wbd_stall;

    snoop.cyc <= snp_cyc;
    snoop.stb <= snp_stb;
    snoop.we <= snp_we;
    snoop.adr <= snp_adr;
    snoop.dat <= (others => '0');
    snoop.sel <= (others => '0');

    terminated <= term;
end architecture rtl;
