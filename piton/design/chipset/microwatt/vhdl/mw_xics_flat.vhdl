-- mw_xics_flat — Microwatt XICS (ICP + ICS) with flat, record-free ports (OPN-P2.15)
--
-- The GHDL->Verilog conversion (hw/scripts/convert_microwatt.sh, top
-- mw_xics_flat) elaborates THIS entity, so mw_xics.v instantiates plain ports
-- instead of escaped record fields. xics_icp and xics_ics are Microwatt's own,
-- unmodified (ext/microwatt/xics.vhdl).
--
-- One 8 KB window (contract hw/ip/mw_xics/requirements.yml XICS-MAP-001):
--   wb_adr is the WORD address inside it (byte offset >> 2), 11 bits;
--   wb_adr(10) = byte bit 12 selects the block: 0 = ICP (base + 0x0000),
--   1 = ICS (base + 0x1000). Each block sees the low 10 word-address bits, as
--   on a native Microwatt SoC, where its decoders take adr(5:0) (ICP) and
--   adr(9:0) (ICS).
-- The bus is Microwatt's Wishbone I/O: byte at offset A on lane A mod 4; the
-- blocks byte-swap to their big-endian register view themselves.
-- Neither block stalls; each acks one cycle after cyc & stb [xics.vhdl].
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.common.all;
use work.wishbone_types.all;

entity mw_xics_flat is
    generic (
        NCPUS : natural := 1
        );
    port (
        clk       : in  std_ulogic;
        rst       : in  std_ulogic;

        wb_cyc    : in  std_ulogic;
        wb_stb    : in  std_ulogic;
        wb_we     : in  std_ulogic;
        wb_adr    : in  std_ulogic_vector(10 downto 0);
        wb_sel    : in  std_ulogic_vector(3 downto 0);
        wb_dat_w  : in  std_ulogic_vector(31 downto 0);
        wb_dat_r  : out std_ulogic_vector(31 downto 0);
        wb_ack    : out std_ulogic;

        -- level interrupts; int_level(0) is XICS source 0x10
        int_level : in  std_ulogic_vector(15 downto 0);
        core_irq  : out std_ulogic_vector(NCPUS-1 downto 0)
        );
end entity mw_xics_flat;

architecture rtl of mw_xics_flat is
    signal icp_in, ics_in   : wb_io_master_out;
    signal icp_out, ics_out : wb_io_slave_out;
    signal ics_to_icp       : ics_to_icp_t;
    signal sel_ics          : std_ulogic;
begin
    sel_ics <= wb_adr(10);

    icp_in.adr <= (29 downto 10 => '0') & wb_adr(9 downto 0);
    icp_in.dat <= wb_dat_w;
    icp_in.sel <= wb_sel;
    icp_in.we  <= wb_we;
    icp_in.cyc <= wb_cyc and not sel_ics;
    icp_in.stb <= wb_stb and not sel_ics;

    ics_in.adr <= (29 downto 10 => '0') & wb_adr(9 downto 0);
    ics_in.dat <= wb_dat_w;
    ics_in.sel <= wb_sel;
    ics_in.we  <= wb_we;
    ics_in.cyc <= wb_cyc and sel_ics;
    ics_in.stb <= wb_stb and sel_ics;

    -- one access in flight at a time (the adapter waits for ack), so the
    -- acking block's data is the answer
    wb_ack   <= icp_out.ack or ics_out.ack;
    wb_dat_r <= ics_out.dat when ics_out.ack = '1' else icp_out.dat;

    icp : entity work.xics_icp
        generic map (NCPUS => NCPUS)
        port map (
            clk          => clk,
            rst          => rst,
            wb_in        => icp_in,
            wb_out       => icp_out,
            ics_in       => ics_to_icp,
            core_irq_out => core_irq
            );

    ics : entity work.xics_ics
        generic map (NCPUS => NCPUS, SRC_NUM => 16, PRIO_BITS => 3)
        port map (
            clk          => clk,
            rst          => rst,
            wb_in        => ics_in,
            wb_out       => ics_out,
            int_level_in => int_level,
            icp_out      => ics_to_icp
            );
end architecture rtl;
