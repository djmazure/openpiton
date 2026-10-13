-- SPDX-FileCopyrightText: 2026 Daniel J. Mazure
-- SPDX-License-Identifier: MIT
--
-- mw_syscon_flat — Microwatt syscon with flat, record-free ports
--
-- The GHDL->Verilog conversion (piton/tools/bin/convert_microwatt, top
-- mw_syscon_flat) elaborates THIS entity, so mw_syscon.v instantiates plain
-- ports instead of escaped record fields. syscon is Microwatt's own,
-- unmodified (design/chip/tile/microwatt/syscon.vhdl).
--
-- One 256-byte window (the SYSCON_LENGTH Linux maps): wb_adr is the WORD address
-- inside it (byte offset >> 2), 6 bits. syscon decodes adr(6 downto 1) as the
-- 64-bit register and adr(0) as its high/low word, so adr(6) is 0 here: the
-- registers beyond 0x100 (none that Linux uses) are not reachable.
-- The bus is Microwatt's Wishbone I/O: byte at offset A on lane A mod 4, the
-- registers little-endian, as Linux's readl/writel expect.
-- syscon never stalls and acks one cycle after cyc & stb [syscon.vhdl].
--
-- The SoC facts below are what INFO/BRAMINFO/DRAMINFO/CLKINFO/UART0_INFO read
-- back. Linux's Microwatt platform code does not read them (only smp.c's
-- CPU_CTRL); they describe this system as closely as the fields allow.
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.wishbone_types.all;

entity mw_syscon_flat is
    generic (
        NCPUS    : positive := 1;
        CLK_FREQ : positive := 50000000
        );
    port (
        clk        : in  std_ulogic;
        rst        : in  std_ulogic;

        wb_cyc     : in  std_ulogic;
        wb_stb     : in  std_ulogic;
        wb_we      : in  std_ulogic;
        wb_adr     : in  std_ulogic_vector(5 downto 0);
        wb_sel     : in  std_ulogic_vector(3 downto 0);
        wb_dat_w   : in  std_ulogic_vector(31 downto 0);
        wb_dat_r   : out std_ulogic_vector(31 downto 0);
        wb_ack     : out std_ulogic;

        -- syscon's core_reset (= not CPU_CTRL enable bits unless CTRL.CORE_RESET)
        core_reset : out std_ulogic_vector(NCPUS-1 downto 0)
        );
end entity mw_syscon_flat;

architecture rtl of mw_syscon_flat is
    signal wb_in  : wb_io_master_out;
    signal wb_out : wb_io_slave_out;
begin
    wb_in.adr <= (29 downto 6 => '0') & wb_adr;
    wb_in.dat <= wb_dat_w;
    wb_in.sel <= wb_sel;
    wb_in.we  <= wb_we;
    wb_in.cyc <= wb_cyc;
    wb_in.stb <= wb_stb;

    wb_ack   <= wb_out.ack;
    wb_dat_r <= wb_out.dat;

    syscon0 : entity work.syscon
        generic map (
            NCPUS            => NCPUS,
            CLK_FREQ         => CLK_FREQ,
            HAS_UART         => true,
            HAS_DRAM         => true,
            BRAM_SIZE        => 0,
            DRAM_SIZE        => 536870912,     -- the 512 MB PS-DDR carve-out
            DRAM_INIT_SIZE   => 0,
            HAS_SPI_FLASH    => false,
            SPI_FLASH_OFFSET => 0,
            HAS_LITEETH      => false,
            HAS_SD_CARD      => false,
            HAS_SD_CARD2     => false,
            UART0_IS_16550   => true,
            HAS_UART1        => false
            )
        port map (
            clk          => clk,
            rst          => rst,
            wishbone_in  => wb_in,
            wishbone_out => wb_out,
            dram_at_0    => open,             -- drives nothing
            core_reset   => core_reset,
            soc_reset    => open,
            alt_reset    => open,
            tb_rdp       => open,
            tb_frz       => open
            );
end architecture rtl;
