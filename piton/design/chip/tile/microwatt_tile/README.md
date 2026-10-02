# Microwatt tile

A Microwatt (OpenPOWER, Power ISA 3.0) core as an OpenPiton tile core:
`TILE_TYPE == MICROWATT_TILE`, selected with `PITON_MICROWATT`.

| Path | What |
|---|---|
| `../microwatt` | Microwatt, as a git submodule (`git submodule update --init piton/design/chip/tile/microwatt`) |
| `rtl/mw_piton_core.v` | What `tile.v` instantiates: waits for the IOB wake-up, maps the 32-bit core bus onto the 40-bit physical address space |
| `rtl/mw_tile.v` | The converted core plus the bridge |
| `rtl/tri_wb_bridge.v` | Microwatt's pipelined Wishbone (instruction and data) to the L1.5 TRI, with L1D and L1I coherence |
| `vhdl/mw_core_flat.vhdl` | Flat-port wrapper around Microwatt's `core`, the elaboration top for conversion |
| `rtl/Flist.microwatt_tile`, `rtl/Flist.microwatt_tile_nofpu` | Sources, with and without the FPU |

The chipset devices a Microwatt Linux needs live in `design/chipset/microwatt`:
the XICS interrupt controller (`PITON_MW_XICS`) and the syscon `CPU_CTRL` that
starts secondary CPUs (`PITON_MW_SYSCON`). The KV260 board pieces (the PS-DDR
memory shim and an open 16550) live in `design/xilinx/kv260`.

## Netlists for simulation

Simulators that cannot read VHDL need Verilog netlists of Microwatt. With
`DV_ROOT` set, and yosys with the ghdl plugin on `PATH`:

    convert_microwatt            # all six netlists, into the gen/ directories

Vivado reads VHDL, so an FPGA build can list the wrapper and Microwatt's
sources instead.

## Configuration

Size the L1D to match the core's data cache (for example `CONFIG_L1D_SIZE=1024`,
`CONFIG_L1D_ASSOCIATIVITY=1` for 64 x 16 B direct-mapped): the L1.5 shadows it.
