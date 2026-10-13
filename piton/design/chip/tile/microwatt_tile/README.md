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

## Prerequisites

| Tool | Used for |
|---|---|
| `/bin/csh` (`csh` or `tcsh` package) | `bw_cpp`, which sims uses to preprocess its config (as for every OpenPiton core) |
| yosys with the ghdl plugin | `convert_microwatt` (Microwatt is VHDL) |
| Verilator 5 (tested: 5.036) | the simulation model |
| A powerpc64le GCC and binutils (default prefix `powerpc64le-linux-gnu-`, e.g. Ubuntu `gcc-powerpc64le-linux-gnu`) | building diags (`power_cc`, `power_img`) |

## Running a diag in simulation

    git submodule update --init piton/design/chip/tile/microwatt
    source piton/piton_settings.bash
    convert_microwatt            # all six netlists, into the gen/ directories
    cd $MODEL_DIR
    sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_build
    sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_run mw_hello.S

The run ends with `Simulation -> PASS (HIT GOOD TRAP)`, and `fake_uart.log`
holds `Hello from Microwatt on OpenPiton`.

| sims option | Effect |
|---|---|
| `-microwatt` | Microwatt tiles: `FLIST_MICROWATT`, `PITON_MICROWATT`, `RTL_MICROWATT<n>`. Implies `-power` (diags built with `power_cc` / `power_img`) and an L1D of 1 KiB, direct-mapped |
| `-mw_platform` | Adds the XICS interrupt controller and the syscon `CPU_CTRL` (`PITON_MW_XICS`, `PITON_MW_SYSCON`, `verif/env/manycore/devices_microwatt_platform.xml`). 1 or 2 tiles |
| `-power_target_triple=<prefix>` | Cross-compiler prefix, default `$POWER_TARGET_TRIPLE` or `powerpc64le-linux-gnu` |

Diags live in `verif/diag/power`:

| Diag | Expected result |
|---|---|
| `mw_hello.S` | PASS. Prints on the UART, then writes a line, flushes it with `dcbf` and reads it back |
| `mw_hello_c.c` | PASS. The C variant (start-up code in `include/mw_head.S`) |
| `mw_fail.S` | HIT BAD TRAP. The control: a flow that passes it checks nothing |

End of test: a diag branches to `mw_pass` or `mw_fail`
(`verif/diag/power/include/mw_diag.h`). Each stores to the data symbol
`pass` or `fail`. `power_img` writes those addresses into `symbol.tbl` as
good and bad trap. `pc_cmp` matches them against the address of each store
the tile sends to the L1.5. Microwatt's dcache is write-through, so every
store reaches the TRI. A diag that reaches neither times out.

Microwatt starts at address 0, little-endian. Core addresses below
0xC000_0000 are memory, and core 0xCxxx_xxxx is OpenPiton's device space
0xFF_Fxxx_xxxx, so the UART is at 0xC0C2_C000. Accesses to it must be
cache-inhibited (`stbcix`).

With 2 tiles only tile 0 runs: the IOB wakes tile 0, and software starts the
others through the syscon (`-mw_platform`).

## FPGA builds

`protosyn` does not build the Microwatt tile yet: it has no Microwatt core
option. Vivado reads VHDL, so an FPGA build can list the wrapper and
Microwatt's sources directly instead of the converted netlists.

`design/xilinx/kv260` holds the KV260 RTL: the PS-DDR memory shim, the open
16550 UART, and the `devices_microwatt.xml` map. The KV260 build itself is
not in this tree: the Zynq PS block design, its constraints and the PS-side
program loader that starts the tiles.

## Configuration

Size the L1D to match the core's data cache (`CONFIG_L1D_SIZE=1024`,
`CONFIG_L1D_ASSOCIATIVITY=1` for 64 x 16 B direct-mapped): the L1.5 shadows
it. sims sets this for `-microwatt`.
