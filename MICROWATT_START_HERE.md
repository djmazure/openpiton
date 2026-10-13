# Microwatt tiles in OpenPiton: start here

This branch adds a Microwatt core (OpenPOWER, Power ISA 3.0) as an OpenPiton
tile, with L1D and L1I coherence through the L1.5. It also adds the chipset
devices a Microwatt Linux needs (XICS interrupt controller, syscon) and the
diags and `sims` options to run it. The full description is
`piton/design/chip/tile/microwatt_tile/README.md`.

## What is NOT on upstream, and what you cannot reproduce from here

- **The Microwatt core is a fork.** The submodule
  `piton/design/chip/tile/microwatt` is pinned to `c0a3fd30` on
  `https://github.com/djmazure/microwatt`, branch `opn/tri-atomics`. That is
  7 commits on Microwatt master `f341bed`, none of them upstream:
  - an external-atomics mode for `lwarx`/`stcx.`;
  - external invalidation of one icache index;
  - the cache-inhibited attribute of each data beat as an output;
  - `DCBZ_ALLOCATE` (a non-allocating `dcbz` miss);
  - a reset of `r1.store_way`;
  - configurable icache and dcache line sizes;
  - the PIR value taken from a port.
- **Only the simulation results below can be reproduced from this branch.**
  Microwatt Linux (1 and 2 CPUs) has also run on a KV260, but that build is not
  in this tree: `protosyn` has no Microwatt option, and the Zynq PS block design,
  constraints and PS-side loader live elsewhere.
  `piton/design/xilinx/kv260` holds only the board RTL (PS-DDR memory shim,
  16550 UART, device map).

## Prerequisites

| Tool | Used for |
|---|---|
| `/bin/csh` (`csh` or `tcsh` package) | `bw_cpp`, which `sims` uses to preprocess its config |
| yosys with the ghdl plugin | `convert_microwatt` (Microwatt is VHDL) |
| Verilator 5 (tested: 5.036) | the simulation model |
| powerpc64le GCC and binutils (`powerpc64le-linux-gnu-`, e.g. Ubuntu `gcc-powerpc64le-linux-gnu`) | building the diags |

## Run it

    git clone -b opn/microwatt-tile-move https://github.com/djmazure/openpiton.git
    cd openpiton
    git submodule update --init piton/design/chip/tile/microwatt
    export PITON_ROOT=$PWD
    source piton/piton_settings.bash
    convert_microwatt                  # Microwatt VHDL -> Verilog netlists
    mkdir -p $MODEL_DIR && cd $MODEL_DIR

Build each model once, then run diags on it (`-build_id` keeps the models
apart):

    sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_build -build_id=m1x1
    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_build -build_id=m2x1
    sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_build -build_id=m2x2

    sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_run -build_id=m1x1 mw_hello.S
    sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_run -build_id=m2x2 mw_hello.S
    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run -build_id=m2x1 mw_coh.S
    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run -build_id=m2x1 mw_icoh.S
    sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_run -build_id=m2x2 mw_coh4.S

Each run ends with

    Simulation -> PASS (HIT GOOD TRAP)

and `mw_hello.S` also writes `Hello from Microwatt on OpenPiton` to
`fake_uart.log`.

| Diag | Model | What it checks |
|---|---|---|
| `mw_hello.S` | 1x1, 2x2 | UART output, then a line written, flushed with `dcbf` and read back |
| `mw_coh.S` | 2x1 | tile 1's store invalidates the line tile 0 holds in its L1D |
| `mw_coh4.S` | 2x2 | one store from tile 0 invalidates the line in all three other tiles |
| `mw_icoh.S` | 2x1 | tile 1 rewrites code tile 0 has cached; tile 0 runs the new code |

## Red controls

A diag that passes on a broken design checks nothing, so each coherence diag
also runs on a model with a test-only mutation in
`piton/design/chip/tile/microwatt_tile/rtl/mw_tile.v` that drops the L1.5's
invalidations before they reach Microwatt:

    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_build -build_id=m2x1ns -config_rtl=MW_MUTANT_NO_SNOOP
    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_build -build_id=m2x1ni -config_rtl=MW_MUTANT_NO_ICINV
    sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_build -build_id=m2x2ns -config_rtl=MW_MUTANT_NO_SNOOP

    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run -build_id=m2x1ns mw_coh.S
    sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run -build_id=m2x1ni mw_icoh.S
    sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_run -build_id=m2x2ns mw_coh4.S

Each of these must end with

    Simulation -> FAIL(HIT BAD TRAP)

`mw_fail.S` on any model is the end-of-test control: it must HIT BAD TRAP too.
