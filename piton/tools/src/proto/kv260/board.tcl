# SPDX-FileCopyrightText: 2026 Daniel J. Mazure
# SPDX-License-Identifier: MIT
#
# Board specific variables
# Not intended to be run standalone
#
# Kria KV260 (K26 SOM, Zynq UltraScale+). There is no PL DDR and no PL clock
# or UART pin: the board wraps `system` in a shell (PITONSYS_SHELL) whose PS
# supplies the clock, the reset and DDR through S_AXI_HP0, and whose UART0 is
# routed to the system UART over EMIO.
#
# BOARD_PART is left empty: the K26 board files are not part of a default
# Vivado install, and the PS is configured explicitly, not from board presets.

set BOARD_PART ""
set FPGA_PART "xck26-sfvc784-2LV-c"
set VIVADO_FLOW_PERF_OPT 0
set BOARD_DEFAULT_VERILOG_MACROS "KV260_BOARD"

# Board RTL. The system UART is the open 16550 in uart_16550's slot (the
# OpenCores register file and FIFOs that Microwatt ships), not the Xilinx
# AXI UART16550 core. afifo_w64_d128_std is a behavioural FIFO in place of the
# FIFO Generator core: the shell runs the system and the memory path on one
# clock (pl_clk0).
set UART16550_DIR "${DV_ROOT}/design/chip/tile/microwatt/uart16550"
set BOARD_INCLUDE_DIRS "${UART16550_DIR}"
set BOARD_RTL_IMPL_FILES [list \
    "${UART16550_DIR}/uart_sync_flops.v" \
    "${UART16550_DIR}/raminfr.v" \
    "${UART16550_DIR}/uart_tfifo.v" \
    "${UART16550_DIR}/uart_rfifo.v" \
    "${UART16550_DIR}/uart_transmitter.v" \
    "${UART16550_DIR}/uart_receiver.v" \
    "${UART16550_DIR}/uart_regs.v" \
    "${DV_ROOT}/design/xilinx/kv260/rtl/uart_16550.v" \
    "${DV_ROOT}/design/xilinx/kv260/sim/afifo_w64_d128_std.v" \
]

# The board top: the PS block design (kv260_ps.tcl), the run-control
# registers and the shell around `system`
lappend BOARD_RTL_IMPL_FILES \
    "${DV_ROOT}/design/xilinx/kv260/rtl/pmesh_hp_shim.v" \
    "${DV_ROOT}/design/xilinx/kv260/rtl/kv260_shell.v" \
    "${DV_ROOT}/design/xilinx/kv260/rtl/kv260_ctrl.v" \
    "${DV_ROOT}/design/xilinx/kv260/rtl/kv260_top.v"
set BOARD_TOP_MODULE "kv260_top"
set BOARD_PROJECT_TCL "${DV_ROOT}/tools/src/proto/kv260/kv260_ps.tcl"
