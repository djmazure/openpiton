# SPDX-FileCopyrightText: 2026 Daniel J. Mazure
# SPDX-License-Identifier: MIT
#
# Kria KV260. No package pins: the clock and reset come from the PS
# (pl_clk0, pl_resetn0), memory is PS DDR through S_AXI_HP0, and the system
# UART goes to the PS UART0 over EMIO. The PS block constrains pl_clk0 itself.
