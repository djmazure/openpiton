/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_head.S - start-up for C Microwatt diags (power_cc links it in front of
 * diag.c): vectors, stack, TOC, then main(). main returning 0 passes, any
 * other value fails. */
#include "mw_diag.h"

	.section .text.start, "ax"
	MW_VECTORS mw_boot
mw_boot:
	lis	1, _stack_top@ha
	addi	1, 1, _stack_top@l
	addi	1, 1, -64
	lis	2, .TOC.@ha
	addi	2, 2, .TOC.@l
	bl	main
	nop
	cmpdi	3, 0
	beq	mw_pass
	b	mw_fail
	MW_DIAG_END
