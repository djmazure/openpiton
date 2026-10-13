/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_hello.S - Microwatt hello for the OpenPiton sims flow.
 *
 *   sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_build
 *   sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_run mw_hello.S
 *
 * Prints "Hello from Microwatt on OpenPiton\r\n" on the chipset UART
 * (fake_uart.log), then checks the memory path: it writes a pattern to a
 * cacheable line, reads it back with a load that must miss the L1 (dcbf
 * first), and passes only if every word came back. That covers instruction
 * fetch, cacheable loads and stores through the L1.5/L2/memory, and
 * cache-inhibited stores to the device space.
 */
#include "mw_diag.h"

	.text
	MW_VECTORS start
start:
	MW_UART_ADDR 10
	lis	4, msg@ha
	addi	4, 4, msg@l
1:	lbz	3, 0(4)
	cmpwi	3, 0
	beq	2f
	stbcix	3, 0, 10
	addi	4, 4, 1
	b	1b

2:	/* 8 dwords: buf[i] = PATTERN ^ i */
	lis	5, buf@ha
	addi	5, 5, buf@l
	lis	6, 0x5a5a
	ori	6, 6, 0xc3c3
	sldi	7, 6, 32
	or	6, 6, 7			/* r6 = 0x5a5ac3c3_5a5ac3c3 */
	li	7, 0
3:	xor	8, 6, 7
	sldi	9, 7, 3
	stdx	8, 5, 9
	addi	7, 7, 1
	cmpdi	7, 8
	blt	3b
	/* push the line out of the L1 so the reads below miss */
	dcbf	0, 5
	sync
	li	7, 0
4:	sldi	9, 7, 3
	ldx	8, 5, 9
	xor	8, 8, 7
	cmpd	8, 6
	bne	mw_fail
	addi	7, 7, 1
	cmpdi	7, 8
	blt	4b
	b	mw_pass

	MW_DIAG_END

	.section .rodata
msg:	.string	"Hello from Microwatt on OpenPiton\r\n"

	.section .data
	.balign	64
buf:	.space	64
