/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_diag.h - conventions for Microwatt diags in the OpenPiton sims flow.
 *
 * END OF TEST. A diag ends by branching to mw_pass or mw_fail. Each stores
 * to the data symbol of the same name (`pass` / `fail`) and spins. power_img
 * writes those two addresses into symbol.tbl as good_trap / bad_trap, and
 * pc_cmp matches the address of every store the tile sends to the L1.5
 * (Microwatt's dcache is write-through, so each store reaches the TRI with
 * its exact address). sims then reports PASS (HIT GOOD TRAP) or HIT BAD TRAP.
 * A diag that never reaches either times out, which sims reports as a fail.
 *
 * ADDRESSES. Microwatt starts at address 0 in little-endian mode. In an
 * OpenPiton tile, core addresses below 0xC000_0000 are physical memory, and
 * core 0xCxxx_xxxx is OpenPiton's device space 0xFF_Fxxx_xxxx
 * (mw_piton_core.v), so the chipset UART is at MW_UART_BASE. I/O accesses
 * must be cache-inhibited (stbcix / lbzcix).
 *
 * Include it from a .S diag (then put MW_DIAG_END once, after the code) or
 * from a .c diag (then call mw_pass() / mw_fail(); mw_head.S provides them).
 */
#ifndef MW_DIAG_H
#define MW_DIAG_H

#define MW_UART_BASE	0xC0C2C000

#ifdef __ASSEMBLER__

/* r10 <- MW_UART_BASE (lis sign-extends; clear the top word) */
	.macro	MW_UART_ADDR reg
	lis	\reg, MW_UART_BASE@h
	ori	\reg, \reg, MW_UART_BASE@l
	clrldi	\reg, \reg, 32
	.endm

/* mw_pass / mw_fail, the exception vectors' landing pad, and the two data
 * words whose addresses are the good and bad trap. Every vector at
 * 0x100-0xf00 is set by mw_vectors to branch to mw_fail, so an unexpected
 * exception ends the test as a fail instead of a hang. */
	.macro	MW_DIAG_END
	.globl	mw_pass
mw_pass:
	lis	3, pass@ha
	addi	3, 3, pass@l
	std	3, 0(3)
1:	b	1b
	.globl	mw_fail
mw_fail:
	lis	3, fail@ha
	addi	3, 3, fail@l
	std	3, 0(3)
1:	b	1b

	.section .data.mw_trap, "aw"
	.balign	64
	.globl	pass
pass:	.quad	0
	.balign	64
	.globl	fail
fail:	.quad	0
	.previous
	.endm

/* Wake the tile at mesh (x, y) = (\rx, \ry), registers, with an
 * inter-processor interrupt: a store of P = 1<<63 | y<<26 | x<<18 to core
 * 0xCF00_0800 (PA 0x98_0000_0800). Only tile 0 is woken at reset; every
 * other tile sleeps until this store. The store is byte-reversed (stdbrx):
 * the bridge byte-swaps store data into P-Mesh lanes, so this puts P on the
 * NoC unchanged. Clobbers r5, r8. */
	.macro	MW_WAKE rx, ry
	sync
	li	5, 1
	sldi	5, 5, 63
	sldi	8, \rx, 18
	or	5, 5, 8
	sldi	8, \ry, 26
	or	5, 5, 8
	lis	8, 0xCF00
	clrldi	8, 8, 32
	ori	8, 8, 0x0800
	stdbrx	5, 0, 8
	.endm

/* _start at 0 branching to \entry, and every exception vector to mw_fail */
	.macro	MW_VECTORS entry
	.globl	_start
_start:
	b	\entry
	.irp	v, 0x100, 0x200, 0x300, 0x380, 0x400, 0x480, 0x500, 0x600, 0x700, 0x800, 0x900, 0xc00, 0xd00, 0xe00, 0xf00
	. = \v
	b	mw_fail
	.endr
	. = 0x1000
	.endm

#else /* C */

void mw_pass(void) __attribute__((noreturn));
void mw_fail(void) __attribute__((noreturn));

static inline void mw_putc(char c)
{
	__asm__ volatile("stbcix %0, 0, %1" : : "r"(c), "r"((unsigned long)MW_UART_BASE) : "memory");
}

static inline void mw_puts(const char *s)
{
	while (*s)
		mw_putc(*s++);
}

#endif /* __ASSEMBLER__ */
#endif /* MW_DIAG_H */
