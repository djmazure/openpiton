/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_icoh.S - two Microwatt tiles: code modified on one must not keep
 * running stale on the other (cross-core instruction-cache invalidation).
 *
 *   sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_build
 *   sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run mw_icoh.S
 *
 * The instruction-side twin of mw_coh.S. Tile 0 starts at reset, claims
 * ROLE and is A; tile 1 sleeps until A's inter-processor interrupt and is B.
 *
 *   A: calls fn ("li 3,T0; blr"), so fn's line is in A's I-cache, wakes B,
 *      then loops:
 *          d = DONE        (a fresh miss every time: ALIAS evicts it)
 *          isync           (the reader's half of the ISA sequence)
 *          x = fn()        (hits in A's I-cache until something removes the line)
 *          x != T0         -> PASS if x == T1, else FAIL
 *          d == DONE_MAGIC -> FAIL: B's new code is in memory, A runs the old
 *   B: stores "li 3,T1" over fn's first instruction, then the writer's half
 *      of the Power ISA sequence for modified code (dcbst; sync; icbi; sync;
 *      isync), then DONE_MAGIC to DONE.
 *
 * Power ISA: icbi on storage that is coherence-required acts on every
 * processor's instruction cache. Microwatt's icbi flushes only its own, so A
 * relies on the L1.5's instruction-cache invalidations reaching its I-cache.
 *
 * fn is alone on its 32 B I-line and A never reads it as data. In the
 * 1 KiB-aligned block below, DONE and ALIAS share L1D index 0x10.
 *
 * Red control: built with -config_rtl=MW_MUTANT_NO_ICINV (the tile drops the
 * L1.5's instruction-cache invalidations), this diag must HIT BAD TRAP.
 */
#include "mw_diag.h"

#define O_DONE		0x100
#define O_T1SRC		0x200
#define O_ROLE		0x380
#define O_ALIAS		(O_DONE + 0x400)
#define T0		0x1234
#define T1		0x5678
#define LI_R3(imm)	(0x38600000 | (imm))	/* addi 3, 0, imm */
#define DONE_MAGIC	0x600d

	.text
	MW_VECTORS start
start:
	lis	30, blk@ha
	addi	30, 30, blk@l
	ld	5, O_ROLE(30)
	cmpdi	5, 0
	bne	role_b
	li	5, 1
	std	5, O_ROLE(30)		/* A, before B can exist */
	sync

	bl	fn			/* fn's line now in A's I-cache */
	cmpdi	3, T0
	bne	mw_fail

	li	6, 1
	li	7, 0
	MW_WAKE	6, 7			/* tile (1, 0) */

watch:
	ld	24, O_DONE(30)		/* always a miss */
	ld	25, O_ALIAS(30)		/* evict DONE for the next round */
	isync
	bl	fn
	cmpdi	3, T0
	bne	saw
	cmpdi	24, DONE_MAGIC
	bne	watch
	b	mw_fail			/* stale code: B is done, fn still T0 */
saw:
	cmpdi	3, T1
	bne	mw_fail
	b	mw_pass

role_b:
	lwz	5, O_T1SRC(30)		/* "li 3,T1" */
	lis	6, fn@ha
	addi	6, 6, fn@l
	stw	5, 0(6)			/* the code A must run next */
	dcbst	0, 6
	sync
	icbi	0, 6
	sync
	isync
	li	5, DONE_MAGIC
	std	5, O_DONE(30)
	sync
1:	b	1b

	.balign	32
fn:
	li	3, T0
	blr
	.balign	32

	MW_DIAG_END

	.section .data
	.balign	1024
blk:
	. = blk + O_T1SRC
	.long	LI_R3(T1)
	. = blk + 0x800
