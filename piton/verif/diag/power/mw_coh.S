/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_coh.S - two Microwatt tiles: a store on one must reach the other's L1D.
 *
 *   sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_build
 *   sims -sys=manycore -x_tiles=2 -y_tiles=1 -microwatt -vlt_run mw_coh.S
 *
 * Both tiles run this image. Tile 0 starts at reset, claims ROLE and is A;
 * tile 1 sleeps until A's inter-processor interrupt and is B.
 *
 *   A: loads X (= T0), so X's line sits in A's L1D, wakes B, then loops:
 *          d = DONE        (a fresh miss every time: ALIAS evicts it)
 *          x = X           (hits in A's L1D until an invalidation removes it)
 *          x != T0         -> PASS if x == T1, else FAIL
 *          d == DONE_MAGIC -> FAIL: B's store has finished, A still reads T0
 *   B: stores T1 to X, sync, then DONE_MAGIC to DONE.
 *
 * DONE is read before X, and B makes its X store visible before its DONE
 * store, so seeing DONE while X still reads T0 can only mean A's L1D kept a
 * line the L2 directory invalidated. DONE itself never stays in A's L1D:
 * ALIAS has the same L1D index, so every DONE read goes to the L1.5, which
 * the directory keeps coherent on its own.
 *
 * The L1D is Microwatt's dcache, 64 lines x 16 B, direct-mapped: index =
 * addr[9:4]. In the 1 KiB-aligned block below, DONE and ALIAS share index
 * 0x10, T1SRC is 0x20, X is 0x30 and ROLE 0x38; after loading X, A touches
 * nothing else in the block's index range.
 *
 * Red control: built with -config_rtl=MW_MUTANT_NO_SNOOP (the tile drops the
 * L1.5's invalidations before they reach Microwatt's L1D), this diag must
 * HIT BAD TRAP.
 */
#include "mw_diag.h"

#define O_DONE		0x100
#define O_T1SRC		0x200
#define O_X		0x300
#define O_ROLE		0x380
#define O_ALIAS		(O_DONE + 0x400)
#define T0		0x1111222233334444
#define T1		0x5555666677778888
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

	ld	20, O_X(30)		/* T0, now in A's L1D */
	ld	21, O_T1SRC(30)		/* T1, what B will store */

	li	6, 1
	li	7, 0
	MW_WAKE	6, 7			/* tile (1, 0) */

watch:
	ld	24, O_DONE(30)		/* always a miss */
	ld	25, O_ALIAS(30)		/* evict DONE for the next round */
	lwsync
	ld	26, O_X(30)
	cmpd	26, 20
	bne	saw
	cmpdi	24, DONE_MAGIC
	bne	watch
	b	mw_fail			/* stale: B is done, X still T0 */
saw:
	cmpd	26, 21
	bne	mw_fail
	b	mw_pass

role_b:
	ld	5, O_T1SRC(30)
	std	5, O_X(30)		/* the store A must observe */
	sync
	li	5, DONE_MAGIC
	std	5, O_DONE(30)
	sync
1:	b	1b

	MW_DIAG_END

	.section .data
	.balign	1024
blk:
	. = blk + O_DONE
	.quad	0
	. = blk + O_T1SRC
	.quad	T1
	. = blk + O_X
	.quad	T0
	. = blk + O_ROLE
	.quad	0
	. = blk + O_ALIAS
	.quad	0
	. = blk + 0x800
