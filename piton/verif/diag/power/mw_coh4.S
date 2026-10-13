/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_coh4.S - 2x2 Microwatt mesh: one store must invalidate three sharers.
 *
 *   sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_build
 *   sims -sys=manycore -x_tiles=2 -y_tiles=2 -microwatt -vlt_run mw_coh4.S
 *
 * All four tiles run this image. Tile 0 starts at reset, claims ROLE and is
 * the writer; tiles 1..3 are readers, each woken in turn by the writer's
 * inter-processor interrupt. Tile k sits at (x, y) = (k & 1, k >> 1), so
 * tiles 2 and 3 are reached through Y routing.
 *
 *   writer: for k = 1..3: NEXTID = k, wake tile k, wait READY[k].
 *           All three readers now hold X = T0 in their L1Ds.
 *           Store T1 to X, sync, DONE = DONE_MAGIC.
 *           Wait FIN[1..3]; PASS if every reader saw X change to T1.
 *   reader: id = NEXTID, load X (T0), READY[id] = 1, then loop:
 *               d = DONE (fresh), x = X
 *               x != T0          -> saw it
 *               d == DONE_MAGIC  -> stale (the store is done, X still T0)
 *           RES[id] = x, FIN[id] = 1 (saw) or 2 (stale).
 *
 * Every word another tile writes is read fresh: a load of the same L1D
 * index 1 KiB away evicts any copy first, so the load goes to the L1.5,
 * which the directory keeps coherent on its own. Only X is left to the
 * invalidation under test. The L1D is 64 x 16 B, direct-mapped (index =
 * addr[9:4]); in the 1 KiB-aligned block below the mailboxes take indexes
 * 0x00..0x0a, DONE 0x10, T1SRC 0x20 and X 0x30.
 *
 * Red control: built with -config_rtl=MW_MUTANT_NO_SNOOP, this diag must
 * HIT BAD TRAP.
 */
#include "mw_diag.h"

#define O_NEXTID	0x000
#define O_READY(k)	(16 * (k))		/* k = 1..3 */
#define O_FIN(k)	(16 * (3 + (k)))
#define O_RES(k)	(16 * (6 + (k)))
#define O_ROLE		(16 * 10)
#define O_DONE		0x100
#define O_T1SRC		0x200
#define O_X		0x300
#define T0		0x0123456789abcdef
#define T1		0x7e57c0de600dcafe
#define DONE_MAGIC	0x600d

/* rt <- 0(ra), never from a stale L1D copy */
	.macro	FRESH rt, ra
	ld	\rt, 0x400(\ra)
	ld	\rt, 0(\ra)
	.endm

/* r9 <- blk + 16 * (\slot + id), the id in \rid */
	.macro	MBOX slot, rid
	addi	9, \rid, \slot
	sldi	9, 9, 4
	add	9, 9, 30
	.endm

	.text
	MW_VECTORS start
start:
	lis	30, blk@ha
	addi	30, 30, blk@l
	ld	5, O_ROLE(30)
	cmpdi	5, 0
	bne	reader
	li	5, 1
	std	5, O_ROLE(30)		/* the writer is the first core up */
	sync

	/* ---- writer: wake readers 1..3 one by one ---- */
	li	14, 1
wake:
	std	14, O_NEXTID(30)
	sync
	andi.	6, 14, 1		/* x */
	srdi	7, 14, 1		/* y */
	MW_WAKE	6, 7
	MBOX	0, 14			/* READY[k] */
1:	FRESH	5, 9
	cmpdi	5, 1
	bne	1b
	addi	14, 14, 1
	cmpdi	14, 4
	blt	wake

	/* ---- all three hold X: one store ---- */
	ld	21, O_T1SRC(30)
	std	21, O_X(30)
	sync
	li	5, DONE_MAGIC
	std	5, O_DONE(30)
	sync

	/* ---- every reader must have seen T1 ---- */
	li	14, 1
2:	MBOX	3, 14			/* FIN[k] */
3:	FRESH	5, 9
	cmpdi	5, 0
	beq	3b
	cmpdi	5, 1
	bne	mw_fail			/* stale */
	MBOX	6, 14			/* RES[k] */
	FRESH	5, 9
	cmpd	5, 21
	bne	mw_fail
	addi	14, 14, 1
	cmpdi	14, 4
	blt	2b
	b	mw_pass

	/* ---- reader ---- */
reader:
	addi	9, 30, O_NEXTID
	FRESH	16, 9			/* my id */
	ld	20, O_X(30)		/* T0, now in this L1D */
	MBOX	0, 16			/* READY[id] */
	li	5, 1
	std	5, 0(9)
	sync
	addi	21, 30, O_DONE
watch:
	FRESH	24, 21
	lwsync
	ld	26, O_X(30)		/* hits until an invalidation removes the line */
	cmpd	26, 20
	bne	saw
	cmpdi	24, DONE_MAGIC
	bne	watch
	li	17, 2			/* stale */
	b	done
saw:
	li	17, 1
done:
	MBOX	6, 16			/* RES[id] */
	std	26, 0(9)
	sync
	MBOX	3, 16			/* FIN[id] */
	std	17, 0(9)
	sync
4:	b	4b

	MW_DIAG_END

	.section .data
	.balign	1024
blk:
	. = blk + O_T1SRC
	.quad	T1
	. = blk + O_X
	.quad	T0
	. = blk + 0x800
