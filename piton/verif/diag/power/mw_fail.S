/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_fail.S - the red control for the Microwatt sims flow: it ends at
 * mw_fail, so sims must report HIT BAD TRAP. A flow that reports PASS for
 * this diag is not checking anything.
 */
#include "mw_diag.h"

	.text
	MW_VECTORS start
start:
	b	mw_fail

	MW_DIAG_END
