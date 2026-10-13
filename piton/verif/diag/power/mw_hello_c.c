/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * mw_hello_c.c - the C variant of mw_hello.S: prints on the chipset UART and
 * checks a small computation. main returning 0 passes (mw_head.S).
 *
 *   sims -sys=manycore -x_tiles=1 -y_tiles=1 -microwatt -vlt_run mw_hello_c.c
 */
#include "mw_diag.h"

static volatile unsigned long fib[16];

int main(void)
{
	mw_puts("Hello from Microwatt (C) on OpenPiton\r\n");

	fib[0] = 0;
	fib[1] = 1;
	for (int i = 2; i < 16; i++)
		fib[i] = fib[i - 1] + fib[i - 2];

	return fib[15] == 610 ? 0 : 1;
}
