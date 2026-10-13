/* SPDX-FileCopyrightText: 2026 Daniel J. Mazure
 * SPDX-License-Identifier: MIT
 *
 * Microwatt diag link script: everything from address 0 (Microwatt's reset
 * address), a 16 KiB stack after the image. */
ENTRY(_start)
SECTIONS
{
	. = 0;
	.text : { KEEP(*(.text.start)) *(.text*) }
	. = ALIGN(64);
	.rodata : { *(.rodata*) }
	. = ALIGN(64);
	.data : { *(.data*) *(.sdata*) }
	. = ALIGN(256);
	.got : { __toc_start = .; *(.got) *(.toc) }
	. = ALIGN(64);
	.bss : { *(.sbss*) *(.bss*) *(COMMON) }
	. = ALIGN(64);
	. = . + 0x4000;
	_stack_top = .;
	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
