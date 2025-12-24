/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "riscv_internal.h"
#include "riscv_macros.S"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/
  	/* Exported Symbols */
.org 0
	.extern __trap_vec
	.section .vectors, "ax"
	.global __start
	.global sg2002_trace
	.type __start,@function

__start:
	/* Continue primary hart */
	csrr a0, mhartid
	li   a1, SG2002_PRIM_HART
	bne  a0, a1, infinity_loop

	la   sp, SG2002_IDLESTACK_TOP

#ifndef CONFIG_SG2002_QEMU_SIM 
	/* invalidate all memory for BTB,BHT,DCACHE,ICACHE */
	li x3, 0x30013
	csrs mcor, x3

	/* enable ICACHE,DCACHE,BHT,BTB,RAS,WA */
	li x3, 0x7f
	csrs mhcr, x3

	/* enable data_cache_prefetch, amr */
	li x3, 0x610c
	csrs mhint, x3 #mhint
#endif

  	csrw mie, zero			/* disable all interrupts enable */
	csrw mip, zero			/* clear all pending interrupts  */

	li   x3, ((0x1 << 13) | (0x03 << 11))
	csrs CSR_MSTATUS, x3	/* enable floating point set MPP to machine mode */

    fsflags      zero
    fsrm         zero

    fence.i

	la   t0, __trap_vec
	csrw mtvec, t0			/* set the trap vector        */

  	jal	 x1, sg2002_start	/* jump to the start function */

	/* We shouldn't return from _start */

	.global _init
	.global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret

infinity_loop:
  csrw CSR_IE, zero  /* disable all interrupt */
  wfi