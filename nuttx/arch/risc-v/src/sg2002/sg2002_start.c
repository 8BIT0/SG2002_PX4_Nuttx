/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_start.c
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
#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "chip.h"
#include "sg2002_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

uintptr_t g_idle_topstack = SG2002_IDLESTACK_TOP;

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sg2002_clear_bss
 *
 * Description:
 *   Clear .bss.  We'll do this inline (vs. calling memset) just to be
 *   certain that there are no issues with the state of global variables.
 *
 ****************************************************************************/

void sg2002_clear_bss(void)
{
	uint32_t *dest = NULL;

	for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
    	*dest++ = 0;
    }
}

/****************************************************************************
 * Name: sg2002_start
 *
 * Description:
 *   Start the NuttX Kernel.  Called by Boot Code.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void sg2002_start(int mhartid)
{
  	DEBUGASSERT(mhartid == 0); /* Only Hart 0 supported for now */
  
  	if (0 == mhartid)
    {
      	/* Clear the BSS */

      	sg2002_clear_bss();
	}
	else
		while (true);

  	/* Start M-mode */

#ifdef USE_EARLYSERIALINIT
  	riscv_earlyserialinit();
#endif

  	/* Call nx_start() */

  	nx_start();

  	while (true)
    {
      	asm("WFI");
    }
}

