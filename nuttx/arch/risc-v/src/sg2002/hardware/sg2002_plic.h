/****************************************************************************
 * arch/risc-v/src/sg2002/hardware/sg2002_plic.h
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

#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "sg2002_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Priority */

#define SG2002_PLIC_PRIORITY  	(SG2002_PLIC_BASE + 0x000000)

#define SG2002_PLIC_PENDING1    (SG2002_PLIC_BASE + 0x001000)
#define SG2002_PLIC_PENDING2    (SG2002_PLIC_BASE + 0x001004)
#define SG2002_PLIC_PENDING3    (SG2002_PLIC_BASE + 0x001008)
#define SG2002_PLIC_PENDING4    (SG2002_PLIC_BASE + 0x00100C)

/* Hart 0 M-Mode Interrupt Enable */

#define SG2002_PLIC_ENABLE1   	(SG2002_PLIC_BASE + 0x002000)
#define SG2002_PLIC_ENABLE2   	(SG2002_PLIC_BASE + 0x002004)
#define SG2002_PLIC_ENABLE3   	(SG2002_PLIC_BASE + 0x002008)
#define SG2002_PLIC_ENABLE4   	(SG2002_PLIC_BASE + 0x00200C)

/* Hart 0 M-Mode Priority Threshold */

#define SG2002_PLIC_THRESHOLD 	(SG2002_PLIC_BASE + 0x200000)

/* Hart 0 M-Mode Claim / Complete */

#define SG2002_PLIC_CLAIM     	(SG2002_PLIC_BASE + 0x200004)

#ifdef CONFIG_SG2002_QEMU_SIM
#define SG2002_CLINT_BASE		(0x02000000)
#define SG2002_CLINT_MTIME      (SG2002_CLINT_BASE + 0xbff8)
#else
#define SG2002_CLINT_BASE		(SG2002_PLIC_BASE + 0x4000000)
#define SG2002_CLINT_MTIME      0
#endif

#define SG2002_CLINT_MTIMECMP	(SG2002_CLINT_BASE + 0x4000)

#endif /* __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H */
