/****************************************************************************
 * arch/risc-v/include/sg2002/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_SG2002_IRQ_H
#define __ARCH_RISCV_INCLUDE_SG2002_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define SG2002_IRQ_PERI_START   (RISCV_IRQ_MEXT)
#ifdef CONFIG_SG2002_QEMU_SIM
#define SG2002_IRQ_SIM_BASE     (SG2002_IRQ_PERI_START + 10)
#define NR_IRQS                 (SG2002_IRQ_SIM_BASE + 1)
#else
#define SG2002_IRQ_UART1_BASE   (SG2002_IRQ_PERI_START + 31)
#define SG2002_IRQ_I2C1_BASE    (SG2002_IRQ_PERI_START + 33)
#define SG2002_IRQ_I2C3_BASE    (SG2002_IRQ_PERI_START + 35)
#define SG2002_IRQ_SPI1_BASE    (SG2002_IRQ_PERI_START + 37)
#define SG2002_IRQ_SPI2_BASE    (SG2002_IRQ_PERI_START + 38)
#define SG2002_IRQ_MBOX_BASE    (SG2002_IRQ_PERI_START + 61)
#define NR_IRQS                 (SG2002_IRQ_PERI_START + 62)
#endif

#endif /* __ARCH_RISCV_INCLUDE_SG2002_IRQ_H */
