#pragma once
#ifndef __SG2002_MMIO_H
#define __SG2002_MMIO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "hardware/sg2002_memorymap.h"

/* GPIO Reg */
#define SG2002_UART1_AF_TX_REG              (SG2002_PINMUX_BASE + 0x64)     /* PA 19 */
#define SG2002_UART1_AF_RX_REG              (SG2002_PINMUX_BASE + 0x68)     /* PA 18 */

/* GPIO Field Offset */
#define SG2002_UART1_TX_FIELD_OFFSET        0x00
#define SG2002_UART1_RX_FIELD_OFFSET        0x00

/* GPIO Field Mask */
#define SG2002_UART1_TX_FIELD_MASK          0x07
#define SG2002_UART1_RX_FIELD_MASK          0x07

/* GPIO Reg Value */
#define SG2002_JTAG_CPU_TCK_AF_UART1_TX     6
#define SG2002_JTAG_CPU_TMS_AF_UART1_RX     6

typedef enum
{
    sg2002_pinmux_none = -1,
    sg2002_gpio_out = 0,
    sg2002_gpio_in,
    sg2002_pinmux_uart1,
    sg2002_pinmux_uart2,
    sg2002_pinmux_spi4,
    sg2002_pinmux_i2c5,
    sg2002_pinmux_sdio1,
} sg2002_pinmux_list;

bool sg2002_pinmux_config(sg2002_pinmux_list index);

#endif
