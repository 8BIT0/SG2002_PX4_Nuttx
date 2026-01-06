#pragma once
#ifndef __SG2002_MMIO_H
#define __SG2002_MMIO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "hardware/sg2002_memorymap.h"

#define __raw_readw(a)                      (*(volatile unsigned short *)(a))
#define __raw_readl(a)                      (*(volatile unsigned int *)(a))
#define __raw_readq(a)                      (*(volatile unsigned long long *)(a))

#define __raw_writeb(v,a)                   (*(volatile unsigned char *)(a) = (v))
#define __raw_writew(v,a)                   (*(volatile unsigned short *)(a) = (v))
#define __raw_writel(v,a)                   (*(volatile unsigned int *)(a) = (v))
#define __raw_writeq(v,a)                   (*(volatile unsigned long long *)(a) = (v))

#define __io_br()                           do {} while (0)
#define __io_ar(v)                          __asm__ __volatile__ ("fence i,r" : : : "memory")
#define __io_bw()                           __asm__ __volatile__ ("fence w,o" : : : "memory")
#define __io_aw()                           do {} while (0)

#define readb(c)                            ({ uint8_t  __v; __io_br(); __v = __raw_readb(c); __io_ar(__v); __v; })
#define readw(c)                            ({ uint16_t __v; __io_br(); __v = __raw_readw(c); __io_ar(__v); __v; })
#define readl(c)                            ({ uint32_t __v; __io_br(); __v = __raw_readl(c); __io_ar(__v); __v; })

#define writeb(v, c)                        ({ __io_bw(); __raw_writeb((v), (c)); __io_aw(); })
#define writew(v, c)                        ({ __io_bw(); __raw_writew((v), (c)); __io_aw(); })
#define writel(v, c)                        ({ __io_bw(); __raw_writel((v), (c)); __io_aw(); })

/* GPIO Reg */
/* uart1 reg */
#define SG2002_UART1_AF_TX_REG              (SG2002_PINMUX_BASE + 0x64)     /* PA 19 */
#define SG2002_UART1_AF_RX_REG              (SG2002_PINMUX_BASE + 0x68)     /* PA 18 */

/* i2c1 reg */
#define SG2002_I2C1_AF_SCL_REG              (SG2002_PINMUX_BASE + 0xD0)      /* PP 18  */
#define SG2002_I2C1_AF_SDA_REG              (SG2002_PINMUX_BASE + 0xD4)      /* PP 21  */

/* i2c3 reg */
#define SG2002_I2C3_AF_SCL_REG              (SG2002_PINMUX_BASE + 0xE0)      /* PP 22  */
#define SG2002_I2C3_AF_SDA_REG              (SG2002_PINMUX_BASE + 0xE4)      /* PP 23  */

/* GPIO Reg Value */
/* uart1 reg val */
#define SG2002_JTAG_CPU_TCK_AF_UART1_TX     6
#define SG2002_JTAG_CPU_TMS_AF_UART1_RX     6

/* i2c1 reg val */
#define SG2002_SD1_D3_IIC1_SCL              2
#define SG2002_SD1_D0_IIC1_SDA              2

/* i2c3 reg val */
#define SG2002_SD1_CMD_IIC3_SCL             2
#define SG2002_SD1_CLK_IIC3_SDA             2

typedef enum {
    sg2002_pinmux_none = -1,
    sg2002_gpio_out = 0,
    sg2002_gpio_in,
    sg2002_pinmux_uart1,
    sg2002_pinmux_spi2,
    sg2002_pinmux_i2c1,
    sg2002_pinmux_i2c3,
} sg2002_pinmux_list;

static inline uint32_t mmio_read_32(uintptr_t addr) {
	return readl((void *) addr);
}

static inline void mmio_clrbits_32(uintptr_t addr, uint32_t clear) {
	writel(readl((void *) addr) & ~clear , (void *) addr);
}

static inline void mmio_setbits_32(uintptr_t addr, uint32_t set) {
	writel(readl((void *) addr) | set , (void *) addr);
}

static inline void mmio_clrsetbits_32(uintptr_t addr, uint32_t clear, uint32_t set) {
	writel((readl((void *) addr) & ~clear) | set , (void *) addr);
}

bool sg2002_pinmux_config(sg2002_pinmux_list index);

#endif
