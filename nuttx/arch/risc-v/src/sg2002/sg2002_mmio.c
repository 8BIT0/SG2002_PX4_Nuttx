#include "hardware/sg2002_mmio.h"
#include "hardware/sg2002_memorymap.h"

#include "riscv_internal.h"
#include "chip.h"

#define __raw_readw(a)          (*(volatile unsigned short *)(a))
#define __raw_readl(a)          (*(volatile unsigned int *)(a))
#define __raw_readq(a)          (*(volatile unsigned long long *)(a))

#define __raw_writeb(v,a)       (*(volatile unsigned char *)(a) = (v))
#define __raw_writew(v,a)       (*(volatile unsigned short *)(a) = (v))
#define __raw_writel(v,a)       (*(volatile unsigned int *)(a) = (v))
#define __raw_writeq(v,a)       (*(volatile unsigned long long *)(a) = (v))

#define __io_br()               do {} while (0)
#define __io_ar(v)              __asm__ __volatile__ ("fence i,r" : : : "memory")
#define __io_bw()               __asm__ __volatile__ ("fence w,o" : : : "memory")
#define __io_aw()               do {} while (0)

#define readb(c)                ({ uint8_t  __v; __io_br(); __v = __raw_readb(c); __io_ar(__v); __v; })
#define readw(c)                ({ uint16_t __v; __io_br(); __v = __raw_readw(c); __io_ar(__v); __v; })
#define readl(c)                ({ uint32_t __v; __io_br(); __v = __raw_readl(c); __io_ar(__v); __v; })

#define writeb(v, c)            ({ __io_bw(); __raw_writeb((v), (c)); __io_aw(); })
#define writew(v, c)            ({ __io_bw(); __raw_writew((v), (c)); __io_aw(); })
#define writel(v, c)            ({ __io_bw(); __raw_writel((v), (c)); __io_aw(); })

static void mmio_clrsetbits_32(uintptr_t addr, uint32_t clear, uint32_t set) {
	writel((readl((void *) addr) & ~clear) | set , (void *) addr);
}

bool sg2002_pinmux_config(sg2002_pinmux_list index)
{
    switch ((uint8_t) index)
    {
        case sg2002_pinmux_uart1: {
            /* set uart1 rx pin */
            mmio_clrsetbits_32(SG2002_UART1_AF_RX_REG, 0x07, SG2002_JTAG_CPU_TMS_AF_UART1_RX);

            /* set uart1 tx pin */
            mmio_clrsetbits_32(SG2002_UART1_AF_TX_REG, 0x07, SG2002_JTAG_CPU_TCK_AF_UART1_TX);
            return true;
        }

        default: return false;
    }

    return false;
}

