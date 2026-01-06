#include "hardware/sg2002_mmio.h"
#include "hardware/sg2002_memorymap.h"

#include "riscv_internal.h"
#include "chip.h"

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

        case sg2002_pinmux_i2c1: {
            /* set i2c 1 sda pin */
            mmio_clrsetbits_32(SG2002_I2C1_AF_SDA_REG, 0x07, SG2002_SD1_D0_IIC1_SDA);
            /* set i2c 1 scl pin */
            mmio_clrsetbits_32(SG2002_I2C1_AF_SCL_REG, 0x07, SG2002_SD1_D3_IIC1_SCL);
            return true;
        }

        case sg2002_pinmux_i2c3:{
            /* set i2c 3 sda pin */
            mmio_clrsetbits_32(SG2002_I2C3_AF_SDA_REG, 0x07, SG2002_SD1_CLK_IIC3_SDA);
            /* set i2c 3 scl pin */
            mmio_clrsetbits_32(SG2002_I2C3_AF_SCL_REG, 0x07, SG2002_SD1_CMD_IIC3_SCL);
            return true;
        }

        default: return false;
    }

    return false;
}

