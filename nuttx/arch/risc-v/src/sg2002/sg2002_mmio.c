#include "hardware/sg2002_mmio.h"
#include "hardware/sg2002_memorymap.h"

#include "riscv_internal.h"
#include "chip.h"

bool sg2002_pinmux_config(sg2002_pinmux_list index)
{
    switch ((uint8_t) index)
    {
        case sg2002_pinmux_uart0: {
            return false;
        }

        case sg2002_pinmux_uart1: {
            /* set uart1 rx pin */
            mmio_clrsetbits_32(SG2002_UART1_AF_RX_REG, 0x07, SG2002_JTAG_CPU_TMS_AF_UART1_RX);
            /* set uart1 tx pin */
            mmio_clrsetbits_32(SG2002_UART1_AF_TX_REG, 0x07, SG2002_JTAG_CPU_TCK_AF_UART1_TX);
            return true;
        }

#if defined (CONFIG_I2C)
#if defined (CONFIG_SG2002_I2C1)
        case sg2002_pinmux_i2c1: {
            /* set i2c 1 sda pin */
            mmio_clrsetbits_32(SG2002_I2C1_AF_SDA_REG, 0x07, SG2002_SD1_D0_IIC1_SDA);
            /* set i2c 1 scl pin */
            mmio_clrsetbits_32(SG2002_I2C1_AF_SCL_REG, 0x07, SG2002_SD1_D3_IIC1_SCL);
            return true;
        }
#endif
#if defined (CONFIG_SG2002_I2C3)
        case sg2002_pinmux_i2c3:{
            /* set i2c 3 sda pin */
            mmio_clrsetbits_32(SG2002_I2C3_AF_SDA_REG, 0x07, SG2002_SD1_CLK_IIC3_SDA);
            /* set i2c 3 scl pin */
            mmio_clrsetbits_32(SG2002_I2C3_AF_SCL_REG, 0x07, SG2002_SD1_CMD_IIC3_SCL);
            return true;
        }
#endif
#endif

#if defined (CONFIG_SG2002_SPI2)
        case sg2002_pinmux_spi2: {
            /* set spi 2 clk  pin */
            mmio_clrsetbits_32(SG2002_SPI2_AF_SCK_REG, 0x07, SG2002_SD1_CLK_SPI2_SCK);
            /* set spi 2 miso pin */
            mmio_clrsetbits_32(SG2002_SPI2_AF_MISO_REG, 0x07, SG2002_SD1_D0_SPI2_MISO);
            /* set spi 2 mosi pin */
            mmio_clrsetbits_32(SG2002_SPI2_AF_MOSI_REG, 0x07, SG2002_SD1_CMD_SPI2_MOSI);
#if defined (CONFIG_SG2002_SPI_SW_CS)
            /* use spi 2 cs pin (software) */
            mmio_clrsetbits_32(SG2002_SPI2_AF_CS_REG, 0x07, SG2002_SD1_D3_SPI2_CS_SW);
#else
            /* set spi 2 cs pin (hardware) */
            mmio_clrsetbits_32(SG2002_SPI2_AF_CS_REG, 0x07, SG2002_SD1_D3_SPI2_CS_HW);
#endif
            return true;
        }
#endif

#if defined (CONFIG_SG2002_SPI3)
        case sg2002_pinmux_spi3: {
            /* use software CS pin */

            /* set spi 3 clk  pin */
            // mmio_clrsetbits_32();
            /* set spi 3 miso pin */
            // mmio_clrsetbits_32();
            /* set spi 3 mosi pin */
            // mmio_clrsetbits_32();
            /* set spi 3 cs pin */
            // mmio_clrsetbits_32();
            return false;
        }
#endif
        default: return false;
    }

    return false;
}

