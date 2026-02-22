#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "hardware/sg2002_mmio.h"
#include "sg200x.h"

// #define SG2002_SPI_TraceOut(fmt, ...)       sg2002_trace_dirout(fmt, ##__VA_ARGS__)
#define SG2002_SPI_TraceOut(fmt, ...)

#define SG2002_Priv_2_BaseReg(x)            (volatile sg2002_spi_reg_TypeDef *)((uintptr_t)(x));

#define SG2002_Ctrlr0_Reg                   volatile SG2002_SPI_CTRLR0_Reg_TypeDef
#define SG2002_Ctrlr1_Reg                   volatile SG2002_SPI_CTRLR1_Reg_TypeDef
#define SG2002_SpiEnr_Reg                   volatile SG2002_SPI_SPIENR_Reg_TypeDef
#define SG2002_Mwcr_Reg                     volatile SG2002_SPI_MWCR_Reg_TypeDef
#define SG2002_Ser_Reg                      volatile SG2002_SPI_SER_Reg_TypeDef
#define SG2002_Baudr_Reg                    volatile SG2002_SPI_BAUDR_Reg_TypeDef
#define SG2002_TxFtlr_Reg                   volatile SG2002_SPI_TXFTLR_Reg_TypeDef
#define SG2002_RxFtlr_Reg                   volatile SG2002_SPI_RXFTLR_Reg_TypeDef
#define SG2002_TxFlr_Reg                    volatile SG2002_SPI_TXFLR_Reg_TypeDef
#define SG2002_RxFlr_Reg                    volatile SG2002_SPI_RXFLR_Reg_TypeDef
#define SG2002_Sr_Reg                       volatile SG2002_SPI_SR_Reg_TypeDef
#define SG2002_Imr_Reg                      volatile SG2002_SPI_IMR_Reg_TypeDef
#define SG2002_Isr_Reg                      volatile SG2002_SPI_ISR_Reg_TypeDef
#define SG2002_Risr_Reg                     volatile SG2002_SPI_RISR_Reg_TypeDef
#define SG2002_TxOicr_Reg                   volatile SG2002_SPI_TXOICR_Reg_TypeDef
#define SG2002_RxOicr_Reg                   volatile SG2002_SPI_RXOICR_Reg_TypeDef
#define SG2002_RxUicr_Reg                   volatile SG2002_SPI_RXUICR_Reg_TypeDef
#define SG2002_Msticr_Reg                   volatile SG2002_SPI_MSTICR_Reg_TypeDef
#define SG2002_Icr_Reg                      volatile SG2002_SPI_ICR_Reg_TypeDef
#define SG2002_DmaCr_Reg                    volatile SG2002_DMACR_Reg_TypeDef
#define SG2002_DmaTdlr_Reg                  volatile SG2002_DMATDLR_Reg_TypeDef
#define SG2002_DmaRdlr_Reg                  volatile SG2002_SPI_DMARDLR_Reg_TypeDef
#define SG2002_Data_Reg                     volatile SG2002_SPI_Data_Reg_TypeDef
#define SG2002_RxSampleDly_Reg              volatile SG2002_SPI_RxSampleDly_Reg_TypeDef

#define To_SG2002_Ctrlr0_Reg(x)             ((SG2002_Ctrlr0_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Ctrlr1_Reg(x)             ((SG2002_Ctrlr1_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_SpiEnr_Reg(x)             ((SG2002_SpiEnr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Mwcr_Reg(x)               ((SG2002_Mwcr_Reg        *)((uintptr_t)&(x)))
#define To_SG2002_Ser_Reg(x)                ((SG2002_Ser_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_Baudr_Reg(x)              ((SG2002_Baudr_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_TxFtlr_Reg(x)             ((SG2002_TxFtlr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_RxFtlr_Reg(x)             ((SG2002_RxFtlr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_TxFlr_Reg(x)              ((SG2002_TxFlr_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_RxFlr_Reg(x)              ((SG2002_RxFlr_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_Sr_Reg(x)                 ((SG2002_Sr_Reg          *)((uintptr_t)&(x)))
#define To_SG2002_Imr_Reg(x)                ((SG2002_Imr_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_Isr_Reg(x)                ((SG2002_Isr_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_Risr_Reg(x)               ((SG2002_Risr_Reg        *)((uintptr_t)&(x)))
#define To_SG2002_TxOicr_Reg(x)             ((SG2002_TxOicr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_RxOicr_Reg(x)             ((SG2002_RxOicr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_RxUicr_Reg(x)             ((SG2002_RxUicr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Msticr_Reg(x)             ((SG2002_Msticr_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Icr_Reg(x)                ((SG2002_Icr_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_DmaCr_Reg(x)              ((SG2002_DmaCr_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_DmaTdlr_Reg(x)            ((SG2002_DmaTdlr_Reg     *)((uintptr_t)&(x)))
#define To_SG2002_DmaRdlr_Reg(x)            ((SG2002_DmaRdlr_Reg     *)((uintptr_t)&(x)))
#define To_SG2002_Data_Reg(x)               ((SG2002_Data_Reg        *)((uintptr_t)&(x)))
#define To_SG2002_RxSampleDly_Reg(x)        ((SG2002_RxSampleDly_Reg *)((uintptr_t)&(x)))

#define reg_t volatile uint32_t

typedef struct {
    reg_t ctrlr0;           /* 0x00 */
    reg_t ctrlr1;           /* 0x04 */
    reg_t enr;              /* 0x08 */
    reg_t mwcr;             /* 0x0c */
    reg_t ser;              /* 0x10 */
    reg_t baudr;            /* 0x14 */
    reg_t txftlr;           /* 0x18 */
    reg_t rxftlr;           /* 0x1c */
    reg_t txflr;            /* 0x20 */
    reg_t rxflr;            /* 0x24 */
    reg_t sr;               /* 0x28 */
    reg_t imr;              /* 0x2c */
    reg_t isr;              /* 0x30 */
    reg_t risr;             /* 0x34 */
    reg_t txoicr;           /* 0x38 */
    reg_t rxoicr;           /* 0x3c */
    reg_t rxuicr;           /* 0x40 */
    reg_t msticr;           /* 0x44 */
    reg_t icr;              /* 0x48 */
    reg_t dmacr;            /* 0x4c */
    reg_t dmatdlr;          /* 0x50 */
    reg_t dmardlr;          /* 0x54 */
    reg_t data;             /* 0x60 */
    reg_t rx_sample_dly;    /* 0xf0 */
} sg2002_spi_reg_TypeDef;

struct sg2002_spi_config_s {
    uint32_t base;
    uint32_t irq;
};

struct sg2002_spi_priv_s {
    /* Standard SPI operations */
    const struct spi_ops_s *ops;

    /* port configuration */
    const struct sg2002_spi_config_s *config;

    int refs;                                       /* Reference count */
};

static bool sg2002_check_spibus_base(uint32_t base);

static bool sg2002_check_spibus_base(uint32_t base) {
    if ((base == SG2002_SPI_1_BASE) || (base == SG2002_SPI_2_BASE))
        return true;

    return false;
}

/* bus enable control */
static bool sg2002_spi_enctl(struct sg2002_spi_priv_s *priv, bool state) {
    int16_t timeout = 100;

    return false;
}

struct spi_dev_s *sg2002_spibus_initialize(int port) {
    
    return NULL;
}

/* developping */
int sg2002_spibus_uninitialize(struct spi_dev_s *dev) {
    return -1;
}
