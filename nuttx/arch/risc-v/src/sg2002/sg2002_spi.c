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

#define SG2002_SPI_FRAME_LEN(x)             (x - 1)

#define SG2002_CLOCK_FREQUENCE              8000000

#define SG2002_Priv_2_BaseReg(x)            (volatile sg2002_spi_reg_TypeDef *)((uintptr_t)(x))

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
    uint32_t clock;
    SG2002_SPI_FrameBitLen_List_TypeDef bit_len;
    SG2002_SPI_Mode_List_TypeDef mode;
};

struct sg2002_spi_priv_s {
    /* Standard SPI operations */
    const struct spi_ops_s *ops;

    /* port configuration */
    const struct sg2002_spi_config_s *config;

    uint32_t speed_hz;                              /* bus speed in hz */
    int refs;                                       /* Reference count */
    bool wait_irq;
};

static const struct sg2002_spi_config_s sg2002_spi1_config = {
    .base = SG2002_SPI_1_BASE,
    .irq = SG2002_IRQ_SPI1_BASE,
    .clock = SG2002_CLOCK_FREQUENCE,        /* default bus clock fequence 8M */
    .bit_len = SG2002_SPI_Frame_8Bit,
    .mode = SG2002_SPI_Mode3,
};

static const struct sg2002_spi_config_s sg2002_spi2_config = {
    .base = SG2002_SPI_2_BASE,
    .irq = SG2002_IRQ_SPI2_BASE,
    .clock = SG2002_CLOCK_FREQUENCE,
    .bit_len = SG2002_SPI_Frame_8Bit,
    .mode = SG2002_SPI_Mode3,
};

static const struct spi_ops_s sg2002_spi_ops = {
//   .lock              = spi_lock,
//   .select            = stm32_spi1select,
//   .setfrequency      = spi_setfrequency,
//   .setmode           = spi_setmode,
//   .setbits           = spi_setbits,
//   .send              = spi_send,
//   .sndblock          = spi_sndblock,
//   .recvblock         = spi_recvblock,
  .status            = NULL,
  .registercallback  = NULL,
};

static struct sg2002_spi_priv_s sg2002_spi1_priv = {
    .ops = &sg2002_spi_ops,
    .config = &sg2002_spi1_config,
    .refs = 0,
};

static struct sg2002_spi_priv_s sg2002_spi2_priv = {
    .ops = &sg2002_spi_ops,
    .config = &sg2002_spi2_config,
    .refs = 0,
};

static bool sg2002_check_spibus_base(uint32_t base);

static bool sg2002_check_spibus_base(uint32_t base) {
    if ((base == SG2002_SPI_1_BASE) || (base == SG2002_SPI_2_BASE))
        return true;

    return false;
}

/* bus enable control */
static bool sg2002_spi_enctl(struct sg2002_spi_priv_s *priv, bool state) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;
    
    /* set first */
    SG2002_SpiEnr_Reg *spienr = To_SG2002_SpiEnr_Reg(spi_reg->enr);
    spienr->field.spienr = state;

    /* check set state */
    if (spienr->field.spienr != state)
        return false;
    
    return true;
}

static bool sg2002_spi_clear_all_irq_mask(struct sg2002_spi_priv_s *priv) {
    SG2002_Imr_Reg imr_tmp;
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;

    /* get default irq mask */
    imr_tmp.val = 0;
    imr_tmp.val = To_SG2002_Imr_Reg(spi_reg->imr);

    imr_tmp.field.transmit_fifo_empty_int_mask = 0;
    imr_tmp.field.transmit_fifo_overflow_int_mask = 0;
    imr_tmp.field.receive_fifo_underflow_int_mask = 0;
    imr_tmp.field.receive_fifo_overflow_int_mask = 0;
    imr_tmp.field.receive_fifo_full_int_mask = 0;
    imr_tmp.field.multi_master = 0;

    To_SG2002_Imr_Reg(spi_reg->imr)->val = imr_tmp.val;

    /* check value after set */
    if (To_SG2002_Imr_Reg(spi_reg->imr)->val != imr_tmp.val)
        return false;

    return true;
}

static bool sg2002_spi_slavemode_ctl(struct sg2002_spi_priv_s *priv, bool state) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;

    To_SG2002_Ser_Reg(spi_reg->ser)->field.ser = state;

    if (state != To_SG2002_Ser_Reg(spi_reg->ser)->field.ser)
        return false;

    return true;
}

/* spi reset */
static bool sg2002_spi_reset(struct sg2002_spi_priv_s *priv) {
    bool ret = false;
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);

    ret = sg2002_spi_enctl(priv, false);

    /* clear all inerrupt mask */
    ret &= sg2002_spi_clear_all_irq_mask(priv);

    /* do read ICR register */
    To_SG2002_Icr_Reg(spi_reg->icr)->val;

    /* disable spi slave mode */
    ret &= sg2002_spi_slavemode_ctl(priv, false);

    ret &= sg2002_spi_enctl(priv, true);

    return ret;
}

static bool sg2002_spi_set_clock(struct sg2002_spi_priv_s *priv) {
    uint16_t clk_div = 0;
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;

    clk_div = (SG2002_SPI_REF_CLOCK + priv->config->clock - 1) / priv->config->clock;
    clk_div &= 0xFFFE;
    priv->speed_hz = SG2002_SPI_REF_CLOCK / clk_div;

    /* set baud register */
    To_SG2002_Baudr_Reg(spi_reg->baudr)->field.baudr = clk_div;

    /* setting failed */
    if (To_SG2002_Baudr_Reg(spi_reg->baudr)->field.baudr != clk_div)
        return false;

    return true;
}

static bool sg2002_spi_set_FrameLen(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint8_t bit_len = 0;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;

    /* set 8bit frame size as default */
    switch ((uint8_t)(priv->config->bit_len))
    {
        case SG2002_SPI_Frame_16Bit:
            bit_len = SG2002_SPI_FRAME_LEN(16);
            break;
        
        case SG2002_SPI_Frame_8Bit:
        default:
            bit_len = SG2002_SPI_FRAME_LEN(8);
            break;
    }

    /* write frame bit len to crtlr0 register */
    To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.data_frame_size = bit_len;

    /* setting frame bit len failed */
    if (To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.data_frame_size != bit_len)
        return false;

    return true;
}

static bool sg2002_spi_set_mode(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint8_t phase = 1;
    uint8_t polarity = 1;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;
    
    /* default mode3 */
    switch (priv->config->mode)
    {
        case SG2002_SPI_Mode0:
            phase = 0;
            polarity = 0;
            break;
    
        case SG2002_SPI_Mode1:
            polarity = 0;
            break;
    
        case SG2002_SPI_Mode2:
            phase = 0;
            break;
    
        case SG2002_SPI_Mode3:
        default:
            break;
    }

    /* write clock phase to crtlr0 register */
    To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.serial_clock_phase = phase;

    /* write clock polarity to crtlr0 register */
    To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.serial_clock_polarity = polarity;
    
    /* check setting */
    if ((To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.serial_clock_phase != phase) | \
        (To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->field.serial_clock_polarity != polarity))
        return false;

    return true;
}

static void sg2002_spi_cs_ctl(struct sg2002_spi_priv_s *priv, bool state) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return;

    To_SG2002_Ser_Reg(spi_reg->ser)->field.ser = state;
}

struct spi_dev_s *sg2002_spibus_initialize(int port) {
    struct sg2002_spi_priv_s *priv = NULL;
    bool state = true;

    switch (port) {
        case SG2002_SPI_1:
            sg2002_pinmux_config(sg2002_pinmux_spi1);
            priv = &sg2002_spi1_priv;
            break;

        case SG2002_SPI_2:
            sg2002_pinmux_config(sg2002_pinmux_spi2);
            priv = &sg2002_spi2_priv;
            break;

        default: return NULL;
    }

    state &= sg2002_spi_reset(priv);

    state &= sg2002_spi_enctl(priv, 0);

    /* set clock */
    state &= sg2002_spi_set_clock(priv);

    /* set frame lenth */
    state &= sg2002_spi_set_FrameLen(priv);

    /* set spi mode */
    state &= sg2002_spi_set_mode(priv);

    /* set cs pin */
    sg2002_spi_cs_ctl(priv, SG2002_SPI_CS_SET);

    state &= sg2002_spi_enctl(priv, 1);

    if (!state)
        return NULL;

    priv->refs ++;
    return ((struct spi_dev_s *)priv);
}

/* developping */
int sg2002_spibus_uninitialize(struct spi_dev_s *dev) {
    return -1;
}
