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
    const struct spi_ops_s *ops;                    /* Standard SPI operations */
    struct sg2002_spi_config_s *config;             /* port configuration */

    sem_t exclsem;                                  /* Held while chip is selected for mutual exclusion */
    uint32_t speed_hz;                              /* bus speed in hz */
    uint16_t fifo_depth;
    
    uint16_t tx_len;
    uint8_t *tx_buf;
    
    uint16_t rx_len;
    uint8_t *rx_buf;

    bool in_proto;
    int refs;                                       /* Reference count */
    bool wait_irq;
};

#ifdef CONFIG_SG2002_SPI1
static const struct sg2002_spi_config_s sg2002_spi1_config = {
    .base = SG2002_SPI_1_BASE,
    .irq = SG2002_IRQ_SPI1_BASE,
    .clock = SG2002_CLOCK_FREQUENCE,                /* default bus clock fequence 8M */
    .bit_len = SG2002_SPI_Frame_8Bit,
    .mode = SG2002_SPI_Mode3,
};
#endif

#ifdef CONFIG_SG2002_SPI2
static const struct sg2002_spi_config_s sg2002_spi2_config = {
    .base = SG2002_SPI_2_BASE,
    .irq = SG2002_IRQ_SPI2_BASE,
    .clock = SG2002_CLOCK_FREQUENCE,
    .bit_len = SG2002_SPI_Frame_8Bit,
    .mode = SG2002_SPI_Mode3,
};
#endif

/* internal function */
static bool sg2002_check_spibus_base(uint32_t base);
static bool sg2002_spi_enctl(struct sg2002_spi_priv_s *priv, bool state);
static bool sg2002_spi_clear_all_irq_mask(struct sg2002_spi_priv_s *priv);
static bool sg2002_spi_slavemode_ctl(struct sg2002_spi_priv_s *priv, bool state);
static bool sg2002_spi_reset(struct sg2002_spi_priv_s *priv);
static bool sg2002_spi_set_clock(struct sg2002_spi_priv_s *priv);
static bool sg2002_spi_set_FrameLen(struct sg2002_spi_priv_s *priv);
static bool sg2002_spi_set_mode(struct sg2002_spi_priv_s *priv);
static int32_t sg2002_spi_get_status(struct sg2002_spi_priv_s *priv, bool raw);
static void sg2002_spi_cs_ctl(struct sg2002_spi_priv_s *priv, bool state);

/* external function */
static void sg2002_spi_select_dummy(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
static uint32_t sg2002_spi_set_freq_dummy(FAR struct spi_dev_s *dev, uint32_t frequency);
static void sg2002_spi_setbits_dummy(FAR struct spi_dev_s *dev, int nbits);
static void sg2002_spi_set_mode_external(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static int sg2002_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t sg2002_spi_send_word(FAR struct spi_dev_s *dev, uint32_t wd);
static void sg2002_spi_send_block_buff(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords);
static void sg2002_spi_receive_block_buff(FAR struct spi_dev_s *dev, void *rxbuffer, size_t nwords);

static const struct spi_ops_s sg2002_spi_ops = {
    .lock              = sg2002_spi_lock,
    .select            = sg2002_spi_select_dummy,
    .setfrequency      = sg2002_spi_set_freq_dummy,
    .setmode           = sg2002_spi_set_mode_external,
    .setbits           = sg2002_spi_setbits_dummy,
    .send              = sg2002_spi_send_word,
    .sndblock          = sg2002_spi_send_block_buff,
    .recvblock         = sg2002_spi_receive_block_buff,
    .status            = NULL,
    .registercallback  = NULL,
};

#ifdef CONFIG_SG2002_SPI1
static struct sg2002_spi_priv_s sg2002_spi1_priv = {
    .ops = &sg2002_spi_ops,
    .config = &sg2002_spi1_config,
    .refs = 0,
    .in_proto = false,
};
#endif

#ifdef CONFIG_SG2002_SPI2
static struct sg2002_spi_priv_s sg2002_spi2_priv = {
    .ops = &sg2002_spi_ops,
    .config = &sg2002_spi2_config,
    .refs = 0,
    .in_proto = false,
};
#endif

/*********************************************************** internal function section ***********************************************/

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
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    SG2002_Imr_Reg imr_tmp;
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;

    /* get default irq mask */
    imr_tmp.val = 0;
    imr_tmp.val = To_SG2002_Imr_Reg(spi_reg->imr)->val;

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
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    bool ret = false;

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
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint16_t clk_div = 0;

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
    switch ((uint8_t)(priv->config->bit_len)) {
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

static int32_t sg2002_spi_get_status(struct sg2002_spi_priv_s *priv, bool raw) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return -1;
    
    if (raw) {
        SG2002_Risr_Reg risr;

        risr.val = To_SG2002_Risr_Reg(spi_reg->risr)->val;

        if (risr.field.receive_fifo_overflow_raw_int_status) {
            SG2002_SPI_TraceOut("receive fifo overflow \n");
            return -1;
        }

        if (risr.field.receive_fifo_underflow_raw_int_status) {
            SG2002_SPI_TraceOut("receive fifo underflow \n");
            return -2;
        }

        if (risr.field.transmit_contention_raw_int_status) {
            SG2002_SPI_TraceOut("transmit contention \n");
            return -3;
        }

        if (risr.field.transmit_fifo_empty_raw_int_status) {
            SG2002_SPI_TraceOut("transmit fifo empty \n");
            return -4;
        }

        if (risr.field.transmit_fifo_full_raw_int_status) {
            SG2002_SPI_TraceOut("transmit fifo full \n");
            return -5;
        }

        if (risr.field.transmit_fifo_overflow_raw_int_status) {
            SG2002_SPI_TraceOut("transmit fifo overflow \n");
            return -6;
        }
    } else {
        SG2002_Isr_Reg isr;

        isr.val = To_SG2002_Isr_Reg(spi_reg->isr)->val;

        if (isr.field.multi_master_contention_int_status) {
            SG2002_SPI_TraceOut("multi master contention \n");
            return -7;
        }

        if (isr.field.receive_fifo_full_int_status) {
            SG2002_SPI_TraceOut("receive fifo full \n");
            return -8;
        }

        if (isr.field.receive_fifo_overflow_int_status) {
            SG2002_SPI_TraceOut("receive fifo overflow \n");
            return -9;
        }

        if (isr.field.receive_fifo_underflow_int_status) {
            SG2002_SPI_TraceOut("receive fifo underflow \n");
            return -10;
        }

        if (isr.field.transmit_fifo_empty_int_status) {
            SG2002_SPI_TraceOut("transmit fifo mepty \n");
            return -11;
        }

        if (isr.field.transmit_fifo_overflow_int_status) {
            SG2002_SPI_TraceOut("transmit fifo overflow \n");
            return -12;
        }
    }

    return 0;
}

static bool sg2002_spi_set_mode(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint8_t phase = 1;
    uint8_t polarity = 1;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return false;
    
    /* default mode3 */
    switch (priv->config->mode) {
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

static uint16_t sg2002_spi_check_fifo_depth(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint16_t depth = 1;
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return 0;

    if (priv->fifo_depth == 0) {
        for (depth = 1; depth < 256; depth ++) {
            To_SG2002_TxFlr_Reg(spi_reg->txflr)->field.txflr = depth;

            if (To_SG2002_TxFlr_Reg(spi_reg->txflr)->field.txflr != depth)
                break;
        }

        priv->fifo_depth = (depth == 1) ? 0 : depth;
    }

    return depth;
}

static void sg2002_spi_cs_ctl(struct sg2002_spi_priv_s *priv, bool state) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    
    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return;

    To_SG2002_Ser_Reg(spi_reg->ser)->field.ser = state;
}

static inline uint32_t sg2002_spi_get_min(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t tmp = (a < b) ? a : b;
    return ((tmp < c) ? tmp : c);
}

static inline uint32_t sg2002_spi_get_rx_max_size(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint32_t data_in_fifo = 0;
    uint32_t set_rx_size = 0;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return 0;

    set_rx_size = priv->rx_len;
    data_in_fifo = To_SG2002_RxFlr_Reg(spi_reg->rxflr)->field.rxflr;

    if (priv->rx_buf == NULL)
        return 0;

    return (uint32_t)((set_rx_size < data_in_fifo) ? set_rx_size : data_in_fifo);
}

static inline uint32_t sg2002_spi_get_tx_max_size(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint32_t total_tx_size = 0;
    uint32_t availabel_tx_size = 0;
    uint32_t rxtx_diff = 0;
    uint32_t tmp_size = 0;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base) || (priv->tx_buf == NULL))
        return 0;

    total_tx_size = priv->tx_len;

    /* get available transimt size read fifo register */
    availabel_tx_size = priv->fifo_depth - To_SG2002_TxFlr_Reg(spi_reg->txflr)->val;

    tmp_size = (total_tx_size < availabel_tx_size) ? total_tx_size : availabel_tx_size;
    if (priv->rx_buf != NULL) {
        rxtx_diff = priv->fifo_depth - (priv->rx_len - priv->tx_len);

        /* find minimize size */
        tmp_size = sg2002_spi_get_min(total_tx_size, availabel_tx_size, rxtx_diff);
    }

    return tmp_size;
}

static void sg2002_spi_tx(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint32_t max_trans_size = 0;
    uint16_t data = 0;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return;

    max_trans_size = sg2002_spi_get_tx_max_size(priv);
    
    while (max_trans_size) {
        if (priv->tx_buf) {
            switch (priv->config->bit_len) {
                case SG2002_SPI_Frame_16Bit:
                    data = *(uint16_t *)(priv->tx_buf);
                    break;

                case SG2002_SPI_Frame_8Bit:
                default:
                    data = *(uint8_t *)(priv->tx_buf);
                    break;
            }

            To_SG2002_Data_Reg(spi_reg->data)->field.data = data;

            priv->tx_buf += (uint8_t)(priv->config->bit_len);
            priv->tx_len --;
        }
        
        max_trans_size --;
    }
}

static void sg2002_spi_rx(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    uint32_t max_trans_size = 0;
    uint16_t data_tmp = 0;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return;

    max_trans_size = sg2002_spi_get_rx_max_size(priv);

    while (max_trans_size) {
        data_tmp = To_SG2002_Data_Reg(spi_reg->data)->field.data;

        if (priv->rx_buf) {
            switch (priv->config->bit_len) {
                case SG2002_SPI_Frame_16Bit:
                    *(uint16_t *)(priv->rx_buf) = data_tmp;
                    break;
                
                case SG2002_SPI_Frame_8Bit:
                default:
                    *(uint8_t *)(priv->rx_buf) = (uint8_t)data_tmp;
                    break;
            }

            priv->rx_buf += (uint8_t)(priv->config->bit_len);
            priv->rx_len --;
        }

        max_trans_size --;
    }
}

static void sg2002_spi_transmit(struct sg2002_spi_priv_s *priv) {
    volatile sg2002_spi_reg_TypeDef *spi_reg = SG2002_Priv_2_BaseReg(priv->config->base);
    SG2002_Ctrlr0_Reg ctrlr0_tmp;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_spibus_base(priv->config->base))
        return;

    priv->in_proto = true;

    sg2002_spi_enctl(priv, false);
    sg2002_spi_clear_all_irq_mask(priv);

    /* set trans mode */
    ctrlr0_tmp.val = To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->val;
    if ((priv->tx_buf != NULL) && (priv->rx_buf != NULL)) {
        ctrlr0_tmp.field.transfer_mode = SG2002_SPI_Transmit_Receive;
    } else if ((priv->tx_buf != NULL) && (priv->rx_buf == NULL)) {
        ctrlr0_tmp.field.transfer_mode = SG2002_SPI_Receive_Only;
    } else if ((priv->tx_buf == NULL) && (priv->rx_buf != NULL)) {
        ctrlr0_tmp.field.transfer_mode = SG2002_SPI_Transmit_Only;
    } else {
        sg2002_spi_enctl(priv, true);
        return;
    }

    To_SG2002_Ctrlr0_Reg(spi_reg->ctrlr0)->val = ctrlr0_tmp.val;

    sg2002_spi_cs_ctl(priv, true);
    sg2002_spi_enctl(priv, true);

    up_udelay(10);

    do {
        sg2002_spi_tx(priv);
        sg2002_spi_rx(priv);

        /* check status */
        if (sg2002_spi_get_status(priv, true) < 0)
            return;

    } while (priv->rx_len && priv->tx_len);
    
    priv->in_proto = false;
}

/*********************************************************** external function section ***********************************************/
static void sg2002_spi_select_dummy(FAR struct spi_dev_s *dev, uint32_t devid, bool selected) {
    return;
}

static uint32_t sg2002_spi_set_freq_dummy(FAR struct spi_dev_s *dev, uint32_t frequency) {
    return 0;
}

static void sg2002_spi_setbits_dummy(FAR struct spi_dev_s *dev, int nbits) {
    return;
}

static void sg2002_spi_set_mode_external(FAR struct spi_dev_s *dev, enum spi_mode_e mode) {
    struct sg2002_spi_priv_s *priv = (struct sg2002_spi_priv_s *)dev;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_spibus_base(priv->config->base) || \
        (priv->refs == 0) || priv->in_proto)
        return;

    switch ((uint8_t) mode)
    {
        case SPIDEV_MODE0: priv->config->mode = SG2002_SPI_Mode0; break;
        case SPIDEV_MODE1: priv->config->mode = SG2002_SPI_Mode1; break;
        case SPIDEV_MODE2: priv->config->mode = SG2002_SPI_Mode2; break;
        case SPIDEV_MODE3: priv->config->mode = SG2002_SPI_Mode3; break;
        default:
            return;
    }

    sg2002_spi_set_mode(priv);
}

static int sg2002_spi_lock(FAR struct spi_dev_s *dev, bool lock) {
    struct sg2002_spi_priv_s *priv = (struct sg2002_spi_priv_s *)dev;

    if (lock)
        return nxsem_wait_uninterruptible(&priv->exclsem);
    
    return nxsem_post(&priv->exclsem);
}

static uint32_t sg2002_spi_send_word(FAR struct spi_dev_s *dev, uint32_t wd) {
    struct sg2002_spi_priv_s *priv = (struct sg2002_spi_priv_s *)dev;
    uint32_t rx_data = 0;
    uint32_t tx_data = wd;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_spibus_base(priv->config->base) || (priv->refs == 0))
        return 0;

    priv->tx_buf = (void *)&tx_data;
    priv->tx_len = sizeof(tx_data) / priv->config->bit_len;

    priv->rx_buf = (void *)&rx_data;
    priv->rx_len = sizeof(rx_data) / priv->config->bit_len;
 
    sg2002_spi_transmit(priv);

    return rx_data;
}

static void sg2002_spi_send_block_buff(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords) {
    struct sg2002_spi_priv_s *priv = (struct sg2002_spi_priv_s *)dev;
    
    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_spibus_base(priv->config->base) || (priv->refs == 0) || \
        (txbuffer == NULL) || (nwords == 0) || (nwords % priv->config->bit_len))
        return;

    priv->tx_buf = (void *)txbuffer;
    priv->tx_len = nwords / priv->config->bit_len;

    priv->rx_buf = NULL;
    priv->rx_len = 0;

    sg2002_spi_transmit(priv);
}

static void sg2002_spi_receive_block_buff(FAR struct spi_dev_s *dev, void *rxbuffer, size_t nwords) {
    struct sg2002_spi_priv_s *priv = (struct sg2002_spi_priv_s *)dev;
    
    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_spibus_base(priv->config->base) || (priv->refs == 0) || \
        (rxbuffer == NULL) || (nwords == 0) || (nwords % priv->config->bit_len))
        return;

    priv->tx_buf = NULL;
    priv->tx_len = 0;

    priv->rx_buf = rxbuffer;
    priv->rx_len = nwords / priv->config->bit_len;

    sg2002_spi_transmit(priv);
}

struct spi_dev_s *sg2002_spibus_initialize(int port) {
    struct sg2002_spi_priv_s *priv = NULL;
    bool state = true;

    switch (port) {
#ifdef CONFIG_SG2002_SPI1
        case SG2002_SPI_1:
            sg2002_pinmux_config(sg2002_pinmux_spi1);
            priv = &sg2002_spi1_priv;
            break;
#endif

#ifdef CONFIG_SG2002_SPI2
        case SG2002_SPI_2:
            sg2002_pinmux_config(sg2002_pinmux_spi2);
            priv = &sg2002_spi2_priv;
            break;
#endif
        default: return NULL;
    }

    state &= sg2002_spi_reset(priv);

    /* check tx fifo depth */
    if (sg2002_spi_check_fifo_depth(priv) == 0) {
        sg2002_spi_enctl(priv, 0);
        return NULL;
    }

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

    /* init semaphore */
    nxsem_init(&priv->exclsem, 0, 1);
    
    return ((struct spi_dev_s *)priv);
}

/* developping */
int sg2002_spibus_uninitialize(struct spi_dev_s *dev) {
    return -1;
}
