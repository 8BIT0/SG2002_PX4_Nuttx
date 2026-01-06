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
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>
#include "hardware/sg2002_mmio.h"

#include "sg2002_i2c.h"

typedef struct {
	volatile uint32_t ic_con;               /* 0x00 */
	volatile uint32_t ic_tar;               /* 0x04 */
	volatile uint32_t ic_sar;               /* 0x08 */
	volatile uint32_t ic_hs_maddr;          /* 0x0c */
	volatile uint32_t ic_cmd_data;          /* 0x10 */
	volatile uint32_t ic_ss_scl_hcnt;       /* 0x14 */
	volatile uint32_t ic_ss_scl_lcnt;       /* 0x18 */
	volatile uint32_t ic_fs_scl_hcnt;       /* 0x1c */
	volatile uint32_t ic_fs_scl_lcnt;       /* 0x20 */
	volatile uint32_t ic_hs_scl_hcnt;       /* 0x24 */
	volatile uint32_t ic_hs_scl_lcnt;       /* 0x28 */
	volatile uint32_t ic_intr_stat;         /* 0x2c */
	volatile uint32_t ic_intr_mask;         /* 0x30 */
	volatile uint32_t ic_raw_intr_stat;     /* 0x34 */
	volatile uint32_t ic_rx_tl;             /* 0x38 */
	volatile uint32_t ic_tx_tl;             /* 0x3c */
	volatile uint32_t ic_clr_intr;          /* 0x40 */
	volatile uint32_t ic_clr_rx_under;      /* 0x44 */
	volatile uint32_t ic_clr_rx_over;       /* 0x48 */
	volatile uint32_t ic_clr_tx_over;       /* 0x4c */
	volatile uint32_t ic_clr_rd_req;        /* 0x50 */
	volatile uint32_t ic_clr_tx_abrt;       /* 0x54 */
	volatile uint32_t ic_clr_rx_done;       /* 0x58 */
	volatile uint32_t ic_clr_activity;      /* 0x5c */
	volatile uint32_t ic_clr_stop_det;	    /* 0x60 */
	volatile uint32_t ic_clr_start_det;     /* 0x64 */
	volatile uint32_t ic_clr_gen_call;      /* 0x68 */
	volatile uint32_t ic_enable;            /* 0x6c */
	volatile uint32_t ic_status;            /* 0x70 */
	volatile uint32_t ic_txflr;             /* 0x74 */
	volatile uint32_t ic_rxflr;	            /* 0x78 */
	volatile uint32_t ic_sda_hold;          /* 0x7c */
	volatile uint32_t ic_tx_abrt_source;	/* 0x80 */
	volatile uint32_t ic_slv_dat_nack_only; /* 0x84 */
	volatile uint32_t ic_dma_cr; 		    /* 0x88 */
	volatile uint32_t ic_dma_tdlr;	        /* 0x8c */
	volatile uint32_t ic_dma_rdlr;	        /* 0x90 */
	volatile uint32_t ic_sda_setup;	        /* 0x94 */
	volatile uint32_t ic_ack_general_call;  /* 0x98 */
	volatile uint32_t ic_enable_status;	    /* 0x9c */
	volatile uint32_t ic_fs_spklen;	        /* 0xa0 */
	volatile uint32_t ic_hs_spklen;	        /* 0xa4 */
} sg2002_i2c_reg_TypeDef;

struct sg2002_i2c_config_s {
    uint32_t base;                          /* I2C base address */
    uint32_t irq;                           /* I2C IRQ */
};

/* I2C Device Private Data */

struct sg2002_i2c_priv_s {
    /* Standard I2C operations */
    const struct i2c_ops_s *ops;

    /* Port configuration */
    const struct sg2002_i2c_config_s *config;
    
    sg2002_i2c_reg_TypeDef regs; /* I2C registers */

    int refs;                    /* Reference count */
    sem_t sem_excl;              /* Mutual exclusion semaphore */
    sem_t sem_isr;               /* Interrupt wait semaphore */
    volatile uint8_t intstate;   /* Interrupt handshake */

    uint8_t msgc;                /* Message count */
    struct i2c_msg_s *msgv;      /* Message list */
    uint8_t *ptr;                /* Current message buffer */
    uint32_t frequency;          /* Current I2C frequency */
    int dcnt;                    /* Current message length */
    uint16_t flags;              /* Current message flags */

    uint32_t status;             /* End of transfer SR2|SR1 status */
};

static inline void sg2002_i2c_sem_init(struct sg2002_i2c_priv_s *priv);

/* I2C interface */

// static const struct i2c_ops_s sg2002_i2c_ops = {
//     .transfer = sg2002_i2c_transfer
// };

/* I2C device structures */

#ifdef CONFIG_SG2002_I2C1
static const struct sg2002_i2c_config_s sg2002_i2c1_config = {
    .base       = SG2002_I2C1_BASE,
    .irq        = SG2002_IRQ_I2C1_BASE,
};

static struct sg2002_i2c_priv_s sg2002_i2c1_priv = {
    .ops        = &sg2002_i2c_ops,
    .config     = &sg2002_i2c1_config,
    .refs       = 0,
    .intstate   = INTSTATE_IDLE,
    .msgc       = 0,
    .msgv       = NULL,
    .ptr        = NULL,
    .dcnt       = 0,
    .flags      = 0,
    .status     = 0
};
#endif

#ifdef CONFIG_SG2002_I2C3
static const struct sg2002_i2c_config_s sg2002_i2c3_config = {
    .base       = SG2002_I2C3_BASE,
    .irq        = SG2002_IRQ_I2C3_BASE,
};

static struct sg2002_i2c_priv_s sg2002_i2c3_priv = {
    .ops        = &sg2002_i2c_ops,
    .config     = &sg2002_i2c3_config,
    .refs       = 0,
    .intstate   = INTSTATE_IDLE,
    .msgc       = 0,
    .msgv       = NULL,
    .ptr        = NULL,
    .dcnt       = 0,
    .flags      = 0,
    .status     = 0
};
#endif

static inline void sg2002_i2c_sem_init(struct sg2002_i2c_priv_s *priv) {
    nxsem_init(&priv->sem_excl, 0, 1);

    /* This semaphore is used for signaling and, hence, should not have
    * priority inheritance enabled.
    */

    nxsem_init(&priv->sem_isr, 0, 0);
    nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
}

static int sg2002_i2c_init(struct sg2002_i2c_priv_s *priv) {

}

struct i2c_master_s *sg2002_i2cbus_initialize(int port) {
    struct sg2002_i2c_priv_s *priv = NULL;
    irqstate_t flags;

    switch (port) {
#ifdef CONFIG_SG2002_I2C1
        case 1: {
            sg2002_pinmux_config(sg2002_pinmux_i2c1);
            priv = (struct sg2002_i2c_priv_s *)&sg2002_i2c1_priv;
            break;
        }
#endif

#ifdef CONFIG_SG2002_I2C3
        case 3: {
            sg2002_pinmux_config(sg2002_pinmux_i2c3);
            priv = (struct sg2002_i2c_priv_s *)&sg2002_i2c3_priv;
            break;
        }
#endif

        default: return NULL;
    }

    /* Initialize private data for the first time, increment reference count,
    * power-up hardware and configure GPIOs.
    */

    flags = enter_critical_section();

    if ((volatile int)priv->refs == 0) {
        sg2002_i2c_sem_init(priv);
        // stm32_i2c_init(priv);
        (volatile int)priv->refs ++;
    }

    leave_critical_section(flags);
    return (struct i2c_master_s *)priv;
}

int sg2002_i2cbus_uninitialize(struct i2c_master_s *dev) {

    return -1;
}

