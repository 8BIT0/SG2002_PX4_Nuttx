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
#include "sg200x.h"

#define SG2002_Priv_2_BaseReg(x) (volatile sg2002_i2c_reg_TypeDef *)((uintptr_t)(x));

typedef struct {
	volatile uint32_t ic_con;               /* 0x00 */
	volatile uint32_t ic_tar;               /* 0x04 */
	volatile uint32_t ic_sar;               /* 0x08 */
	volatile uint32_t ic_hs_maddr;          /* 0x0c */
	volatile uint32_t ic_data_cmd;          /* 0x10 */
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
    
    int refs;                               /* Reference count */
    sem_t sem_excl;                         /* Mutual exclusion semaphore */
    sem_t sem_isr;                          /* Interrupt wait semaphore */

    uint8_t msgc;                           /* Message count */
    struct i2c_msg_s *msgv;                 /* Message list */
    uint8_t *ptr;                           /* Current message buffer */
    int dcnt;                               /* Current message length */
    uint16_t flags;                         /* Current message flags */

    uint32_t status;
};

static bool sg2002_check_i2c_base(uint32_t base);
static bool sg2002_i2c_enctl(struct sg2002_i2c_priv_s *priv, bool state);
static bool sg2002_i2c_wait_for_bb(volatile sg2002_i2c_reg_TypeDef *i2c);
static int sg2002_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
static void sg2002_i2c_dw_read(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num);
static void sg2002_i2c_xfer_init_burst(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs);
static uint32_t sg2002_i2c_dw_read_clear_intrbits(struct sg2002_i2c_priv_s *priv);
static int sg2002_i2c_irq_handle(int irq, void *context, void *arg);
static int sg2002_i2c_init(struct sg2002_i2c_priv_s *priv);
static bool sg2002_set_bus_speed(struct sg2002_i2c_priv_s *priv, uint32_t speed);
static void sg2002_i2c_sem_init(struct sg2002_i2c_priv_s *priv);

/* I2C interface */

#if defined(CONFIG_SG2002_I2C1) || defined(CONFIG_SG2002_I2C3)
static const struct i2c_ops_s sg2002_i2c_ops = {
    .transfer = sg2002_i2c_transfer
};
#endif

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
    .msgc       = 0,
    .msgv       = NULL,
    .ptr        = NULL,
    .dcnt       = 0,
    .flags      = 0,
    .status     = 0
};
#endif

static bool sg2002_check_i2c_base(uint32_t base) {
    if ((base == SG2002_I2C1_BASE) || (base == SG2002_I2C3_BASE))
        return true;

    return false;
}

/* bus enable control */
static bool sg2002_i2c_enctl(struct sg2002_i2c_priv_s *priv, bool state) {
    int16_t timeout = 100;
    volatile sg2002_i2c_reg_TypeDef *i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    volatile SG2002_IC_ENABLE_Reg_TypeDef *ic_en_reg = (volatile SG2002_IC_ENABLE_Reg_TypeDef *)&(i2c->ic_enable);
    volatile SG2002_IC_ENABLE_STATUS_Reg_TypeDef *ic_en_status_reg = (volatile SG2002_IC_ENABLE_STATUS_Reg_TypeDef *)&(i2c->ic_enable_status);
    
    if ((priv == NULL) || (priv->refs == 0) || !sg2002_check_i2c_base(priv->config->base))
        return false;

    while (timeout) {
        ic_en_reg->field.enable = state;
        if (ic_en_status_reg->field.ic_en == state)
            return true;

        up_udelay(25);
        timeout --;
    }

    return false;
}

/* Waits for bus busy */
static bool sg2002_i2c_wait_for_bb(volatile sg2002_i2c_reg_TypeDef *i2c) {
	uint8_t timeout = 0;
    volatile SG2002_IC_STATUS_Reg_TypeDef *ic_status_reg = NULL;

    if (i2c == NULL)
        return false;

    ic_status_reg = (volatile SG2002_IC_STATUS_Reg_TypeDef *)&(i2c->ic_status);

	while (ic_status_reg->field.st_mst_activity || !ic_status_reg->field.st_tfe) {
		/* Evaluate timeout */
		up_udelay(5);
        timeout++;

        /* exceed 1 ms */
        if (timeout > 200)
            return true;
	}

	return false;
}

static void sg2002_i2c_xfer_init_burst(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_CON_Reg_TypeDef *ic_con_reg = NULL;
    volatile SG2002_IC_TAR_Reg_TypeDef *ic_tar_reg = NULL;
    volatile SG2002_IC_CLR_INTR_Reg_TypeDef *ic_clr_intr_reg = NULL;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base) || (msgs == NULL))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_con_reg = (volatile SG2002_IC_CON_Reg_TypeDef *)&(i2c->ic_con);
    ic_tar_reg = (volatile SG2002_IC_TAR_Reg_TypeDef *)&(i2c->ic_tar);
    ic_clr_intr_reg = (volatile SG2002_IC_CLR_INTR_Reg_TypeDef *)&(i2c->ic_clr_intr);
    
    /* Disable the i2c */
    sg2002_i2c_enctl(priv, false);

    ic_con_reg->field.ic_10bitaddr_master = 0;

    /* Set the slave (target) address and enable 10-bit addressing mode if applicable */
    ic_tar_reg->field.ic_tar = msgs->addr;

    /* Enable the adapter */
    sg2002_i2c_enctl(priv, true);

    /* Clear and enable interrupts */
    ic_clr_intr_reg->val = SG2002_I2C_INTR_MASTER_MASK;
}

/* set bus speed */
static bool sg2002_set_bus_speed(struct sg2002_i2c_priv_s *priv, uint32_t speed) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_CON_Reg_TypeDef *ic_con_reg = NULL;
    uint8_t i2c_speed = 0;

    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return false;

    if (speed > I2C_SPEED_FAST) {
        i2c_speed = SG2002_I2C_BUS_MODE_HIGH;
    } else if ((speed <= I2C_SPEED_FAST) && (speed > I2C_SPEED_STANDARD)) {
        i2c_speed = SG2002_I2C_BUS_MODE_FAST;
    } else {
        i2c_speed = SG2002_I2C_BUS_MODE_STANDARD;
    }

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_con_reg = (volatile SG2002_IC_CON_Reg_TypeDef *)&(i2c->ic_con);

    /* to set speed cltr must be disabled */
    sg2002_i2c_enctl(priv, false);
    ic_con_reg->field.speed = 0;

    switch (i2c_speed) {
        case SG2002_I2C_BUS_MODE_HIGH: {
            volatile SG2002_IC_HS_SCL_HCNT_Reg_TypeDef *ic_hs_scl_hcnt_reg = (volatile SG2002_IC_HS_SCL_HCNT_Reg_TypeDef *)&(i2c->ic_hs_scl_hcnt);
            volatile SG2002_IC_HS_SCL_LCNT_Reg_TypeDef *ic_hs_scl_lcnt_reg = (volatile SG2002_IC_HS_SCL_LCNT_Reg_TypeDef *)&(i2c->ic_hs_scl_lcnt);

            ic_hs_scl_hcnt_reg->field.ic_hs_scl_hcnt = 6;
            ic_hs_scl_lcnt_reg->field.ic_hs_scl_lcnt = 8;
            break;
        }

        case SG2002_I2C_BUS_MODE_STANDARD: {
            volatile SG2002_IC_SS_SCL_HCNT_Reg_TypeDef *ic_ss_scl_hcnt_reg = (volatile SG2002_IC_SS_SCL_HCNT_Reg_TypeDef *)&(i2c->ic_ss_scl_hcnt);
            volatile SG2002_IC_SS_SCL_LCNT_Reg_TypeDef *ic_ss_scl_lcnt_reg = (volatile SG2002_IC_SS_SCL_LCNT_Reg_TypeDef *)&(i2c->ic_ss_scl_lcnt);

            ic_ss_scl_hcnt_reg->field.ic_ss_scl_hcnt = (uint16_t)(((SG2002_IC_CLK * SG2002_MIN_SS_SCL_HIGHTIME) / 1000) - 7);
            ic_ss_scl_lcnt_reg->field.ic_ss_scl_lcnt = (uint16_t)(((SG2002_IC_CLK * SG2002_MIN_SS_SCL_LOWTIME) / 1000) - 1);
            break;
        }

        case SG2002_I2C_BUS_MODE_FAST:
        default: {
            volatile SG2002_IC_FS_SCL_HCNT_Reg_TypeDef *ic_fs_scl_hcnt = (volatile SG2002_IC_FS_SCL_HCNT_Reg_TypeDef *)&(i2c->ic_fs_scl_hcnt);
            volatile SG2002_IC_FS_SCL_LCNT_Reg_TypeDef *ic_fs_scl_lcnt = (volatile SG2002_IC_FS_SCL_LCNT_Reg_TypeDef *)&(i2c->ic_fs_scl_lcnt);

            ic_fs_scl_hcnt->field.ic_fs_scl_hcnt = (uint16_t)(((SG2002_IC_CLK * SG2002_MIN_FS_SCL_HIGHTIME) / 1000) - 7);
            ic_fs_scl_lcnt->field.ic_fs_scl_lcnt = (uint16_t)(((SG2002_IC_CLK * SG2002_MIN_FS_SCL_LOWTIME) / 1000) - 1);
            break;
        }
    }
    
    ic_con_reg->field.speed = i2c_speed;

    /* Enable back i2c now speed set */
    sg2002_i2c_enctl(priv, true);
    return true;
}

static int sg2002_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count) {
	struct sg2002_i2c_priv_s *priv = NULL;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;

    if ((dev == NULL) || (msgs == NULL) || (count <= 0))
        return -1;

    priv = (struct sg2002_i2c_priv_s *)dev;

    if ((priv->config == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);

	//Need to acquire lock here
    if ((nxsem_wait(&priv->sem_excl) < 0) || !sg2002_i2c_wait_for_bb(i2c))
		return -1;

	sg2002_i2c_xfer_init_burst(priv, msgs);

    priv->status = 0;

    priv->dcnt = 0;
    priv->ptr = NULL;

	priv->msgv = msgs;
	priv->msgc = count;

    nxsem_post(&priv->sem_excl);
    return 0;
}

static void sg2002_i2c_sem_init(struct sg2002_i2c_priv_s *priv) {
    nxsem_init(&priv->sem_excl, 0, 1);

    /* This semaphore is used for signaling and, hence, should not have
    * priority inheritance enabled.
    */

    nxsem_init(&priv->sem_isr, 0, 0);
    nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
}

static void sg2002_i2c_dw_read(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num) {
    int rx_valid = 0;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_RXFLR_Reg_TypeDef *i2c_rxflr_reg = NULL;
    volatile SG2002_IC_DATA_CMD_Reg_TypeDef *i2c_data_cmd_reg = NULL;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base) || \
        (msgs == NULL) || (num == 0))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);

    for (uint8_t i = 0; i < num; i ++) {

		if (!(msgs[i].flags & I2C_M_READ))
			continue;

        priv->dcnt = msgs[i].length;
        priv->ptr = msgs[i].buffer;
        priv->flags = msgs[i].flags;

        /* get receive fifo level */
        i2c_rxflr_reg = (volatile SG2002_IC_RXFLR_Reg_TypeDef *)&(i2c->ic_rxflr);
        rx_valid = (i2c_rxflr_reg->field.rxflr >= priv->dcnt) ? priv->dcnt : i2c_rxflr_reg->field.rxflr;
        i2c_data_cmd_reg = (volatile SG2002_IC_DATA_CMD_Reg_TypeDef *)&(i2c->ic_data_cmd);

        for (; rx_valid > 0; rx_valid--) {
            *priv->ptr = i2c_data_cmd_reg->field.dat;

            priv->ptr ++;
            priv->dcnt --;
        }
    }

    priv->ptr = NULL;
    priv->dcnt = 0;
    priv->flags = 0;
}

/*
 * Initiate (and continue) low level master read/write transaction.
 * This function is only called from i2c_dw_isr, and pumping i2c_msg
 * messages into the tx buffer.  Even if the size of i2c_msg data is
 * longer than the size of the tx buffer, it handles everything.
 */
static void sg2002_i2c_dw_xfer_msg(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num) {
    uint16_t tx_mem = 0;
    uint16_t rx_mem = 0;
    uint32_t buf_len = 0;
    uint8_t *buf = NULL;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_TXFLR_Reg_TypeDef *i2c_txflr_reg = NULL;
    volatile SG2002_IC_RXFLR_Reg_TypeDef *i2c_rxflr_reg = NULL;
    volatile SG2002_IC_DATA_CMD_Reg_TypeDef *data_cmd_reg = NULL;
    volatile SG2002_IC_INTR_MASK_Reg_TypeDef *ic_intr_mask_reg = NULL;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base) || \
        (msgs == NULL) || (num == 0))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    i2c_txflr_reg = (volatile SG2002_IC_TXFLR_Reg_TypeDef *)&(i2c->ic_txflr);
    i2c_rxflr_reg = (volatile SG2002_IC_RXFLR_Reg_TypeDef *)&(i2c->ic_rxflr);
    data_cmd_reg = (volatile SG2002_IC_DATA_CMD_Reg_TypeDef *)&(i2c->ic_data_cmd);
    ic_intr_mask_reg = (volatile SG2002_IC_INTR_MASK_Reg_TypeDef *)&(i2c->ic_intr_mask);

	for (uint8_t i = 0; i < num; i++) {
        buf = msgs[i].buffer;
		buf_len = msgs[i].length;

		tx_mem = SG2002_I2C_FIFO_DEPTH - i2c_txflr_reg->field.txflr;
		rx_mem = SG2002_I2C_FIFO_DEPTH - i2c_rxflr_reg->field.rxflr;

        /* read and write fifo is not full */
        while (buf_len > 0 && tx_mem > 0 && rx_mem > 0) {
            /*
             * If IC_EMPTYFIFO_HOLD_MASTER_EN is set we must
             * manually set the stop bit. However, it cannot be
             * detected from the registers so we set it always
             * when writing/reading the last byte.
             */
			if ((i == num - 1) && (buf_len == 1)) {
                data_cmd_reg->field.stop = true;
            }

			if (msgs[i].flags & I2C_M_READ) {
                data_cmd_reg->field.cmd = SG2002_I2C_BUS_READ;
				rx_mem--;
			} else {
                data_cmd_reg->field.dat = *buf;
                buf ++;
            }

            buf_len--;
		}

        msgs[i].length = buf_len;
        msgs[i].buffer = buf;
	}

    ic_intr_mask_reg->field.m_rx_full = 1;
    ic_intr_mask_reg->field.m_tx_empty = 1;
    ic_intr_mask_reg->field.m_tx_abrt = 1;
    ic_intr_mask_reg->field.m_rx_done = 1;
    ic_intr_mask_reg->field.m_stop_det = 1;
    ic_intr_mask_reg->field.m_start_det = 1;
}

static uint32_t sg2002_i2c_dw_read_clear_intrbits(struct sg2002_i2c_priv_s *priv) {
    volatile sg2002_i2c_reg_TypeDef *i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    volatile SG2002_IC_INTR_STAT_Reg_TypeDef *ic_intr_stat_reg = (volatile SG2002_IC_INTR_STAT_Reg_TypeDef *)&(i2c->ic_intr_stat);
    uint32_t intr_state = 0;
    uint8_t tmp = 0;

    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    /*
     * The IC_INTR_STAT register just indicates "enabled" interrupts.
     * Ths unmasked raw version of interrupt status bits are available
     * in the IC_RAW_INTR_STAT register.
     *
     * The raw version might be useful for debugging purposes.
     */
    intr_state = ic_intr_stat_reg->val;

    /*
     * Do not use the IC_CLR_INTR register to clear interrupts, or
     * you'll miss some interrupts, triggered during the period from
     * read IC_INTR_STAT to read IC_CLR_INTR.
     *
     * Instead, use the separately-prepared IC_CLR_* registers.
     */
    if (intr_state & SG2002_BIT_I2C_INT_RX_UNDER) {
        volatile SG2002_IC_CLR_RX_UNDER_Reg_TypeDef *ic_clr_rx_under_reg = (volatile SG2002_IC_CLR_RX_UNDER_Reg_TypeDef *)&(i2c->ic_clr_rx_under); 
        tmp = ic_clr_rx_under_reg->field.clr_rx_under;
    }

    if (intr_state & SG2002_BIT_I2C_INT_RX_OVER) {
        volatile SG2002_IC_CLR_RX_OVER_Reg_TypeDef *ic_clr_rx_over_reg = (volatile SG2002_IC_CLR_RX_OVER_Reg_TypeDef *)&(i2c->ic_clr_rx_over);
        tmp = ic_clr_rx_over_reg->field.clr_rx_over;
    }

    if (intr_state & SG2002_BIT_I2C_INT_TX_OVER) {
        volatile SG2002_IC_CLR_TX_OVER_Reg_TypeDef *ic_clr_tx_over_reg = (volatile SG2002_IC_CLR_TX_OVER_Reg_TypeDef *)&(i2c->ic_clr_tx_over);
        tmp = ic_clr_tx_over_reg->field.clr_tx_over;
    }

    if (intr_state & SG2002_BIT_I2C_INT_RD_REQ) {
        volatile SG2002_IC_CLR_RD_REQ_Reg_TypeDef *ic_clr_rd_req_reg = (volatile SG2002_IC_CLR_RD_REQ_Reg_TypeDef *)&(i2c->ic_clr_rd_req);
        tmp = ic_clr_rd_req_reg->field.clr_rd_req;
    }

    if (intr_state & SG2002_BIT_I2C_INT_TX_ABRT) {
        /*
        * The IC_TX_ABRT_SOURCE register is cleared whenever
        * the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
        */
        volatile SG2002_IC_CLR_TX_ABRT_Reg_TypeDef *ic_clr_tx_abrt_reg = (volatile SG2002_IC_CLR_TX_ABRT_Reg_TypeDef *)&(i2c->ic_clr_tx_abrt);
        tmp = ic_clr_tx_abrt_reg->field.clr_tx_abrt;
    }

    if (intr_state & SG2002_BIT_I2C_INT_RX_DONE) {
        volatile SG2002_IC_CLR_RX_DONE_Reg_TypeDef *ic_clr_rx_done_reg = (volatile SG2002_IC_CLR_RX_DONE_Reg_TypeDef *)&(i2c->ic_clr_rx_done);
        tmp = ic_clr_rx_done_reg->field.clr_rx_done;
    }

    if (intr_state & SG2002_BIT_I2C_INT_ACTIVITY) {
        volatile SG2002_IC_CLR_ACTIVITY_Reg_TypeDef *ic_clr_activity_reg = (volatile SG2002_IC_CLR_ACTIVITY_Reg_TypeDef *)&(i2c->ic_clr_activity);
        tmp = ic_clr_activity_reg->field.clr_activity;
    }

    if (intr_state & SG2002_BIT_I2C_INT_STOP_DET) {
        volatile SG2002_IC_CLR_STOP_DET_Reg_TypeDef *ic_clr_stop_det_reg = (volatile SG2002_IC_CLR_STOP_DET_Reg_TypeDef *)&(i2c->ic_clr_stop_det);
        tmp = ic_clr_stop_det_reg->field.clr_stop_det;
    }

    if (intr_state & SG2002_BIT_I2C_INT_START_DET) {
        volatile SG2002_IC_CLR_START_DET_Reg_TypeDef *ic_clr_start_det_reg = (volatile SG2002_IC_CLR_START_DET_Reg_TypeDef *)&(i2c->ic_clr_start_det);
        tmp = ic_clr_start_det_reg->field.clr_start_det;
    }

    if (intr_state & SG2002_BIT_I2C_INT_GEN_ALL) {
        volatile SG2002_IC_CLR_GEN_CALL_Reg_TypeDef *ic_clr_gen_call_reg = (volatile SG2002_IC_CLR_GEN_CALL_Reg_TypeDef *)&(i2c->ic_clr_gen_call);
        tmp = ic_clr_gen_call_reg->field.clr_gen_call;
    }

    return intr_state;
}

static int sg2002_i2c_irq_handle(int irq, void *context, void *arg) {
	uint32_t state = 0;
    struct sg2002_i2c_priv_s *priv = (struct sg2002_i2c_priv_s *)arg;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_ENABLE_Reg_TypeDef *ic_enable_reg = NULL;
    volatile SG2002_IC_INTR_STAT_Reg_TypeDef *ic_intr_stat_reg = NULL;
    volatile SG2002_IC_INTR_MASK_Reg_TypeDef *ic_intr_mask_reg = NULL;

    if ((priv == NULL) || (priv->refs == 0) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    /* check i2c and interrupt state */
    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_enable_reg = (volatile SG2002_IC_ENABLE_Reg_TypeDef *)&(i2c->ic_enable);
    ic_intr_stat_reg = (volatile SG2002_IC_INTR_STAT_Reg_TypeDef *)&(i2c->ic_intr_stat);
    ic_intr_mask_reg = (volatile SG2002_IC_INTR_MASK_Reg_TypeDef *)&(i2c->ic_intr_mask);

     /* if i2c is disabled or no interrupt is pending, return */
    if (!ic_enable_reg->field.enable || !(ic_intr_stat_reg->val & SG2002_BIT_I2C_INT_ACTIVITY))
        return -1;

    state = sg2002_i2c_dw_read_clear_intrbits(priv);

	if (state & SG2002_BIT_I2C_INT_TX_ABRT) {
		/*
		 * Anytime TX_ABRT is set, the contents of the tx/rx
		 * buffers are flushed. Make sure to skip them.
		 */
        ic_intr_mask_reg->val = 0;
		goto tx_aborted;
	}

	if (state & SG2002_BIT_I2C_INT_RX_FULL) {
        if (priv->msgv) {
            sg2002_i2c_dw_read(priv, priv->msgv, priv->msgc);
        } else {
            sg2002_i2c_enctl(priv, false);
        }
	}

    if (state & SG2002_BIT_I2C_INT_TX_EMPTY) {
        if (priv->msgv) {
            sg2002_i2c_dw_xfer_msg(priv, priv->msgv, priv->msgc);
        } else {
			sg2002_i2c_enctl(priv, false);
        }
	}

tx_aborted:

    if (state & (SG2002_BIT_I2C_INT_TX_ABRT | SG2002_BIT_I2C_INT_STOP_DET))
        sg2002_i2c_enctl(priv, false);

    return 0;
}

static int sg2002_i2c_init(struct sg2002_i2c_priv_s *priv) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    volatile SG2002_IC_CON_Reg_TypeDef *ic_con_reg = NULL;
    volatile SG2002_IC_RX_TL_Reg_TypeDef *ic_rx_tl_reg = NULL;
    volatile SG2002_IC_TX_TL_Reg_TypeDef *ic_tx_tl_reg = NULL;
    volatile SG2002_IC_INTR_MASK_Reg_TypeDef *ic_intr_mask_reg = NULL;
    
    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;
    
    if (!sg2002_i2c_enctl(priv, false))
        return -1;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_con_reg = (volatile SG2002_IC_CON_Reg_TypeDef *)&(i2c->ic_con);
    ic_rx_tl_reg = (volatile SG2002_IC_RX_TL_Reg_TypeDef *)&(i2c->ic_rx_tl);
    ic_tx_tl_reg = (volatile SG2002_IC_TX_TL_Reg_TypeDef *)&(i2c->ic_tx_tl);
    ic_intr_mask_reg = (volatile SG2002_IC_INTR_MASK_Reg_TypeDef *)&(i2c->ic_intr_mask);

    ic_con_reg->val = 0;
    ic_rx_tl_reg->val = 0;
    ic_tx_tl_reg->val = 0;
    ic_intr_mask_reg->val = 0;

    ic_con_reg->field.master_mode = true;
    ic_con_reg->field.speed = SG2002_I2C_BUS_MODE_FAST;
    ic_con_reg->field.ic_slave_disable = true;
    ic_con_reg->field.ic_restart_en = false;

    ic_rx_tl_reg->field.rx_tl = 0;
    ic_tx_tl_reg->field.tx_tl = 0;

    if (!sg2002_set_bus_speed(priv, I2C_SPEED_FAST) | \
        !sg2002_i2c_enctl(priv, false))
        return -1;

    return 0;
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

    irq_attach(priv->config->irq, sg2002_i2c_irq_handle, priv);
    up_enable_irq(priv->config->irq);

    flags = enter_critical_section();

    if ((volatile int)priv->refs++ == 0) {
        sg2002_i2c_sem_init(priv);
        sg2002_i2c_init(priv);
    }

    leave_critical_section(flags);
    return (struct i2c_master_s *)priv;
}

/* developping */
int sg2002_i2cbus_uninitialize(struct i2c_master_s *dev) {

    return -1;
}

