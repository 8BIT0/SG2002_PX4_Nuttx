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

// #define SG2002_I2C_TraceOut(fmt, ...)       sg2002_trace_dirout(fmt, ##__VA_ARGS__)
#define SG2002_I2C_TraceOut(fmt, ...)

#define SG2002_Priv_2_BaseReg(x)            (volatile sg2002_i2c_reg_TypeDef *)((uintptr_t)(x));

#define SG2002_Enable_Reg                   volatile SG2002_IC_ENABLE_Reg_TypeDef
#define SG2002_EnableStatus_Reg             volatile SG2002_IC_ENABLE_STATUS_Reg_TypeDef
#define SG2002_Status_Reg                   volatile SG2002_IC_STATUS_Reg_TypeDef
#define SG2002_Con_Reg                      volatile SG2002_IC_CON_Reg_TypeDef
#define SG2002_Hs_Scl_Hcnt_Reg              volatile SG2002_IC_HS_SCL_HCNT_Reg_TypeDef
#define SG2002_Hs_Scl_Lcnt_Reg              volatile SG2002_IC_HS_SCL_LCNT_Reg_TypeDef
#define SG2002_Ss_Scl_Hcnt_Reg              volatile SG2002_IC_SS_SCL_HCNT_Reg_TypeDef
#define SG2002_Ss_Scl_Lcnt_Reg              volatile SG2002_IC_SS_SCL_LCNT_Reg_TypeDef
#define SG2002_Fs_Scl_Hcnt_Reg              volatile SG2002_IC_FS_SCL_HCNT_Reg_TypeDef
#define SG2002_Fs_Scl_Lcnt_Reg              volatile SG2002_IC_FS_SCL_LCNT_Reg_TypeDef
#define SG2002_Tar_Reg                      volatile SG2002_IC_TAR_Reg_TypeDef
#define SG2002_Data_Cmd_Reg                 volatile SG2002_IC_DATA_CMD_Reg_TypeDef
#define SG2002_Raw_Intr_Stat_Reg            volatile SG2002_IC_RAW_INTR_STAT_Reg_TypeDef
#define SG2002_Clr_Stop_Det_Reg             volatile SG2002_IC_CLR_STOP_DET_Reg_TypeDef
#define SG2002_Clr_Intr_Reg                 volatile SG2002_IC_CLR_INTR_Reg_TypeDef
#define SG2002_RxFlr_Reg                    volatile SG2002_IC_RXFLR_Reg_TypeDef
#define SG2002_TxFlr_Reg                    volatile SG2002_IC_TXFLR_Reg_TypeDef
#define SG2002_Intr_Mask_Reg                volatile SG2002_IC_INTR_MASK_Reg_TypeDef
#define SG2002_Intr_Stat_Reg                volatile SG2002_IC_INTR_STAT_Reg_TypeDef
#define SG2002_Rx_Tl_Reg                    volatile SG2002_IC_RX_TL_Reg_TypeDef
#define SG2002_Tx_Tl_Reg                    volatile SG2002_IC_TX_TL_Reg_TypeDef
#define SG2002_Clr_Rx_Under_Reg             volatile SG2002_IC_CLR_RX_UNDER_Reg_TypeDef
#define SG2002_Clr_Rx_Over_Reg              volatile SG2002_IC_CLR_RX_OVER_Reg_TypeDef
#define SG2002_Clr_Tx_Over_Reg              volatile SG2002_IC_CLR_TX_OVER_Reg_TypeDef
#define SG2002_Clr_Rd_Req_Reg               volatile SG2002_IC_CLR_RD_REQ_Reg_TypeDef
#define SG2002_Clr_Tx_Abrt_Reg              volatile SG2002_IC_CLR_TX_ABRT_Reg_TypeDef
#define SG2002_Clr_Rx_Done_Reg              volatile SG2002_IC_CLR_RX_DONE_Reg_TypeDef
#define SG2002_Clr_Activity_Reg             volatile SG2002_IC_CLR_ACTIVITY_Reg_TypeDef
#define SG2002_Clr_Start_Det_Reg            volatile SG2002_IC_CLR_START_DET_Reg_TypeDef
#define SG2002_Clr_Gen_Call_Reg             volatile SG2002_IC_CLR_GEN_CALL_Reg_TypeDef
#define SG2002_Sda_Hold_Reg                 volatile SG2002_IC_SDA_HOLD_Reg_TypeDef
#define SG2002_Sda_Setup_Reg                volatile SG2002_IC_SDA_SETUP_Reg_TypeDef
#define SG2002_Fs_Spklen_Reg                volatile SG2002_IC_FS_SPKLEN_Reg_TypeDef

#define To_SG2002_Enable_Reg_Ptr(x)         ((SG2002_Enable_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_EnableStatus_Reg_Ptr(x)   ((SG2002_EnableStatus_Reg   *)((uintptr_t)&(x)))
#define To_SG2002_Status_Reg_Ptr(x)         ((SG2002_Status_Reg         *)((uintptr_t)&(x)))
#define To_SG2002_Con_Reg_Ptr(x)            ((SG2002_Con_Reg            *)((uintptr_t)&(x)))
#define To_SG2002_Hs_Scl_Hcnt_Reg_Ptr(x)    ((SG2002_Hs_Scl_Hcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Hs_Scl_Lcnt_Reg_Ptr(x)    ((SG2002_Hs_Scl_Lcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Ss_Scl_Hcnt_Reg_Ptr(x)    ((SG2002_Ss_Scl_Hcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Ss_Scl_Lcnt_Reg_Ptr(x)    ((SG2002_Ss_Scl_Lcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Fs_Scl_Hcnt_Reg_Ptr(x)    ((SG2002_Fs_Scl_Hcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Fs_Scl_Lcnt_Reg_Ptr(x)    ((SG2002_Fs_Scl_Lcnt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Tar_Reg_Ptr(x)            ((SG2002_Tar_Reg            *)((uintptr_t)&(x)))
#define To_SG2002_Data_Cmd_Reg_Ptr(x)       ((SG2002_Data_Cmd_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_Raw_Intr_Stat_Reg_Ptr(x)  ((SG2002_Raw_Intr_Stat_Reg  *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Stop_Det_Reg_Ptr(x)   ((SG2002_Clr_Stop_Det_Reg   *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Intr_Reg_Ptr(x)       ((SG2002_Clr_Intr_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_RxFlr_Reg_Ptr(x)          ((SG2002_RxFlr_Reg          *)((uintptr_t)&(x)))
#define To_SG2002_TxFlr_Reg_Ptr(x)          ((SG2002_TxFlr_Reg          *)((uintptr_t)&(x)))
#define To_SG2002_Intr_Mask_Reg_Ptr(x)      ((SG2002_Intr_Mask_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Intr_Stat_Reg_Ptr(x)      ((SG2002_Intr_Stat_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Rx_Tl_Reg_Ptr(x)          ((SG2002_Rx_Tl_Reg          *)((uintptr_t)&(x)))
#define To_SG2002_Tx_Tl_Reg_Ptr(x)          ((SG2002_Tx_Tl_Reg          *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Rx_Under_Reg_Ptr(x)   ((SG2002_Clr_Rx_Under_Reg   *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Rx_Over_Reg_Ptr(x)    ((SG2002_Clr_Rx_Over_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Tx_Over_Reg_Ptr(x)    ((SG2002_Clr_Tx_Over_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Rd_Req_Reg_Ptr(x)     ((SG2002_Clr_Rd_Req_Reg     *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Tx_Abrt_Reg_Ptr(x)    ((SG2002_Clr_Tx_Abrt_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Rx_Done_Reg_Ptr(x)    ((SG2002_Clr_Rx_Done_Reg    *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Activity_Reg_Ptr(x)   ((SG2002_Clr_Activity_Reg   *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Start_Det_Reg_Ptr(x)  ((SG2002_Clr_Start_Det_Reg  *)((uintptr_t)&(x)))
#define To_SG2002_Clr_Gen_Call_Reg_Ptr(x)   ((SG2002_Clr_Gen_Call_Reg   *)((uintptr_t)&(x)))
#define To_SG2002_Sda_Hold_Reg_Ptr(x)       ((SG2002_Sda_Hold_Reg       *)((uintptr_t)&(x)))
#define To_SG2002_Sda_Setup_Reg_Ptr(x)      ((SG2002_Sda_Setup_Reg      *)((uintptr_t)&(x)))
#define To_SG2002_Fs_Spklen_Reg_Ptr(x)      ((SG2002_Fs_Spklen_Reg      *)((uintptr_t)&(x)))

#define reg_t volatile uint32_t

typedef struct {
	reg_t ic_con;               /* 0x00 */
	reg_t ic_tar;               /* 0x04 */
	reg_t ic_sar;               /* 0x08 */
	reg_t ic_hs_maddr;          /* 0x0c */
	reg_t ic_data_cmd;          /* 0x10 */
	reg_t ic_ss_scl_hcnt;       /* 0x14 */
	reg_t ic_ss_scl_lcnt;       /* 0x18 */
	reg_t ic_fs_scl_hcnt;       /* 0x1c */
	reg_t ic_fs_scl_lcnt;       /* 0x20 */
	reg_t ic_hs_scl_hcnt;       /* 0x24 */
	reg_t ic_hs_scl_lcnt;       /* 0x28 */
	reg_t ic_intr_stat;         /* 0x2c */
	reg_t ic_intr_mask;         /* 0x30 */
	reg_t ic_raw_intr_stat;     /* 0x34 */
	reg_t ic_rx_tl;             /* 0x38 */
	reg_t ic_tx_tl;             /* 0x3c */
	reg_t ic_clr_intr;          /* 0x40 */
	reg_t ic_clr_rx_under;      /* 0x44 */
	reg_t ic_clr_rx_over;       /* 0x48 */
	reg_t ic_clr_tx_over;       /* 0x4c */
	reg_t ic_clr_rd_req;        /* 0x50 */
	reg_t ic_clr_tx_abrt;       /* 0x54 */
	reg_t ic_clr_rx_done;       /* 0x58 */
	reg_t ic_clr_activity;      /* 0x5c */
	reg_t ic_clr_stop_det;	    /* 0x60 */
	reg_t ic_clr_start_det;     /* 0x64 */
	reg_t ic_clr_gen_call;      /* 0x68 */
	reg_t ic_enable;            /* 0x6c */
	reg_t ic_status;            /* 0x70 */
	reg_t ic_txflr;             /* 0x74 */
	reg_t ic_rxflr;	            /* 0x78 */
	reg_t ic_sda_hold;          /* 0x7c */
	reg_t ic_tx_abrt_source;	/* 0x80 */
	reg_t ic_slv_dat_nack_only; /* 0x84 */
	reg_t ic_dma_cr; 		    /* 0x88 */
	reg_t ic_dma_tdlr;	        /* 0x8c */
	reg_t ic_dma_rdlr;	        /* 0x90 */
	reg_t ic_sda_setup;	        /* 0x94 */
	reg_t ic_ack_general_call;  /* 0x98 */
	reg_t ic_enable_status;	    /* 0x9c */
	reg_t ic_fs_spklen;	        /* 0xa0 */
	reg_t ic_hs_spklen;	        /* 0xa4 */
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
    bool wait_irq;
    uint16_t msg_index;
    uint16_t rx_index;
};

static bool sg2002_check_i2c_base(uint32_t base);
static bool sg2002_i2c_enctl(struct sg2002_i2c_priv_s *priv, bool state);
static bool sg2002_i2c_wait_for_bb(volatile sg2002_i2c_reg_TypeDef *i2c);
static int sg2002_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
static void sg2002_i2c_dw_read(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num);
static uint32_t sg2002_i2c_dw_read_clear_intrbits(struct sg2002_i2c_priv_s *priv);
static int sg2002_i2c_irq_handle(int irq, void *context, void *arg);
static int sg2002_i2c_init(struct sg2002_i2c_priv_s *priv);
static bool sg2002_set_bus_speed(struct sg2002_i2c_priv_s *priv, uint32_t speed);

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
    .wait_irq   = false,
    .msg_index  = 0,
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
    .wait_irq   = false,
    .msg_index  = 0,
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
    SG2002_Enable_Reg *ic_en_reg = To_SG2002_Enable_Reg_Ptr(i2c->ic_enable);
    SG2002_EnableStatus_Reg *ic_en_status_reg = To_SG2002_EnableStatus_Reg_Ptr(i2c->ic_enable_status);
    
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
	uint32_t timeout = 0;
    SG2002_Status_Reg *ic_status_reg = NULL;
    
    if (i2c == NULL)
        return false;

    ic_status_reg = To_SG2002_Status_Reg_Ptr(i2c->ic_status);
    while (ic_status_reg->field.st_mst_activity || !ic_status_reg->field.st_tfe) {
        /* Evaluate timeout */
        timeout++;
        up_udelay(5);

        if (timeout > 200) {
            SG2002_I2C_TraceOut("i2c busy\n");
            return true;
        }
	}

	return false;
}

/* set bus speed */
static bool sg2002_set_bus_speed(struct sg2002_i2c_priv_s *priv, uint32_t speed) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    uint8_t i2c_speed = 0;
    SG2002_Con_Reg ic_con_reg;
    SG2002_Ss_Scl_Hcnt_Reg ic_ss_scl_hcnt_reg;
    SG2002_Ss_Scl_Lcnt_Reg ic_ss_scl_lcnt_reg;
    SG2002_Fs_Scl_Hcnt_Reg ic_fs_scl_hcnt_reg;
    SG2002_Fs_Scl_Lcnt_Reg ic_fs_scl_lcnt_reg;
    SG2002_Sda_Hold_Reg ic_sda_hold_reg;
    SG2002_Sda_Setup_Reg ic_sda_setup_reg;
    SG2002_Fs_Spklen_Reg ic_fs_spklen_reg;

    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return false;

    if ((speed <= I2C_SPEED_FAST) && (speed > I2C_SPEED_STANDARD)) {
        i2c_speed = SG2002_I2C_BUS_MODE_FAST;
    } else {
        i2c_speed = SG2002_I2C_BUS_MODE_STANDARD;
    }

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);

    ic_sda_hold_reg.val = To_SG2002_Sda_Hold_Reg_Ptr(i2c->ic_sda_hold)->val;
    ic_sda_setup_reg.val = To_SG2002_Sda_Setup_Reg_Ptr(i2c->ic_sda_setup)->val;
    ic_fs_spklen_reg.val = To_SG2002_Fs_Spklen_Reg_Ptr(i2c->ic_fs_spklen)->val;

    ic_con_reg.val = To_SG2002_Con_Reg_Ptr(i2c->ic_con)->val;
    ic_ss_scl_hcnt_reg.val = 0;
    ic_ss_scl_lcnt_reg.val = 0;
    ic_fs_scl_hcnt_reg.val = 0;
    ic_fs_scl_lcnt_reg.val = 0;

    /* to set speed cltr must be disabled */
    sg2002_i2c_enctl(priv, false);

    switch (i2c_speed) {
        case SG2002_I2C_BUS_MODE_STANDARD: {
            ic_ss_scl_hcnt_reg.field.ic_ss_scl_hcnt = 115;
            ic_ss_scl_lcnt_reg.field.ic_ss_scl_lcnt = 135;
            ic_fs_scl_hcnt_reg.field.ic_fs_scl_hcnt = 21;
            ic_fs_scl_lcnt_reg.field.ic_fs_scl_lcnt = 42;

            ic_sda_hold_reg.field.ic_sda_hold = 1;
            ic_sda_setup_reg.val = 6;
            ic_fs_spklen_reg.field.ic_fs_spklen = 2;
            break;
        }

        case SG2002_I2C_BUS_MODE_FAST:
        default: {
            ic_ss_scl_hcnt_reg.field.ic_ss_scl_hcnt = 460;
            ic_ss_scl_lcnt_reg.field.ic_ss_scl_lcnt = 540;
            ic_fs_scl_hcnt_reg.field.ic_fs_scl_hcnt = 90;
            ic_fs_scl_lcnt_reg.field.ic_fs_scl_lcnt = 160;

            ic_sda_hold_reg.field.ic_sda_hold = 1;
            ic_sda_setup_reg.val = 25;
            ic_fs_spklen_reg.field.ic_fs_spklen = 5;
            break;
        }
    }

    ic_con_reg.field.speed = i2c_speed;
    
    To_SG2002_Con_Reg_Ptr(i2c->ic_con)->val = ic_con_reg.val;

    To_SG2002_Ss_Scl_Hcnt_Reg_Ptr(i2c->ic_ss_scl_hcnt)->val = ic_ss_scl_hcnt_reg.val;
    To_SG2002_Ss_Scl_Lcnt_Reg_Ptr(i2c->ic_ss_scl_lcnt)->val = ic_ss_scl_lcnt_reg.val;
    To_SG2002_Fs_Scl_Hcnt_Reg_Ptr(i2c->ic_fs_scl_hcnt)->val = ic_fs_scl_hcnt_reg.val;
    To_SG2002_Fs_Scl_Lcnt_Reg_Ptr(i2c->ic_fs_scl_lcnt)->val = ic_fs_scl_lcnt_reg.val;
    
    To_SG2002_Sda_Hold_Reg_Ptr(i2c->ic_sda_hold)->val = ic_sda_hold_reg.val;
    To_SG2002_Sda_Setup_Reg_Ptr(i2c->ic_sda_setup)->val = ic_sda_setup_reg.val;
    To_SG2002_Fs_Spklen_Reg_Ptr(i2c->ic_fs_spklen)->val = ic_fs_spklen_reg.val;

    /* Enable back i2c now speed set */
    sg2002_i2c_enctl(priv, true);
    return true;
}

static void sg2002_i2c_sem_init(struct sg2002_i2c_priv_s *priv) {
    nxsem_init(&priv->sem_excl, 0, 1);

    /* This semaphore is used for signaling and, hence, should not have
    * priority inheritance enabled.
    */

    nxsem_init(&priv->sem_isr, 0, 0);
    nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
}

static void sg2002_i2c_xfer_init_burst(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msg) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    uint32_t tmp = 0;
    uint16_t time_out = 100;
    SG2002_Con_Reg ic_con_reg;
    SG2002_Tar_Reg ic_tar_reg;
    SG2002_Intr_Mask_Reg ic_intr_mask_reg;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_con_reg.val = To_SG2002_Con_Reg_Ptr(i2c->ic_con)->val;
    ic_tar_reg.val = To_SG2002_Tar_Reg_Ptr(i2c->ic_tar)->val;

    /* Disable the i2c */
    sg2002_i2c_enctl(priv, false);

    ic_con_reg.field.ic_10bitaddr_master = 0;

    To_SG2002_Con_Reg_Ptr(i2c->ic_con)->val = ic_con_reg.val;
    To_SG2002_Intr_Mask_Reg_Ptr(i2c->ic_intr_mask)->val = 0;

    /* Set the slave (target) address and enable 10-bit addressing mode if applicable */
    ic_tar_reg.field.ic_tar = msg->addr;
    To_SG2002_Tar_Reg_Ptr(i2c->ic_tar)->val = ic_tar_reg.val;

    if (To_SG2002_Tar_Reg_Ptr(i2c->ic_tar)->field.ic_tar != msg->addr)
        return;

    tmp = To_SG2002_Clr_Intr_Reg_Ptr(i2c->ic_clr_intr)->val;
    while (tmp) {
        up_udelay(25);
        time_out --;
        
        if (time_out == 0)
            return;

        tmp = To_SG2002_Clr_Intr_Reg_Ptr(i2c->ic_clr_intr)->val;
    }
    priv->wait_irq = true;

    /* Clear and enable interrupts */
    ic_intr_mask_reg.val = 0;
    ic_intr_mask_reg.field.m_rx_full = true;
    ic_intr_mask_reg.field.m_tx_empty = true;
    ic_intr_mask_reg.field.m_stop_det = true;
    ic_intr_mask_reg.field.m_tx_abrt = true;
    To_SG2002_Intr_Mask_Reg_Ptr(i2c->ic_intr_mask)->val = ic_intr_mask_reg.val;

    /* Enable the adapter */
    sg2002_i2c_enctl(priv, true);
}

static int sg2002_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count) {
    struct sg2002_i2c_priv_s *priv = NULL;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    uint32_t timeout = 200;

    if ((dev == NULL) || (msgs == NULL) || (count <= 0))
        return -1;

    priv = (struct sg2002_i2c_priv_s *)dev;

    if ((priv->config == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    
    // if ((nxsem_wait(&priv->sem_excl) < 0) | sg2002_i2c_wait_for_bb(i2c))
    if (sg2002_i2c_wait_for_bb(i2c))
        return -1;

    priv->dcnt = 0;
    priv->ptr = NULL;

	priv->msgv = msgs;
	priv->msgc = count;
    priv->msg_index = 0;
    priv->rx_index = 0;

    SG2002_I2C_TraceOut("wait i2c bus transmit finish\n");
	sg2002_i2c_xfer_init_burst(priv, msgs);
    
    /* 1ms timeout */
    while (priv->wait_irq && (timeout != 0)) {
        up_udelay(5);
        timeout --;
    }

    // nxsem_post(&priv->sem_excl);
    return 0;
}

static void sg2002_i2c_dw_read(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    SG2002_Data_Cmd_Reg data_cmd_tmp;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base) || \
        (msgs == NULL) || (num == 0))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    priv->dcnt = msgs[priv->msg_index].length;
    priv->ptr = msgs[priv->msg_index].buffer;
    priv->flags = msgs[priv->msg_index].flags;

    uint32_t rxflr = To_SG2002_RxFlr_Reg_Ptr(i2c->ic_rxflr)->val;
    SG2002_I2C_TraceOut("rxflr %d\n", rxflr);

    for (uint8_t i = 0; i < rxflr; i ++) {
        while (!To_SG2002_Status_Reg_Ptr(i2c->ic_status)->field.st_rfne);
        priv->ptr[priv->rx_index] = To_SG2002_Data_Cmd_Reg_Ptr(i2c->ic_data_cmd)->field.dat;
        SG2002_I2C_TraceOut("index %d data 0x%02x\n", i, priv->ptr[priv->rx_index]);
        priv->rx_index ++;
    }

    if (priv->rx_index == priv->dcnt) {
        priv->ptr = NULL;
        priv->dcnt = 0;
        priv->flags = 0;
        priv->rx_index = 0;
    }

    data_cmd_tmp.val = 0;
    data_cmd_tmp.field.stop = true;

    To_SG2002_Data_Cmd_Reg_Ptr(i2c->ic_data_cmd)->val = data_cmd_tmp.val;
}

/*
 * Initiate (and continue) low level master read/write transaction.
 * This function is only called from i2c_dw_isr, and pumping i2c_msg
 * messages into the tx buffer.  Even if the size of i2c_msg data is
 * longer than the size of the tx buffer, it handles everything.
 */
static void sg2002_i2c_dw_xfer_msg(struct sg2002_i2c_priv_s *priv, struct i2c_msg_s *msgs, uint32_t num) {
    uint32_t buf_len = 0;
    uint8_t *buf = NULL;
    int8_t flags = 0;
    uint8_t i = 0;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    SG2002_Data_Cmd_Reg cmd_tmp;
    SG2002_Intr_Mask_Reg intr_mask_tmp;

    if ((priv == NULL) || (priv->config == NULL) || \
        !sg2002_check_i2c_base(priv->config->base) || \
        (msgs == NULL) || (num == 0))
        return;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    
    intr_mask_tmp.val = 0;
    intr_mask_tmp.field.m_rx_full = true;
    intr_mask_tmp.field.m_tx_abrt = true;
    intr_mask_tmp.field.m_stop_det = true;

    for (i = 0; i < num; i++) {
        buf = msgs[i].buffer;
		buf_len = msgs[i].length;
        flags = msgs[i].flags;
        cmd_tmp.val = 0;

        for (int8_t j = 0; j < buf_len; j ++) {
            SG2002_I2C_TraceOut("index %d\n", j);

            if (j == (buf_len - 1) && !(flags & I2C_M_NOSTOP)) {
                SG2002_I2C_TraceOut("tx set stop\n");
                cmd_tmp.field.stop = true;
            }

            cmd_tmp.field.dat = buf[j];
            if (flags & I2C_M_READ) {
                priv->msg_index = i;

                if (j == 0) {
                    SG2002_I2C_TraceOut("set restart\n");
                    cmd_tmp.field.restart = true;
                    cmd_tmp.field.dat = 0;
                }

                cmd_tmp.field.cmd = SG2002_I2C_BUS_READ;
            }

            To_SG2002_Data_Cmd_Reg_Ptr(i2c->ic_data_cmd)->val = cmd_tmp.val;
            while (!To_SG2002_Status_Reg_Ptr(i2c->ic_status)->field.st_tfe);
            
            cmd_tmp.val = 0;
		}
    }

    intr_mask_tmp.field.m_tx_empty = !(i == num);
    To_SG2002_Intr_Mask_Reg_Ptr(i2c->ic_intr_mask)->val = intr_mask_tmp.val;
}

static uint32_t sg2002_i2c_dw_read_clear_intrbits(struct sg2002_i2c_priv_s *priv) {
    volatile sg2002_i2c_reg_TypeDef *i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    uint32_t intr_state = 0;
    uint32_t tmp = 0;

    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    intr_state = To_SG2002_Intr_Stat_Reg_Ptr(i2c->ic_intr_stat)->val;

    if (intr_state & SG2002_BIT_I2C_INT_RX_UNDER)
        tmp = To_SG2002_Clr_Rx_Under_Reg_Ptr(i2c->ic_clr_rx_under)->val;

    if (intr_state & SG2002_BIT_I2C_INT_RX_OVER)
        tmp = To_SG2002_Clr_Rx_Over_Reg_Ptr(i2c->ic_clr_rx_over)->val;

    if (intr_state & SG2002_BIT_I2C_INT_TX_OVER)
        tmp = To_SG2002_Clr_Tx_Over_Reg_Ptr(i2c->ic_clr_tx_over)->val;

    if (intr_state & SG2002_BIT_I2C_INT_RD_REQ)
        tmp = To_SG2002_Clr_Rd_Req_Reg_Ptr(i2c->ic_clr_rd_req)->val;

    if (intr_state & SG2002_BIT_I2C_INT_TX_ABRT)
        tmp = To_SG2002_Clr_Tx_Abrt_Reg_Ptr(i2c->ic_clr_tx_abrt)->val;

    if (intr_state & SG2002_BIT_I2C_INT_RX_DONE)
        tmp = To_SG2002_Clr_Rx_Done_Reg_Ptr(i2c->ic_clr_rx_done)->val;

    if (intr_state & SG2002_BIT_I2C_INT_ACTIVITY)
        tmp = To_SG2002_Clr_Activity_Reg_Ptr(i2c->ic_clr_activity)->val;

    if (intr_state & SG2002_BIT_I2C_INT_START_DET)
        tmp = To_SG2002_Clr_Start_Det_Reg_Ptr(i2c->ic_clr_start_det)->val;

    if (intr_state & SG2002_BIT_I2C_INT_STOP_DET)
        tmp = To_SG2002_Clr_Stop_Det_Reg_Ptr(i2c->ic_clr_stop_det)->val;

    if (intr_state & SG2002_BIT_I2C_INT_GEN_ALL)
        tmp = To_SG2002_Clr_Gen_Call_Reg_Ptr(i2c->ic_clr_gen_call)->val;

    return intr_state;
}

static int sg2002_i2c_irq_handle(int irq, void *context, void *arg) {
	uint32_t state = 0;
    struct sg2002_i2c_priv_s *priv = (struct sg2002_i2c_priv_s *)arg;
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    SG2002_Enable_Reg *ic_enable_reg = NULL;
    SG2002_Intr_Stat_Reg *ic_intr_stat_reg = NULL;
    SG2002_Intr_Mask_Reg *ic_intr_mask_reg = NULL;
    SG2002_Intr_Mask_Reg mask;

    if ((priv == NULL) || (priv->refs == 0) || !sg2002_check_i2c_base(priv->config->base))
        return -1;

    /* check i2c and interrupt state */
    i2c = SG2002_Priv_2_BaseReg(priv->config->base);
    ic_enable_reg = To_SG2002_Enable_Reg_Ptr(i2c->ic_enable);
    ic_intr_stat_reg = To_SG2002_Intr_Stat_Reg_Ptr(i2c->ic_intr_stat);
    ic_intr_mask_reg = To_SG2002_Intr_Mask_Reg_Ptr(i2c->ic_intr_mask);

     /* if i2c is disabled or no interrupt is pending, return */
    if (!ic_enable_reg->field.enable || !(ic_intr_stat_reg->val & ~SG2002_BIT_I2C_INT_ACTIVITY))
        return -1;

    state = sg2002_i2c_dw_read_clear_intrbits(priv);

	if (state & SG2002_BIT_I2C_INT_TX_ABRT) {
		/*
		 * Anytime TX_ABRT is set, the contents of the tx/rx
		 * buffers are flushed. Make sure to skip them.
		 */
        SG2002_I2C_TraceOut("i2c abrt\n");
        ic_intr_mask_reg->val = 0;
		goto tx_aborted;
	}

    if (state & SG2002_BIT_I2C_INT_RX_FULL) {
        SG2002_I2C_TraceOut("i2c rx\n");
        if (priv->msgv) {
            sg2002_i2c_dw_read(priv, priv->msgv, priv->msgc);
        } else
            priv->wait_irq = false;
	}

    if (state & SG2002_BIT_I2C_INT_TX_EMPTY) {
        SG2002_I2C_TraceOut("i2c tx\n");
        if (priv->msgv) {
            sg2002_i2c_dw_xfer_msg(priv, priv->msgv, priv->msgc);
        } else
            priv->wait_irq = false;
	}

tx_aborted:
    
    if (state & (SG2002_BIT_I2C_INT_TX_ABRT | SG2002_BIT_I2C_INT_STOP_DET)) {
        priv->wait_irq = false;
    } else {
        mask.val = ic_intr_mask_reg->val;
        ic_intr_mask_reg->val = 0;
        ic_intr_mask_reg->val = mask.val;

        SG2002_I2C_TraceOut("mask.m_rx_full   %d\n", mask.field.m_rx_full);
        SG2002_I2C_TraceOut("mask.m_tx_abrt   %d\n", mask.field.m_tx_abrt);
        SG2002_I2C_TraceOut("mask.m_stop_det  %d\n", mask.field.m_stop_det);
        SG2002_I2C_TraceOut("mask.m_tx_empty  %d\n", mask.field.m_tx_empty);
    }

    return 0;
}

static int sg2002_i2c_init(struct sg2002_i2c_priv_s *priv) {
    volatile sg2002_i2c_reg_TypeDef *i2c = NULL;
    SG2002_Con_Reg ic_con_reg;

    if ((priv == NULL) || !sg2002_check_i2c_base(priv->config->base))
        return -1;
    
    if (!sg2002_i2c_enctl(priv, false))
        return -1;

    i2c = SG2002_Priv_2_BaseReg(priv->config->base);

    ic_con_reg.val = 0;
    ic_con_reg.field.master_mode = true;
    ic_con_reg.field.ic_slave_disable = true;
    ic_con_reg.field.ic_restart_en = true;
    To_SG2002_Con_Reg_Ptr(i2c->ic_con)->val = ic_con_reg.val;

    To_SG2002_Rx_Tl_Reg_Ptr(i2c->ic_rx_tl)->field.rx_tl = 0;
    To_SG2002_Tx_Tl_Reg_Ptr(i2c->ic_tx_tl)->field.tx_tl = 0;
    
    /* disable all interrupt */
    To_SG2002_Intr_Mask_Reg_Ptr(i2c->ic_intr_mask)->val = 0;

    /* clear all interrupt */
    uint32_t tmp = To_SG2002_Clr_Intr_Reg_Ptr(i2c->ic_clr_intr)->val;

    if ((sg2002_set_bus_speed(priv, SG2002_I2C_BUS_MODE_FAST)) | !sg2002_i2c_enctl(priv, false))
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

    flags = enter_critical_section();

    if ((volatile int)priv->refs++ == 0) {
#ifndef CONFIG_I2C_POLLED
        sg2002_i2c_sem_init(priv);
#endif        
        sg2002_i2c_init(priv);
    }

#ifndef CONFIG_I2C_POLLED
    if (irq_attach(priv->config->irq, sg2002_i2c_irq_handle, priv) == OK)
        up_enable_irq(priv->config->irq);
#endif

    leave_critical_section(flags);
    return (struct i2c_master_s *)priv;
}

/* developping */
int sg2002_i2cbus_uninitialize(struct i2c_master_s *dev) {

    return -1;
}

