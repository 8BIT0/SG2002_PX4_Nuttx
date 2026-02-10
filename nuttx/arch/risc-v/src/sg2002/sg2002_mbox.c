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
#include <nuttx/mbox/mbox.h>

#include <arch/board/board.h>

#include "hardware/sg2002_cache.h"
#include "hardware/sg2002_mbox.h"

#define C906_MAGIC_HEADER                       0xA55AC906 // master cpu is c906
#define CA53_MAGIC_HEADER                       0xA55ACA53 // master cpu is ca53
#define RTOS_MAGIC_HEADER                       C906_MAGIC_HEADER

#define SG2002_Mailbox_TraceOut(fmt, ...)       sg2002_trace_dirout(fmt, ##__VA_ARGS__)
// #define SG2002_Mailbox_TraceOut(fmt, ...)

typedef struct {
    uint32_t reg_base;
    uint32_t done_base;
    uint32_t context_base;
    uint32_t irq;
} sg2002_mailbox_config_s;

typedef struct {
    const struct mbox_ops_s *ops;

    const sg2002_mailbox_config_s *config;
    SG2002_MailboxSet_Reg_TypeDef *set_reg;
    SG2002_MailboxSetDone_Reg_TypeDef *done_reg;
    uint32_t *context;
    transfer_config_t transfer_config;
} sg2002_mailbox_priv_s;

static const sg2002_mailbox_config_s sg2002_mailbox_config = {
    .reg_base = SG2002_MAILBOX_REG_ADDR,
    .done_base = SG2002_MAILBOX_DONE_REG_ADDR,
    .context_base = SG2002_MAILBOX_CONTEXT_ADDR,
    .irq = SG2002_IRQ_MBOX_BASE,
};

static const struct mbox_ops_s sg2002_mbox_ops = {
    .send = NULL,
    .registercallback = NULL,
};

static sg2002_mailbox_priv_s sg2002_mailbox_priv = {
    .ops = &sg2002_mbox_ops,
    .config = &sg2002_mailbox_config,
    .set_reg = NULL,
    .done_reg = NULL,
    .context = NULL,
};

static bool sg2002_check_mailbox_valid(sg2002_mailbox_priv_s *priv) {
    if ((priv == NULL) || \
        (priv->config == NULL) || \
        (priv->config->reg_base != SG2002_MAILBOX_REG_ADDR) || \
        (priv->config->done_base != SG2002_MAILBOX_DONE_REG_ADDR) || \
        (priv->config->context_base != SG2002_MAILBOX_CONTEXT_ADDR))
        return false;
        
    return true;
}

static int sg2002_mailbox_irq_handle(int irq, void *context, void *arg) {
    uint8_t set_val = 0;
    uint8_t mb_valid = 0;

    sg2002_mailbox_priv_s *priv = (sg2002_mailbox_priv_s *)arg;
    SG2002_CMDQU_TypeDef *r_cmdqu = NULL;
    SG2002_CMDQU_TypeDef t_cmdqu;
    SG2002_MailboxSet_Reg_TypeDef *set_reg = priv->set_reg;

    if (!sg2002_check_mailbox_valid(priv))
        return -1;

    set_val = set_reg->cpu_mbox_set[SG2002_RECEIVE_CPU].mb_int._int;
    if (set_val) {
        for (uint8_t i = 0; i < SG2002_MAILBOX_MAX_NUM; i ++) {
            mb_valid = set_val & (1 << i);

            memset(&t_cmdqu, 0, sizeof(SG2002_CMDQU_TypeDef));

            if (mb_valid) {
                r_cmdqu = ((SG2002_CMDQU_TypeDef *)(priv->context)) + i;
                sg2002_flush_dcache_range((uintptr_t)(&r_cmdqu->param_ptr), sizeof(uint32_t));

                set_reg->cpu_mbox_set[SG2002_RECEIVE_CPU].mb_clr._clr= mb_valid;
                set_reg->cpu_mbox_en[SG2002_RECEIVE_CPU]._info &= ~mb_valid;
                
                *((uint32_t *)((uintptr_t)&t_cmdqu)) = *((uint32_t *)((uintptr_t)r_cmdqu));
                *((uint32_t *)r_cmdqu) = 0;

                if (t_cmdqu.resv.valid.linux_valid == SG2002_LINUX_VALID_VALID) {
                    SG2002_Mailbox_TraceOut("i          %d\n", i);
                    SG2002_Mailbox_TraceOut("ip_id      %d\n", t_cmdqu.ip_id);
                    SG2002_Mailbox_TraceOut("cmd_id     %d\n", t_cmdqu.cmd_id);
                    SG2002_Mailbox_TraceOut("block      %s\n", t_cmdqu.block ? "true" : "false");
                    SG2002_Mailbox_TraceOut("ms         %d\n", t_cmdqu.resv.ms);
                    SG2002_Mailbox_TraceOut("linux_v    %s\n", t_cmdqu.resv.valid.linux_valid ? "true" : "false");
                    SG2002_Mailbox_TraceOut("rtos_v     %s\n", t_cmdqu.resv.valid.rtos_valid ? "true" : "false");
                    SG2002_Mailbox_TraceOut("param_ptr  0x%08x\n", t_cmdqu.param_ptr);
                    SG2002_Mailbox_TraceOut("\n");
                }
            }
        }
    }

    return 0;
}

static void sg2002_get_comm_info(void) {
    uint16_t checksum = 0;
    uint8_t *ptr = (uint8_t *)(&sg2002_mailbox_priv.transfer_config);

    memcpy((uint8_t *)&sg2002_mailbox_priv.transfer_config, (uint8_t *)SG2002_MAILBOX_CONTEXT_ADDR, sizeof(transfer_config_t));
    /* clear communication information from mailbox 0x1900400 */
    memset((uint8_t *)SG2002_MAILBOX_CONTEXT_ADDR, 0 , sizeof(SG2002_CMDQU_TypeDef) * SG2002_MAILBOX_MAX_NUM);

    /* check configuration from fsbl */
    for (int i = 0; i < sg2002_mailbox_priv.transfer_config.conf_size; i++, ptr++) {
        checksum += *ptr;
    }

    if (checksum != sg2002_mailbox_priv.transfer_config.checksum) {
        SG2002_Mailbox_TraceOut("checksum fail (%x, %x)\n", sg2002_mailbox_priv.transfer_config.checksum, checksum);
        SG2002_Mailbox_TraceOut("use default confi setting\n");
        sg2002_mailbox_priv.transfer_config.conf_magic = RTOS_MAGIC_HEADER;
        sg2002_mailbox_priv.transfer_config.conf_size = ((uint64_t) &sg2002_mailbox_priv.transfer_config.checksum - (uint64_t) &sg2002_mailbox_priv.transfer_config.conf_magic);
        sg2002_mailbox_priv.transfer_config.isp_buffer_addr = ISP_MEM_BASE_ADDR;
        sg2002_mailbox_priv.transfer_config.isp_buffer_size = ISP_MEM_BASE_SIZE;
        sg2002_mailbox_priv.transfer_config.encode_img_addr = H26X_BITSTREAM_ADDR;
        sg2002_mailbox_priv.transfer_config.encode_img_size = H26X_BITSTREAM_SIZE;
        sg2002_mailbox_priv.transfer_config.encode_buf_addr = H26X_ENC_BUFF_ADDR;
        sg2002_mailbox_priv.transfer_config.encode_buf_size = H26X_ENC_BUFF_SIZE;
        sg2002_mailbox_priv.transfer_config.image_type = 0;
        sg2002_mailbox_priv.transfer_config.dump_print_enable = 0;
        sg2002_mailbox_priv.transfer_config.dump_print_size_idx = 0;
        for (int i = 0, checksum = 0; i < sg2002_mailbox_priv.transfer_config.conf_size; i++, ptr++) {
            checksum += *ptr;
        }
        sg2002_mailbox_priv.transfer_config.checksum = checksum;
    }

    sg2002_mailbox_priv.transfer_config.mcu_status = MCU_STATUS_RTOS_T1_INIT;
    sg2002_mailbox_priv.transfer_config.linux_status = MCU_STATUS_LINUX_INIT ;
    sg2002_flush_dcache_range((uintptr_t)&sg2002_mailbox_priv.transfer_config, sizeof(transfer_config_t));
}

struct mbox_dev_s *sg2002_mailbox_initialize(void) {
    irqstate_t flags;

    flags = enter_critical_section();
    
    sg2002_mailbox_priv.set_reg = (SG2002_MailboxSet_Reg_TypeDef *)((uintptr_t)(sg2002_mailbox_priv.config->reg_base));
    sg2002_mailbox_priv.done_reg = (SG2002_MailboxSetDone_Reg_TypeDef *)((uintptr_t)(sg2002_mailbox_priv.config->done_base));
    sg2002_mailbox_priv.context = (uint32_t *)((uintptr_t)(sg2002_mailbox_priv.config->context_base));

    /* cache init */
    sg2002_get_comm_info();

    /* init spinlock */

    /* attach irq */
    if (irq_attach(sg2002_mailbox_priv.config->irq, sg2002_mailbox_irq_handle, &sg2002_mailbox_priv) == OK)
        up_enable_irq(sg2002_mailbox_priv.config->irq);

    leave_critical_section(flags);

    return (struct mbox_dev_s *)((uintptr_t)&sg2002_mailbox_priv);
}

/* developping */
int sg2002_mailbox_uninitialize(struct mbox_dev_s *dev) {
    return -1;
}
