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

#include "hardware/sg2002_mbox.h"

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

                set_reg->cpu_mbox_set[SG2002_RECEIVE_CPU].mb_clr._clr= mb_valid;
                set_reg->cpu_mbox_en[SG2002_RECEIVE_CPU]._info &= ~mb_valid;
                
                *((uint32_t *)((uintptr_t)&t_cmdqu)) = *((uint32_t *)((uintptr_t)r_cmdqu));
                *((uint32_t *)r_cmdqu) = 0;

                if (t_cmdqu.resv.valid.linux_valid == SG2002_LINUX_VALID_VALID) {
                    SG2002_Mailbox_TraceOut("mailbox irq\n");
                }
            }
        }
    }

    return 0;
}

struct mbox_dev_s *sg2002_mailbox_initialize(void) {
    irqstate_t flags;

    flags = enter_critical_section();
    
    sg2002_mailbox_priv.set_reg = (SG2002_MailboxSet_Reg_TypeDef *)((uintptr_t)(sg2002_mailbox_priv.config->reg_base));
    sg2002_mailbox_priv.done_reg = (SG2002_MailboxSetDone_Reg_TypeDef *)((uintptr_t)(sg2002_mailbox_priv.config->done_base));
    sg2002_mailbox_priv.context = (uint32_t *)((uintptr_t)(sg2002_mailbox_priv.config->context_base));

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
