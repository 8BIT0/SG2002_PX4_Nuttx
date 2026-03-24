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
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "sg200x.h"

#ifdef CONFIG_TIMER

#define SG2002_Timer_TraceOut(fmt, ...)       sg2002_trace_dirout(fmt, ##__VA_ARGS__)
// #define SG2002_Timer_TraceOut(fmt, ...)

#define To_SG2002_SingleTimerReg_Ptr(x)     (sg2002_timer_reg *)((uintptr_t)x)

#define sg2002_timer_control_reg            volatile SG2002_TimerControlReg_TypeDef
#define sg2002_timer_EOI_reg                volatile SG2002_TimerEOI_Reg_TypeDef
#define sg2002_timer_int_status_reg         volatile SG2002_TimerIntStatus_TypeDef

#define To_SG2002_Timer_ControlReg_Ptr(x)   ((sg2002_timer_control_reg *)&x)
#define To_SG2002_Timer_EOIReg_Ptr(x)       ((sg2002_timer_EOI_reg *)&x)
#define To_SG2002_Timer_IntStatusReg_Ptr(x) ((sg2002_timer_int_status_reg *)&x)

typedef struct {
    reg_t load_count;
    reg_t current_val;
    reg_t control_reg;
    reg_t eoi;                      /* read only. read to clear interrupt */
    reg_t int_status;               /* read only. */
} _sg2002_single_timer_reg_TypeDef;

#define sg2002_timer_reg                    volatile _sg2002_single_timer_reg_TypeDef

typedef struct {
    sg2002_timer_reg Timer0_reg;    /* 0x00 ~ 0x10 */
    sg2002_timer_reg Timer1_reg;    /* 0x14 ~ 0x24 */
    sg2002_timer_reg Timer2_reg;    /* 0x28 ~ 0x38 */
    sg2002_timer_reg Timer3_reg;    /* 0x3C ~ 0x4C */
    sg2002_timer_reg Timer4_reg;    /* 0x50 ~ 0x60 */
    sg2002_timer_reg Timer5_reg;    /* 0x64 ~ 0x74 */
    sg2002_timer_reg Timer6_reg;    /* 0x78 ~ 0x88 */
    sg2002_timer_reg Timer7_reg;    /* 0x8C ~ 0x9C */

    reg_t TimersIntStatus;          /* 0xa0 */
    reg_t TimersEOI;                /* 0xa4 */
    reg_t TimersRawIntStatus;       /* 0xa8 */
} sg2002_all_timer_reg_TypeDef;

struct sg2002_timer_config_s {
    const uint32_t base;
    const uint32_t irq;
};

struct sg2002_timer_priv_s {
    const struct timer_ops_s *ops;                  /* Standard SPI operations */
    struct sg2002_timer_config_s *config;           /* port configuration */
    
    uint32_t freq;                                  /* unit: us */
    uint32_t load_count;

    tccb_t callback;
    void *callback_arg;
};

static int sg2002_timer_start(FAR struct timer_lowerhalf_s *lower);
static int sg2002_timer_stop(FAR struct timer_lowerhalf_s *lower);
static int sg2002_timer_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd, unsigned long arg);
static int sg2002_timer_set_callback(FAR struct timer_lowerhalf_s *lower, CODE tccb_t callback, FAR void *arg);
static int sg2002_timer_get_status_dummy(FAR struct timer_lowerhalf_s *lower, FAR struct timer_status_s *status);
static int sg2002_timer_set_timeout_dummy(FAR struct timer_lowerhalf_s *lower, uint32_t timeou);
static int sg2002_timer_max_timeout_dummy(FAR struct timer_lowerhalf_s *lower, FAR uint32_t *maxtimeou);

struct timer_ops_s sg2002_timer_ops = {
    .start = sg2002_timer_start,
    .stop = sg2002_timer_stop,
    .ioctl = sg2002_timer_ioctl,
    .setcallback = sg2002_timer_set_callback,
    .getstatus = sg2002_timer_get_status_dummy,
    .settimeout = sg2002_timer_set_timeout_dummy,
    .maxtimeout = sg2002_timer_max_timeout_dummy,
};

struct sg2002_timer_config_s SG2002_Timer4_Cfg = {
    .base = SG2002_TIMER4_BASE,
    .irq  = SG2002_IRQ_TIMER4_BASE,
};

struct sg2002_timer_config_s SG2002_Timer5_Cfg = {
    .base = SG2002_TIMER5_BASE,
    .irq  = SG2002_IRQ_TIMER5_BASE,
};

struct sg2002_timer_config_s SG2002_Timer6_Cfg = {
    .base = SG2002_TIMER6_BASE,
    .irq  = SG2002_IRQ_TIMER6_BASE,
};

struct sg2002_timer_config_s SG2002_Timer7_Cfg = {
    .base = SG2002_TIMER7_BASE,
    .irq  = SG2002_IRQ_TIMER7_BASE,
};

struct sg2002_timer_priv_s sg2002_timer4_priv = {
    .ops = &sg2002_timer_ops,
    .config = &SG2002_Timer4_Cfg,
    .freq = 0,
    .callback = NULL,
};

struct sg2002_timer_priv_s sg2002_timer5_priv = {
    .ops = &sg2002_timer_ops,
    .config = &SG2002_Timer5_Cfg,
    .freq = 0,
    .callback = NULL,
};

struct sg2002_timer_priv_s sg2002_timer6_priv = {
    .ops = &sg2002_timer_ops,
    .config = &SG2002_Timer6_Cfg,
    .freq = 0,
    .callback = NULL,
};

struct sg2002_timer_priv_s sg2002_timer7_priv = {
    .ops = &sg2002_timer_ops,
    .config = &SG2002_Timer7_Cfg,
    .freq = 0,
    .callback = NULL,
};

static bool sg2002_check_timer_base(uint32_t base) {
    switch (base) {
        case SG2002_TIMER4_BASE:
        case SG2002_TIMER5_BASE:
        case SG2002_TIMER6_BASE:
        case SG2002_TIMER7_BASE:
            return true;

        default: return false;
    }

    return false;
}

static bool sg2002_timer_reset(sg2002_timer_reg *timer_reg) {
    if (timer_reg == NULL)
        return false;

    timer_reg->load_count = 0;
    timer_reg->control_reg = 0;

    return false;
}

static uint32_t sg2002_timer_get_load_count(sg2002_timer_reg *timer_reg) {
    if (timer_reg == NULL)
        return 0;

    return timer_reg->load_count;
}

static bool sg2002_timer_set_load_count(sg2002_timer_reg *timer_reg, uint32_t val) {
    if (timer_reg == NULL)
        return false;

    timer_reg->load_count = val;
    if (timer_reg->load_count == val)
        return true;

    return false;
}

static uint32_t sg2002_timer_get_current_count(sg2002_timer_reg *timer_reg) {
    if (timer_reg == NULL)
        return 0;

    return timer_reg->current_val;
}

static bool sg2002_timer_enctl(sg2002_timer_reg *timer_reg, bool state) {
    if (timer_reg == NULL)
        return false;

    To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.en = state;
    if (To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.en == state)
        return true;

    return false;
}

static bool sg2002_timer_free_mode_ctl(sg2002_timer_reg *timer_reg, bool ctl) {
    if (timer_reg == NULL)
        return false;

    To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.mode = ctl;
    if (To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.mode == ctl)
        return true;

    return false;
}

static bool sg2002_timer_clear_irq(sg2002_timer_reg *timer_reg) {
    sg2002_timer_EOI_reg reg;

    /* read reg to clear */
    reg.val = To_SG2002_Timer_EOIReg_Ptr(timer_reg->eoi)->val;

    return reg.field.eoi;
}

static bool sg2002_timer_int_mask_ctl(sg2002_timer_reg *timer_reg, bool ctl) {
    if (timer_reg == NULL)
        return false;

    To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.int_mask = ctl;
    if (To_SG2002_Timer_ControlReg_Ptr(timer_reg->control_reg)->field.int_mask == ctl)
        return true;

    return false;
}

static int sg2002_timer_start(FAR struct timer_lowerhalf_s *lower) {
    struct sg2002_timer_priv_s *priv = (struct sg2002_timer_priv_s *)lower;
    sg2002_timer_reg *timer = NULL;
    bool state = true;

    if ((priv == NULL) || \
        (priv->config == NULL) || \
        !sg2002_check_timer_base(priv->config->base) || \
        (priv->freq == 0))
        return -1;

    timer = To_SG2002_SingleTimerReg_Ptr(priv->config->base);

    /* disable timer */
    state &= sg2002_timer_enctl(timer, false);

    /* set counter */
    timer->load_count = SG2002_TIMER_FREQ / priv->freq;
    state &= sg2002_timer_set_load_count(timer, timer->load_count);

    SG2002_Timer_TraceOut("Timer freqeunce %d\n", priv->freq);
    SG2002_Timer_TraceOut("Timer load count %d\n", priv->load_count);

    /* set mode */
    state &= sg2002_timer_free_mode_ctl(timer, true);

    /* enable timer */
    state &= sg2002_timer_enctl(timer, true);

    /* clear int mask */
    state &= sg2002_timer_int_mask_ctl(timer, false);

    if (!state)
        return -1;

    SG2002_Timer_TraceOut("timer start\n");

    return 0;
}

static int sg2002_timer_stop(FAR struct timer_lowerhalf_s *lower) {
    struct sg2002_timer_priv_s *priv = (struct sg2002_timer_priv_s *)lower;
    sg2002_timer_reg *timer = NULL;
    bool state = true;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_timer_base(priv->config->base))
        return -1;

    /* set int mask */
    state &= sg2002_timer_int_mask_ctl(timer, true);

    /* disable tiemr */
    state &= sg2002_timer_enctl(timer, false);

    if (!state)
        return -1;

    return 0;
}

static int sg2002_timer_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd, unsigned long arg) {
   struct sg2002_timer_priv_s *priv = (struct sg2002_timer_priv_s *)lower;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_timer_base(priv->config->base))
        return -1; 

    switch (cmd) {
        case SG2002_Timer_Set_Freq: priv->freq = (uint32_t)arg; break;
        default: return -1;
    }

    return 0;
}

static int sg2002_timer_set_callback(FAR struct timer_lowerhalf_s *lower, CODE tccb_t callback, FAR void *arg) {
    struct sg2002_timer_priv_s *priv = (struct sg2002_timer_priv_s *)lower;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_timer_base(priv->config->base))
        return -1;

    priv->callback = (tccb_t)callback;
    priv->callback_arg = (void *)arg;

    return 0;
}

static int sg2002_timer_get_status_dummy(FAR struct timer_lowerhalf_s *lower, FAR struct timer_status_s *status) {
    return 0;
}

static int sg2002_timer_set_timeout_dummy(FAR struct timer_lowerhalf_s *lower, uint32_t timeou) {
    return 0;
}

static int sg2002_timer_max_timeout_dummy(FAR struct timer_lowerhalf_s *lower, FAR uint32_t *maxtimeou) {
    return 0;
}


static int sg2002_timer_irq_handle(int irq, void *context, void *arg) {
    struct sg2002_timer_priv_s *priv = (struct sg2002_timer_priv_s *)arg;

    if ((priv == NULL) || (priv->config == NULL) || !sg2002_check_timer_base(priv->config->base))
        return -1;

    sg2002_timer_reg *timer = To_SG2002_SingleTimerReg_Ptr(priv->config->base); 

    /* check int state */
    if (To_SG2002_Timer_IntStatusReg_Ptr(timer->int_status)->field.int_status) {
        
        /* clear irq */
        sg2002_timer_clear_irq(timer);
        
        if (priv->callback)
            priv->callback(NULL, priv->callback_arg);

        /* clear int mask */
        sg2002_timer_int_mask_ctl(timer, false);
    }

    return 0;
}

struct timer_lowerhalf_s *sg2002_timer_initialize(int port) {
    struct sg2002_timer_priv_s *priv = NULL;
    irqstate_t flags;

    switch (port) {
        case SG2002_Timer_4: priv = &sg2002_timer4_priv; break;
        case SG2002_Timer_5: priv = &sg2002_timer5_priv; break;
        case SG2002_Timer_6: priv = &sg2002_timer6_priv; break;
        case SG2002_Timer_7: priv = &sg2002_timer7_priv; break;
        default: return NULL;
    }
    
    /* check register */
    if ((priv->config == NULL) || !sg2002_check_timer_base(priv->config->base))
        return NULL;

    flags = enter_critical_section();

    /* reset reg */
    sg2002_timer_reset(To_SG2002_SingleTimerReg_Ptr(priv->config->base));

    if (irq_attach(priv->config->irq, sg2002_timer_irq_handle, priv) == OK)
        up_enable_irq(priv->config->irq);

    leave_critical_section(flags);

    return ((struct timer_lowerhalf_s *)priv);
}

/* developing */
int sg2002_timer_uninitialize(struct timer_lowerhalf_s *dev) {
    return 0;
}

#endif