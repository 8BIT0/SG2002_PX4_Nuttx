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

#include <arch/board/board.h>

#include "hardware/sg2002_mmio.h"
#include "hardware/sg2002_gpio.h"
#include "sg2002_gpio.h"

#define SG2002_GPIO_TraceOut(fmt, ...)       sg2002_trace_dirout(fmt, ##__VA_ARGS__)
// #define SG2002_GPIO_TraceOut(fmt, ...)

typedef struct {
    xcpt_t func;
    void *arg;
} SG2002_ExtiObj_TypeDef;

static SG2002_ExtiObj_TypeDef SG2002_GPIO_Exti_Callback_List[4][32] = {
    {{NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}},
    
    {{NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}},

    {{NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}},
    
    {{NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, \
     {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}, {NULL, NULL}},
};

typedef struct {
    const uint32_t base_addr;
    const uint32_t irq_base;
    SG2002_ExtiObj_TypeDef *irq_obj;
} SG2002_Config_Info_TypeDef;

static SG2002_Config_Info_TypeDef SG2002_Conf[4] = {
    {SG2002_GPIO0_BASE, SG2002_IRQ_GPIO0_BASE, 0, 0, SG2002_GPIO_Exti_Callback_List[0]},
    {SG2002_GPIO1_BASE, SG2002_IRQ_GPIO1_BASE, 0, 0, SG2002_GPIO_Exti_Callback_List[1]},
    {SG2002_GPIO2_BASE, SG2002_IRQ_GPIO2_BASE, 0, 0, SG2002_GPIO_Exti_Callback_List[2]},
    {SG2002_GPIO3_BASE, SG2002_IRQ_GPIO3_BASE, 0, 0, SG2002_GPIO_Exti_Callback_List[3]}
};

typedef volatile uint32_t reg_t;

#define SG2002_Port_2_BaseReg(x)                    (volatile sg2002_gpio_reg_TypeDef *)((uintptr_t)(x))

#define To_SG2002_GPIO_SWPortA_DR_Reg_Ptr(x)        ((volatile SG2002_GPIO_SWPortA_DR_Reg_TyeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(x)       ((volatile SG2002_GPIO_SWPortA_DDR_Reg_TypeDef      *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntEn_Reg_Ptr(x)             ((volatile SG2002_GPIO_IntEN_Reg_TypeDef            *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntMask_Reg_Ptr(x)           ((volatile SG2002_GPIO_IntMask_Reg_TypeDef          *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntType_Level_Reg_Ptr(x)     ((volatile SG2002_GPIO_IntType_Level_Reg_TypeDef    *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntPolarity_Reg_Ptr(x)       ((volatile SG2002_GPIO_Int_Polarity_Reg_TypeDef     *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntStatus_Reg_Ptr(x)         ((volatile SG2002_GPIO_IntStatus_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_RawIntStatus_Reg_Ptr(x)      ((volatile SG2002_GPIO_RawIntStatus_Reg_TypeDef     *)(uintptr_t)&(x))
#define To_SG2002_GPIO_Debounce_Reg_Ptr(x)          ((volatile SG2002_GPIO_Debounce_Reg_TypeDef         *)(uintptr_t)&(x))
#define To_SG2002_GPIO_PortA_EOI_Reg_Ptr(x)         ((volatile SG2002_GPIO_PortA_EOI_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_EXT_PortA_Reg_Ptr(x)         ((volatile SG2002_GPIO_EXT_PortA_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_LS_Sync_Reg_Ptr(x)           ((volatile SG2002_GPIO_LS_Sync_Reg_TypeDef          *)(uintptr_t)&(x))

typedef struct {
    reg_t swporta_dr;                       /* 0x00 */
    reg_t swporta_ddr;                      /* 0x04 */
    reg_t res_1[10];                        /* 0x08 ~ 0x2C */
    reg_t int_en;                           /* 0x30 */
    reg_t int_mask;                         /* 0x34 */
    reg_t int_type_level;                   /* 0x38 */
    reg_t int_polarity;                     /* 0x3C */
    reg_t int_status;                       /* 0x40 */
    reg_t raw_int_status;                   /* 0x44 */
    reg_t debounce;                         /* 0x48 */
    reg_t porta_eoi;                        /* 0x4C */
    reg_t ext_porta;                        /* 0x50 */
    reg_t res_2[3];                         /* 0x54 ~ 0x5C */
    reg_t ls_sync;                          /* 0x60 */
} sg2002_gpio_reg_TypeDef;

static bool sg2002_gpio_check_base(sg2002_gpioset_t pin) {
    if (pin.field.pin < 32) {
        switch (pin.field.port) {
            case 0:
            case 1:
            case 2:
            case 3:  return true;
            default: break;
        }
    }

    return false;
}

static bool sg2002_gpio_check_int_type(uint8_t int_type) {
    switch (int_type) {
        case (uint8_t)SG2002_INT_LowLevel:
        case (uint8_t)SG2002_INT_HighLevel:
        case (uint8_t)SG2002_INT_FallingEdge:
        case (uint8_t)SG2002_INT_RisingEdge:
            return true;

        default: return false;
    }
}

/* config interrupt */
static bool sg2002_gpio_cfg_int(sg2002_gpioset_t pin, uint8_t int_type) {
    uint8_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;

    if (!sg2002_gpio_check_base(pin) | !sg2002_gpio_check_int_type(int_type))
        return false;

    mask = (1 << pin.field.pin);
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.field.port].base_addr);

    /* set int type level reg */
    if ((int_type == SG2002_INT_LowLevel) || (int_type == SG2002_INT_HighLevel)) {
        /* level type */
        To_SG2002_GPIO_IntType_Level_Reg_Ptr(port_reg->int_type_level)->val |= mask;
    } else {
        /* edge type */
        To_SG2002_GPIO_IntType_Level_Reg_Ptr(port_reg->int_type_level)->val &= !mask;
    }

    /* set int polarity */
    if ((int_type == SG2002_INT_HighLevel) || (int_type == SG2002_INT_RisingEdge)) {
        /* high or rising */
        To_SG2002_GPIO_IntPolarity_Reg_Ptr(port_reg->int_polarity)->val |= mask;
    } else {
        /* low or falling */
        To_SG2002_GPIO_IntPolarity_Reg_Ptr(port_reg->int_polarity)->val &= !mask;
    }

    return true;
}

static void sg2002_gpio_set_int(sg2002_gpioset_t pin, bool en) {
    uint32_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;

    if (!sg2002_gpio_check_base(pin))
        return;

    mask = (1 << pin.field.pin);
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.field.port].base_addr);
    
    if (en) {
        To_SG2002_GPIO_IntEn_Reg_Ptr(port_reg->int_en)->val |= mask;
    } else {
        To_SG2002_GPIO_IntEn_Reg_Ptr(port_reg->int_en)->val &= !mask;
    }
}

static void sg2002_gpio_set_dir(sg2002_gpioset_t pin, uint8_t dir) {
    uint32_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;

    if (!sg2002_gpio_check_base(pin))
        return;

    mask = (1 << pin.field.pin);
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.field.port].base_addr);
    
    SG2002_GPIO_TraceOut("addr reg addr 0x%08X\n", (uint32_t)((uintptr_t)port_reg));

    if ((SG2002_GPIO_DirType_List)dir == SG2002_GPIO_Input) {
        To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(port_reg->swporta_ddr)->val &= !mask;
    } else {
        To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(port_reg->swporta_ddr)->val |= mask;
    }
}

static int sg2002_gpio_irq_handle(int irq, void *context, void *arg) {
    SG2002_GPIO_TraceOut("EXTI irq\n");
    return 0;
}

void sg2002_gpio_write(sg2002_gpioset_t pin, bool value) {
    uint32_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;
    
    if (!sg2002_gpio_check_base(pin))
        return;

    mask = (1 << pin.field.pin);
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.field.port].base_addr);

    if (value) {
        /* level output high */
        To_SG2002_GPIO_SWPortA_DR_Reg_Ptr(port_reg->swporta_dr)->val |= mask;
    } else {
        /* level output low */
        To_SG2002_GPIO_SWPortA_DR_Reg_Ptr(port_reg->swporta_dr)->val &= !mask;
    }
}

bool sg2002_gpio_read(sg2002_gpioset_t pin) {
    uint32_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;

    if (!sg2002_gpio_check_base(pin))
        return false;

    mask = (1 << pin.field.pin);
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.field.port].base_addr);

    return ((To_SG2002_GPIO_EXT_PortA_Reg_Ptr(port_reg)->val & mask) != 0);
}

int sg2002_gpio_set_event(sg2002_gpioset_t pin, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg) {
    uint8_t int_type = 0;
    
    /* both set or both reset */
    if ((risingedge & fallingedge) || !(risingedge | fallingedge))
        return -1;

    if (risingedge) {
        int_type = SG2002_INT_RisingEdge;
    } else if (fallingedge) {
        int_type = SG2002_INT_FallingEdge;
    }

    if (!sg2002_gpio_cfg_int(pin, int_type))
        return -1;

    sg2002_gpio_set_int(pin, true);

    /* set callback and arg */
    SG2002_Conf[pin.field.port].irq_obj[pin.field.pin].func = func;
    SG2002_Conf[pin.field.port].irq_obj[pin.field.pin].arg = arg;

    irq_attach(SG2002_Conf[pin.field.port].irq_base, sg2002_gpio_irq_handle, &SG2002_Conf[pin.field.port]);
    up_enable_irq(SG2002_Conf[pin.field.port].irq_base);

    return 0;
}

static uint32_t sg2002_get_reg_offset(uint32_t reg) {
    volatile sg2002_gpio_reg_TypeDef *port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[0].base_addr);

    return (reg - (uint32_t)((uintptr_t)port_reg));
}

void sg2002_gpio_init(void) {
    /* set gpioa14 and gpioa15 as normal gpio */
    mmio_clrsetbits_32(SG2002_GPIO_A14_REG, 0x07, SG2002_GPIO_A14_REG_VAL);
    mmio_clrsetbits_32(SG2002_GPIO_A15_REG, 0x07, SG2002_GPIO_A15_REG_VAL);

    /* set gpioa 14 as output */
    /* set gpioa 15 as exti irq */
}


