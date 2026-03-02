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

typedef struct {
    const uint32_t base_addr;
    uint32_t pin_config;
    uint32_t pin_dir;
} SG2002_Config_Info_TypeDef;

static SG2002_Config_Info_TypeDef SG2002_Conf[4] = {
    {SG2002_GPIO0_BASE, 0, 0},
    {SG2002_GPIO1_BASE, 0, 0},
    {SG2002_GPIO2_BASE, 0, 0},
    {SG2002_GPIO3_BASE, 0, 0}
};

typedef reg_t volatile uint32_t;

#define SG2002_Port_2_BaseReg(x)                    (volatile sg2002_gpio_reg_TypeDef *)((uintptr_t)(x))

#define To_SG2002_GPIO_SWPortA_DR_Reg_Ptr(x)        ((volatile SG2002_GPIO_SWPortA_DR_Reg_TyeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(x)       ((volatile SG2002_GPIO_SWPortA_DDR_Reg_TypeDef      *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntEn_Reg_Ptr(x)             ((volatile SG2002_GPIO_IntEN_Reg_TypeDef            *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntMask_Reg_Ptr(x)           ((volatile SG2002_GPIO_IntMask_Reg_TypeDef          *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntType_Level_Reg_Ptr(x)     ((volatile SG2002_GPIO_IntType_Level_Reg_TypeDef    *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntPriority_Reg_Ptr(x)       ((volatile SG2002_GPIO_Int_Priority_Reg_TypeDef     *)(uintptr_t)&(x))
#define To_SG2002_GPIO_IntStatus_Reg_Ptr(x)         ((volatile SG2002_GPIO_IntStatus_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_RawIntStatus_Reg_Ptr(x)      ((volatile SG2002_GPIO_RawIntStatus_Reg_TypeDef     *)(uintptr_t)&(x))
#define To_SG2002_GPIO_Debounce_Reg_Ptr(x)          ((volatile SG2002_GPIO_Debounce_Reg_TypeDef         *)(uintptr_t)&(x))
#define To_SG2002_GPIO_PortA_EOI_Reg_Ptr(x)         ((volatile SG2002_GPIO_PortA_EOI_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_EXT_PortA_Reg_Ptr(x)         ((volatile SG2002_GPIO_EXT_PortA_Reg_TypeDef        *)(uintptr_t)&(x))
#define To_SG2002_GPIO_LS_Sync_Reg_Ptr(x)           ((volatile SG2002_GPIO_LS_Sync_Reg_TypeDef          *)(uintptr_t)&(x))

typedef struct {
    reg_t swporta_dr;                       /* 0x00 */
    reg_t swporta_ddr;                      /* 0x04 */
    reg_t res_1[9];                         /* 0x08 ~ 0x2C */
    reg_t int_en;                           /* 0x30 */
    reg_t int_mask;                         /* 0x34 */
    reg_t int_type_level;                   /* 0x38 */
    reg_t int_priority;                     /* 0x3C */
    reg_t int_status;                       /* 0x40 */
    reg_t raw_int_status;                   /* 0x44 */
    reg_t debounce;                         /* 0x48 */
    reg_t porta_eoi;                        /* 0x4C */
    reg_t ext_porta;                        /* 0x50 */
    reg_t res_2[3];                         /* 0x54 ~ 0x5C */
    reg_t ls_sync;                          /* 0x60 */
} sg2002_gpio_reg_TypeDef;

static bool sg2002_gpio_check_base(sg2002_gpioset_t pin) {
    if (pin.pin < 32) {
        switch (pin.port) {
            case 0:
            case 1:
            case 2:
            case 3:  return true;
            default: break;
        }
    }

    return false;
}

static void sg2002_gpio_set_dir(sg2002_gpioset_t pin, uint8_t dir) {
    uint32_t mask = 0;
    volatile sg2002_gpio_reg_TypeDef *port_reg = NULL;

    if (!sg2002_gpio_check_base(pin))
        return;

    mask = 1 << pin.pin;
    port_reg = SG2002_Port_2_BaseReg(SG2002_Conf[pin.port].base_addr);
    if ((SG2002_GPIO_DirType_List)dir == SG2002_GPIO_Input) {
        ddr_reg_val &= !mask;
        To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(port_reg->swporta_ddr)->val &= !mask;
    } else {
        To_SG2002_GPIO_SWPortA_DDR_Reg_Ptr(port_reg->swporta_ddr)->val |= mask;
    }
}

void sg2002_gpio_write(sg2002_gpioset_t pinset, bool value) {

}

bool sg2002_gpio_read(sg2002_gpioset_t pinset) {

}

int sg2002_gpio_set_event(sg2002_gpioset_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg) {

}

void sg2002_gpio_init(void) {

}


