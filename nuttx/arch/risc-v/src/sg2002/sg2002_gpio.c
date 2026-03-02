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

typedef reg_t volatile uint32_t;

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

