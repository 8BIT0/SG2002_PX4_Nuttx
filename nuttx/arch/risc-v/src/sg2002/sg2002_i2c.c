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

#include "sg2002_i2c.h"

struct sg2002_i2c_config_s
{
    uint32_t base;              /* I2C base address */
    uint32_t clk_bit;           /* Clock enable bit */
    uint32_t reset_bit;         /* Reset bit */
    uint32_t irq;               /* Event IRQ */
};

/* I2C Device Private Data */

struct sg2002_i2c_priv_s {
    /* Standard I2C operations */

    const struct i2c_ops_s *ops;

    /* Port configuration */

    const struct sg2002_i2c_config_s *config;

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

/* I2C interface */

static const struct i2c_ops_s sg2002_i2c_ops =
{
  .transfer = sg2002_i2c_transfer
};

/* I2C device structures */

#ifdef CONFIG_SG2002_I2C1
static const struct sg2002_i2c_config_s sg2002_i2c1_config =
{
  .base       = SG2002_I2C1_BASE,
  .clk_bit    = RCC_APB1ENR_I2C1EN,
  .reset_bit  = RCC_APB1RSTR_I2C1RST,
  .irq        = STM32_IRQ_I2C1EV,
};

static struct sg2002_i2c_priv_s sg2002_i2c1_priv =
{
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
static const struct sg2002_i2c_config_s sg2002_i2c3_config =
{
  .base       = SG2002_I2C3_BASE,
  .clk_bit    = RCC_APB1ENR_I2C3EN,
  .reset_bit  = RCC_APB1RSTR_I2C3RST,
  .irq        = STM32_IRQ_I2C3EV,
};

static struct sg2002_i2c_priv_s stm32_i2c3_priv =
{
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

struct i2c_master_s *sg2002_i2cbus_initialize(int port) {
  struct sg2002_i2c_priv_s *priv = NULL;
  irqstate_t flags;

    switch (port) {
        case 1: {
            /* pin init */

            priv = (struct sg2002_i2c_priv_s *)&sg2002_i2c1_p
            break;
        }

        case 3: {
            /* pin init */

            break;
        }

        default: return NULL;
    }

    return NULL;
}

int sg2002_i2cbus_uninitialize(struct i2c_master_s *dev) {

    return -1;
}

