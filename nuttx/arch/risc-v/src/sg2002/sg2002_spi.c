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
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "hardware/sg2002_mmio.h"
#include "sg200x.h"

struct sg2002_spi_config_s {
    uint32_t base;
    uint32_t irq;
};

struct sg2002_spi_priv_s {
    /* Standard SPI operations */
    const struct spi_ops_s *ops;

    /* port configuration */
    const struct sg2002_spi_config_s *config;

    int refs;                                       /* Reference count */
};

static bool sg2002_check_spibus_base(uint32_t base);

static bool sg2002_check_spibus_base(uint32_t base) {
    return false;
}

struct spi_dev_s *sg2002_spibus_initialize(int port) {
    
    return NULL;
}

/* developping */
int sg2002_spibus_uninitialize(struct spi_dev_s *dev) {
    return -1;
}
