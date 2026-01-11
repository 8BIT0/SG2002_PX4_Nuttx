#include <nuttx/config.h>
#include <nuttx/board.h>
#include <sys/types.h>
#include <errno.h>

#include "sg200x.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void sg2002_i2c_register(int bus) {
    struct i2c_master_s *i2c = sg2002_i2cbus_initialize(bus);

    if (i2c)
        i2c_register(i2c, bus);
}

static void sg2002_i2ctool(void) {
#ifdef CONFIG_SG2002_I2C1
    sg2002_i2c_register(1);
#endif
#ifdef CONFIG_SG2002_I2C3
    sg2002_i2c_register(3);
#endif
}
#endif

int sg2002_bringup(void) {
#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  sg2002_i2ctool();
#endif

    return 0;
}