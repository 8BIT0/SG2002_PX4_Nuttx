#include <nuttx/config.h>
#include <nuttx/board.h>
#if defined(CONFIG_EEPROM)
#if defined(CONFIG_I2C_EE_24XX)
#include <nuttx/eeprom/i2c_xx24xx.h>
#endif
#endif
#include <sys/types.h>
#include <errno.h>

#include "sg200x.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static struct i2c_master_s *sg2002_i2c_register(int bus) {
    struct i2c_master_s *i2c = sg2002_i2cbus_initialize(bus);

    if (i2c)
        i2c_register(i2c, bus);

    return i2c;
}
#endif

int sg2002_bringup(void) {
#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
    struct i2c_master_s *eeprom_i2c = NULL;

#ifdef CONFIG_SG2002_I2C1
    eeprom_i2c = sg2002_i2c_register(1);
#endif

#ifdef CONFIG_SG2002_I2C3
    sg2002_i2c_register(3);
#endif

#if defined(CONFIG_EEPROM) && defined(CONFIG_I2C_EE_24XX)
    if (eeprom_i2c != NULL)
        ee24xx_initialize(eeprom_i2c, 0x50, "/dev/eeprom0", EEPROM_24XX64, 0);
#endif

#endif

    return 0;
}