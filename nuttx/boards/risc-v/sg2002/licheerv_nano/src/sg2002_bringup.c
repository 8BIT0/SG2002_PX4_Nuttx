#include <nuttx/config.h>
#include <nuttx/board.h>
#if defined(CONFIG_EEPROM)
    #if defined(CONFIG_I2C_EE_24XX)
        #include <nuttx/eeprom/i2c_xx24xx.h>
    #endif
#endif

#if defined(CONFIG_USERLED)
    #if defined (CONFIG_USERLED_LOWER)
        #include <nuttx/leds/userled.h>
    #endif
#endif

#if defined(CONFIG_ARCH_BUTTONS)
    #include <nuttx/input/buttons.h>
#endif

#include <nuttx/mbox/mbox.h>
#include <sys/types.h>
#include <errno.h>
#include <arch/board/board.h>

#include "sg200x.h"

#if defined(CONFIG_MTD_W25)
#include <nuttx/mtd/mtd.h>
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static struct i2c_master_s *sg2002_i2c_register(int bus) {
    struct i2c_master_s *i2c = sg2002_i2cbus_initialize(bus);

    if (i2c)
        i2c_register(i2c, bus);

    return i2c;
}
#endif

int sg2002_bringup(void) {

	sg2002_gpio_init();

#if defined(CONFIG_SG2002_SPI2)
    struct spi_dev_s *spi_dev = NULL;

    spi_dev = sg2002_spibus_initialize(2);

#if defined(CONFIG_MTD_W25)
    struct mtd_dev_s *mtd = w25_initialize(spi_dev);
    if (!mtd) {
        sg2002_trace_dirout("W25 init failed\n");
        return -ENODEV;
    } else
        sg2002_trace_dirout("W25 init success\n");
#endif

#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
    struct i2c_master_s *eeprom_i2c = NULL;

#ifdef CONFIG_SG2002_I2C1
    sg2002_i2c_register(1);
#endif

#ifdef CONFIG_SG2002_I2C3
    eeprom_i2c = sg2002_i2c_register(3);
#endif

#if defined(CONFIG_EEPROM) && defined(CONFIG_I2C_EE_24XX)
    if (eeprom_i2c != NULL) {
        if(ee24xx_initialize(eeprom_i2c, 0x50, "/dev/eeprom0", EEPROM_24XX64, 0) < 0)
            return -1;
    }
#endif
#endif

#if defined(CONFIG_ARCH_HAVE_LEDS) && defined(CONFIG_USERLED)
#if defined(CONFIG_USERLED_LOWER)
    // if (userled_lower_initialize("/dev/test_pin") < 0)
    //     return -1;
#endif
#endif

#if defined(CONFIG_ARCH_BUTTONS) && defined(CONFIG_ARCH_IRQBUTTONS)
    // if (btn_lower_initialize("/dev/test_exti") < 0)
        // return -1;
#endif

    /* init mailbox */
    struct mbox_dev_s *mbox_dev = sg2002_mailbox_initialize();
    if (mbox_dev != NULL) {
        if (mbox_register(mbox_dev) < 0)
            return -1;
    }

    return 0;
}
