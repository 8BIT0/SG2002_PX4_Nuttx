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

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static struct i2c_master_s *sg2002_i2c_register(int bus) {
    struct i2c_master_s *i2c = sg2002_i2cbus_initialize(bus);

    if (i2c)
        i2c_register(i2c, bus);

    return i2c;
}
#endif

#if defined(CONFIG_TIMER)
bool Timer_TestCallback(FAR uint32_t *next_interval_us, FAR void *arg) {
    sg2002_trace_dirout("timer 6 test\n");
    return true;
}
#endif

int sg2002_bringup(void) {

	sg2002_gpio_init();

#if defined(CONFIG_TIMER)
    // struct timer_lowerhalf_s *timer_dev = NULL;
    
    // timer_dev = sg2002_timer_initialize(6);

    // if (timer_dev && timer_dev->ops && timer_dev->ops->ioctl && timer_dev->ops->setcallback) {
    //     timer_dev->ops->ioctl(timer_dev, SG2002_Timer_Set_Freq, 10); /* set timer freq 10Hz */
    //     timer_dev->ops->setcallback(timer_dev, Timer_TestCallback, NULL);
    // }
#endif

#if defined(CONFIG_SG2002_SPI2)
    struct spi_dev_s *spi_dev = NULL;

    spi_dev = sg2002_spibus_initialize(2);

    uint8_t test_tx[4] = {0x9F, 0x00, 0x00, 0x00};
    uint8_t test_rx[4] = {0x00, 0x00, 0x00, 0x00};

    spi_dev->ops->exchange(spi_dev, test_tx, test_rx, sizeof(test_tx));
    
    for (uint8_t i = 0; i < 4; i ++) {
        sg2002_trace_dirout("spi rx %d 0x%02X\n", i, test_rx[i]);
    }
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
    if (userled_lower_initialize("/dev/test_pin") < 0)
        return -1;
#endif
#endif

#if defined(CONFIG_ARCH_BUTTONS) && defined(CONFIG_ARCH_IRQBUTTONS)
    if (btn_lower_initialize("/dev/test_exti") < 0)
        return -1;
#endif

    /* init mailbox */
    struct mbox_dev_s *mbox_dev = sg2002_mailbox_initialize();
    if (mbox_dev != NULL) {
        if (mbox_register(mbox_dev) < 0)
            return -1;
    }

    return 0;
}
