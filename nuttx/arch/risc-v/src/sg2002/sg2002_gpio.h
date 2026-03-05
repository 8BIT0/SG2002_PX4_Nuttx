#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "hardware/sg2002_gpio.h"

typedef union{
    struct {
        uint8_t port;
        uint8_t pin;
    } field;

    uint16_t pinset;
} sg2002_gpioset_t;

typedef enum {
    SG2002_GPIO0 = 0,
    SG2002_GPIO1,
    SG2002_GPIO2,
    SG2002_GPIO3,
} SG2002_GPIO_Port_List;

bool sg2002_gpio_read(sg2002_gpioset_t pinset);
void sg2002_gpio_write(sg2002_gpioset_t pinset, bool value);
void sg2002_gpio_init(void);
int sg2002_gpio_set_event(sg2002_gpioset_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);
