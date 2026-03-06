#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "chip.h"
#include "sg200x.h"

#include <arch/board/board.h>


uint32_t board_button_initialize(void) {
    /* fake it has 10 buttons on board */
    return 1;
}

uint32_t board_buttons(void) {
    return 0;
}

int board_button_irq(int id, xcpt_t irqhandler, void *arg) {
    int ret = -EINVAL;
    sg2002_gpioset_t exti_pin;

    exti_pin.field.port = SG2002_GPIO0;
    exti_pin.field.pin = 15;

    ret = sg2002_gpio_set_event(exti_pin, true, false, false, irqhandler, arg);

    return ret;
}