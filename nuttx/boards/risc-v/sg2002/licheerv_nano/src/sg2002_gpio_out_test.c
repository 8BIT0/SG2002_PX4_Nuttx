#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "chip.h"
#include "sg200x.h"

#include <arch/board/board.h>

uint32_t board_userled_initialize(void) {
    /* fake it has 10 LED on board */
    return 10;
}

void board_userled_all(uint32_t ledset) {
    UNUSED(ledset);
}

void board_autoled_off(int led) {
    UNUSED(led);
}

void board_autoled_on(int led) {
    UNUSED(led);
}

void board_userled(int led, bool state) {
    sg2002_gpioset_t pa14;
    UNUSED(led);

    pa14.field.port = SG2002_GPIO0;
    pa14.field.pin = 15;

    sg2002_gpio_write(pa14, state);
}


