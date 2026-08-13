#define _POSIX_C_SOURCE 200809L

#include "motion_control_pkg/suck/neardi47_gpio.h"

#include <stdio.h>
#include <time.h>

static int delay_one_second(void)
{
    struct timespec delay = {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    return nanosleep(&delay, NULL);
}

int main(void)
{
    /* Initialize GPIO4_B0 and GPIO4_B2 as low-level outputs. */
    if (neardi47_gpio_init(false, false) < 0) {
        perror("neardi47_gpio_init");
        return 1;
    }

    if (neardi47_gpio4_b0_write(true) < 0 ||
        neardi47_gpio4_b2_write(false) < 0) {
        perror("write GPIO");
        neardi47_gpio_deinit();
        return 1;
    }

    (void)delay_one_second();

    if (neardi47_gpio4_b0_write(false) < 0 ||
        neardi47_gpio4_b2_write(true) < 0) {
        perror("write GPIO");
        neardi47_gpio_deinit();
        return 1;
    }

    neardi47_gpio_deinit();
    return 0;
}
