#include "motion_control_pkg/suck/neardi47_gpio.h"

#include <errno.h>
#include <stdbool.h>

static bool gpio_initialized = false;
static bool gpio4_b0_value = false;
static bool gpio4_b2_value = false;

static int require_initialized(void)
{
    if (!gpio_initialized) {
        errno = ENODEV;
        return -1;
    }
    return 0;
}

int neardi47_gpio_init(bool gpio4_b0_initial_high,
                       bool gpio4_b2_initial_high)
{
    gpio4_b0_value = gpio4_b0_initial_high;
    gpio4_b2_value = gpio4_b2_initial_high;
    gpio_initialized = true;
    return 0;
}

int neardi47_gpio4_b0_write(bool high)
{
    if (require_initialized() < 0)
        return -1;

    gpio4_b0_value = high;
    return 0;
}

int neardi47_gpio4_b2_write(bool high)
{
    if (require_initialized() < 0)
        return -1;

    gpio4_b2_value = high;
    return 0;
}

int neardi47_gpio_write(bool gpio4_b0_high, bool gpio4_b2_high)
{
    if (require_initialized() < 0)
        return -1;

    gpio4_b0_value = gpio4_b0_high;
    gpio4_b2_value = gpio4_b2_high;
    return 0;
}

void neardi47_gpio_deinit(void)
{
    gpio_initialized = false;
    gpio4_b0_value = false;
    gpio4_b2_value = false;
}
