#ifndef NEARDI47_GPIO_H
#define NEARDI47_GPIO_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NEARDI47_GPIO4_B0 = 136,
    NEARDI47_GPIO4_B2 = 138,
    NEARDI47_GPIO4_B0_OFFSET = 8,
    NEARDI47_GPIO4_B2_OFFSET = 10,
};

/*
 * Request GPIO4_B0/GPIO4_B2 from /dev/gpiochip4 as outputs.
 * The two arguments select their initial levels.
 *
 * Returns 0 on success, -1 on failure and sets errno.
 */
int neardi47_gpio_init(bool gpio4_b0_initial_high,
                       bool gpio4_b2_initial_high);

/* Returns 0 on success, -1 on failure and sets errno. */
int neardi47_gpio4_b0_write(bool high);
int neardi47_gpio4_b2_write(bool high);

/* Set both outputs in one GPIO ioctl. */
int neardi47_gpio_write(bool gpio4_b0_high, bool gpio4_b2_high);

/*
 * Close the GPIO line request. Both GPIOs are released automatically.
 */
void neardi47_gpio_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
