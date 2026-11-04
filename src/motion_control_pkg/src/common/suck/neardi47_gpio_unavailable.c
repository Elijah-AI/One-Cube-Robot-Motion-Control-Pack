#include "motion_control_pkg/suck/neardi47_gpio.h"
#include <errno.h>

/* No Neardi pin mapping exists on the x86 controller. Fail explicitly. */
int neardi47_gpio_init(bool b0, bool b2)
{
    (void)b0; (void)b2;
    errno = ENOTSUP;
    return -1;
}
int neardi47_gpio4_b0_write(bool high)
{
    (void)high;
    errno = ENOTSUP;
    return -1;
}
int neardi47_gpio4_b2_write(bool high) { return neardi47_gpio4_b0_write(high); }
int neardi47_gpio_write(bool b0, bool b2) { return neardi47_gpio_init(b0, b2); }
void neardi47_gpio_deinit(void) {}
