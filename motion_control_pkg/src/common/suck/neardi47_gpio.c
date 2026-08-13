#define _POSIX_C_SOURCE 200809L

#include "motion_control_pkg/suck/neardi47_gpio.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define NEARDI47_GPIOCHIP_DEVICE "/dev/gpiochip4"
#define GPIO4_B0_REQUEST_BIT (UINT64_C(1) << 0)
#define GPIO4_B2_REQUEST_BIT (UINT64_C(1) << 1)
#define BOTH_GPIO_REQUEST_BITS (GPIO4_B0_REQUEST_BIT | GPIO4_B2_REQUEST_BIT)

static int line_request_fd = -1;

static uint64_t initial_value_bits(bool gpio4_b0_high,
                                   bool gpio4_b2_high)
{
    uint64_t bits = 0;

    if (gpio4_b0_high)
        bits |= GPIO4_B0_REQUEST_BIT;
    if (gpio4_b2_high)
        bits |= GPIO4_B2_REQUEST_BIT;

    return bits;
}

static int set_values(uint64_t mask, uint64_t bits)
{
    struct gpio_v2_line_values values;

    if (line_request_fd < 0) {
        errno = ENODEV;
        return -1;
    }

    memset(&values, 0, sizeof(values));
    values.mask = mask;
    values.bits = bits;

    return ioctl(line_request_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
}

int neardi47_gpio_init(bool gpio4_b0_initial_high,
                       bool gpio4_b2_initial_high)
{
    struct gpio_v2_line_request request;
    int chip_fd;
    int saved_errno;

    if (line_request_fd >= 0) {
        return set_values(BOTH_GPIO_REQUEST_BITS,
                          initial_value_bits(gpio4_b0_initial_high,
                                             gpio4_b2_initial_high));
    }

    chip_fd = open(NEARDI47_GPIOCHIP_DEVICE, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0)
        return -1;

    memset(&request, 0, sizeof(request));
    request.offsets[0] = NEARDI47_GPIO4_B0_OFFSET;
    request.offsets[1] = NEARDI47_GPIO4_B2_OFFSET;
    request.num_lines = 2;
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values =
        initial_value_bits(gpio4_b0_initial_high, gpio4_b2_initial_high);
    request.config.attrs[0].mask = BOTH_GPIO_REQUEST_BITS;
    request.config.num_attrs = 1;

    (void)strncpy(request.consumer, "neardi47-gpio",
                  sizeof(request.consumer) - 1);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
        saved_errno = errno;
        (void)close(chip_fd);
        errno = saved_errno;
        return -1;
    }

    (void)close(chip_fd);
    line_request_fd = request.fd;
    return 0;
}

int neardi47_gpio4_b0_write(bool high)
{
    return set_values(GPIO4_B0_REQUEST_BIT,
                      high ? GPIO4_B0_REQUEST_BIT : 0);
}

int neardi47_gpio4_b2_write(bool high)
{
    return set_values(GPIO4_B2_REQUEST_BIT,
                      high ? GPIO4_B2_REQUEST_BIT : 0);
}

int neardi47_gpio_write(bool gpio4_b0_high, bool gpio4_b2_high)
{
    return set_values(BOTH_GPIO_REQUEST_BITS,
                      initial_value_bits(gpio4_b0_high,
                                         gpio4_b2_high));
}

void neardi47_gpio_deinit(void)
{
    if (line_request_fd >= 0) {
        (void)close(line_request_fd);
        line_request_fd = -1;
    }
}
