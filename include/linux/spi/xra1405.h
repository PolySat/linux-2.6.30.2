#ifndef XRA1405_H
#define XRA1405_H

struct timeval;
struct xra1405_platform_data {
    /**
     * Only one chip is allowed for each chipselect. The chip provides
     * 1 gpio_chip instance with 16 gpios.
     */
    // struct xra1405_chip_info    chip[1];

    /**
     * "base" is the number of the first GPIO registered. The GPIO numbers
     * are sequential.
     */
    unsigned int base;
    struct timeval *shared_irq_time;
    unsigned int level_check_interval_ms;

    /**
     * Marks the device as a interrupt controller.
     */
    // bool    irq_controller;

    /**
     * Defines which GPIOs will generate an interrupt when the status changes
     */
    // u32     irq_enabled_mask;

    /**
     * Defines which GPIOs will generate an interrupt on the rising edge
     */
    // u32     irq_rising_mask;

    /**
     * Defines which GPIOs will generate an interrupt on the falling edge
     */
    // u32     irq_falling_mask;

    // u32     irq;
};

#endif
