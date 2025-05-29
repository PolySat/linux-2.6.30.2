#ifndef XRA1405_H
#define XRA1405_H

#include <linux/types.h>

struct timeval;
struct xra1405_platform_data {
    /**
     * "base" is the number of the first GPIO registered. The GPIO numbers
     * are sequential.
     */
    unsigned int base;

    /**
     *  "shared_irq_time" is updated when an irq occures on the xra1405
     */
    struct timeval *shared_irq_time;

    /**
     *  "irq_level_trigger" has the irq (if present) use level based interrupts
     *  rather than edge based interrupts. This has no effect if no irq
     *  is provided.
     */
     int irq_level_trigger;

    /**
     *  "irq_input_filter" sets which pins should have input filters enabled
     *  when a nested interrupt is used. This has no effect if no irq is
     *  provided. Input filtering is enabled when no interrupts are used.
     */
    u16 irq_input_filter;

    /**
     * "internal_pullups" sets which pins should have internal pull-ups enabled
     */
    u16 internal_pullups;
};

#endif
