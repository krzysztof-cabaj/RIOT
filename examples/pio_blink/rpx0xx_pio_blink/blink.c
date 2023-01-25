/*
 * Copyright (C) 2023 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <errno.h>
#include "pio/pio.h"
#include "blink.pio.h"

static void pio_blink_init_pins(pio_t pio, pio_sm_t sm, gpio_t pin)
{
    pio_gpio_init_t init = {
        .gpio_direction = 0x01,
        .gpio_base = pin,
        .gpio_count = 1
    };
    pio_sm_set_set_pins_init(pio, sm, &init);
}

pio_program_t pio_blink_export_program(void)
{
    return pio_blink_create_program();
}

int pio_blink_init(pio_program_t *blink,
                   pio_t pio, pio_sm_t sm, const pio_program_t *pro,
                   gpio_t pin)
{
    pio_instr_t instr[] = PIO_BLINK_PROGRAM;
    pio_program_conf_t conf = PIO_BLINK_PROGRAM_CONF;
    int ret;
    if (pin >= 32) {
        return -ENOTSUP;
    }
    if ((ret = pio_write_program(pio, pro, instr))) {
        return ret;
    }
    if ((ret = pio_sm_init_common(pio, sm, pro, &conf))) {
        return ret;
    }
    pio_sm_set_clkdiv(pio, sm, pio_sm_clkdiv(2000));
    pio_blink_init_pins(pio, sm, pin);
    pio_sm_restart(pio, sm);
    *blink = *pro;
    return 0;
}
