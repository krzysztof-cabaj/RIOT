/*
 * Copyright (C) 2023 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "board.h"
#include "periph/pio.h"

pio_program_t pio_blink_export_program(void);

int pio_blink_init(pio_program_t *blink,
                   pio_t pio, pio_sm_t sm, const pio_program_t *pro,
                   gpio_t pin);

int main(void)
{
    pio_program_t blink = pio_blink_export_program();
    pio_t pio;
    pio_sm_t sm;
    if (pio_alloc_program_sm_lock_any(&pio, &sm, &blink)) {
        puts("No PIO resources available.");
        return 1;
    }
    if (pio_blink_init(&blink, pio, sm, &blink, LED0_PIN)) {
        puts("PIO blink initialization failed");
        return 1;
    }
    pio_sm_start(pio, sm);
    return 0;
}
