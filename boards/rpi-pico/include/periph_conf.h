/*
 * Copyright (C) 2021 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         boards_rpi_pico
 * @{
 *
 * @file
 * @brief           Configuration of CPU peripherals for the Raspberry Pi Pico
 * @author          Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include <stdint.h>

#include "kernel_defines.h"
#include "cpu.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

static const uart_conf_t uart_config[] = {
    {
        .dev = UART0,
        .rx_pin = GPIO_PIN(0, 1),
        .tx_pin = GPIO_PIN(0, 0),
        .irqn = UART0_IRQ_IRQn
    },
    {
        .dev = UART1,
        .rx_pin = GPIO_PIN(0, 9),
        .tx_pin = GPIO_PIN(0, 8),
        .irqn = UART1_IRQ_IRQn
    }
};

#define UART_0_ISR      (isr_uart0)
#define UART_1_ISR      (isr_uart1)

#define UART_NUMOF      ARRAY_SIZE(uart_config)

static const timer_channel_conf_t timer0_channel_config[] = {
    {
        .irqn = TIMER_IRQ_0_IRQn
    },
    {
        .irqn = TIMER_IRQ_1_IRQn
    },
    {
        .irqn = TIMER_IRQ_2_IRQn
    },
    {
        .irqn = TIMER_IRQ_3_IRQn
    }
};

static const timer_conf_t timer_config[] = {
    {
        .dev = TIMER,
        .ch = timer0_channel_config,
        .ch_numof = ARRAY_SIZE(timer0_channel_config)
    }
};

#define TIMER_0_ISRA    isr_timer0
#define TIMER_0_ISRB    isr_timer1
#define TIMER_0_ISRC    isr_timer2
#define TIMER_0_ISRD    isr_timer3

#define TIMER_NUMOF     ARRAY_SIZE(timer_config)

/**
 * @name    I2C configuration
 * @{
 */
/**
 *  @brief  Number of I2C interfaces
 */
#define I2C_NUMOF       0
/** @} */

/**
 * @name    PIO configuration
 * @{
 */
/**
 * @brief   Array of PIO configurations
 */
static const pio_conf_t pio_config[] = {
    {
        .dev = PIO0,
        .irqn0 = PIO0_IRQ_0_IRQn,
        .irqn1 = PIO0_IRQ_1_IRQn
    },
    {
        .dev = PIO1,
        .irqn0 = PIO1_IRQ_0_IRQn,
        .irqn1 = PIO1_IRQ_1_IRQn
    }
};

#define PIO_0_ISR0      isr_pio00   /**< ISR name of PIO 0 IRQ 0 */
#define PIO_0_ISR1      isr_pio01   /**< ISR name of PIO 0 IRQ 1 */
#define PIO_1_ISR0      isr_pio10   /**< ISR name of PIO 1 IRQ 0 */
#define PIO_1_ISR1      isr_pio11   /**< ISR name of PIO 1 IRQ 1 */

#define PIO_NUMOF       ARRAY_SIZE(pio_config)  /**< Number of PIOs */
#define PIO_SM_NUMOF    4                       /**< Number of state machines per PIO */
#define PIO_INSTR_NUMOF 32                      /**< Maximum number of instructions */
#define PIO_IRQ_NUMOF   8                       /**< Number of interrupt flags per PIO */

#if defined(PIO_I2C_CONFIG) || defined(DOXYGEN)
/**
 * @brief   PIO I2C configuration
 *
 * PIO_I2C_CONFIG should be defined during the build process to fit
 * the users pin selection.
 */
static const pio_i2c_conf_t pio_i2c_config[] = {
    PIO_I2C_CONFIG
};
/**
 * @brief   Number of PIO I2C configurations
 */
#define PIO_I2C_NUMOF   ARRAY_SIZE(pio_i2c_config)
#else
#define pio_i2c_config  ((pio_i2c_conf_t *)NULL)
#endif
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
