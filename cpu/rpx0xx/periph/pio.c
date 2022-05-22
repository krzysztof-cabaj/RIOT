/*
 * Copyright (C) 2021 Otto-von-Guericke Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_rpx0xx
 * @ingroup     drivers_periph_pio
 * @{
 *
 * @file
 * @brief       PIO implementation for the RPX0XX
 * @details     The RPX0XX has 2 PIOs with 4 state machines each
 *
 * @author      Fabian Hüßler <fabian.huessler@ovgu.de>
 *
 * @}
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "periph_conf.h"
#include "periph_cpu.h"
#include "io_reg.h"
#include "pio/pio.h"
#include "periph/pio/i2c.h"
#define ENABLE_DEBUG 0
#include "debug.h"

/* ISR vectors */
static const pio_isr_vec_t *_sm_isr[PIO_NUMOF][PIO_SM_NUMOF];
/* locked state machines mask */
static atomic_uint_least32_t _sm_mask[PIO_NUMOF];
/* occupied instruction memory mask */
static atomic_uint_least32_t _instr_mask[PIO_NUMOF];

/* Since we only use one core and the rp2040 is a 32 bit MCU, do we actually need
   stdatomic here?*/
static inline bool _atomic_set_mask_u32(atomic_uint_least32_t *dst, uint32_t msk)
{
    bool exch = false;
    unsigned irq = irq_disable();
    uint32_t expect = atomic_load(dst);
    while (!(expect & msk) &&
           !(exch = atomic_compare_exchange_weak(dst, &expect, expect | msk)));
    irq_restore(irq);
    return exch;
}

static inline bool _atomic_clear_mask_u32(atomic_uint_least32_t *dst, uint32_t msk)
{
    bool exch = false;
    unsigned irq = irq_disable();
    uint32_t expect = atomic_load(dst);
    while ((expect & msk) &&
           !(exch = atomic_compare_exchange_weak(dst, &expect, expect & ~msk)));
    irq_restore(irq);
    return exch;
}

static void _irq(pio_t pio, uint32_t status)
{
    if ((status & PIO0_IRQ0_INTS_SM0_RXNEMPTY_Msk) && _sm_isr[pio][0]
                                                   && _sm_isr[pio][0]->rx_ready) {
        _sm_isr[pio][0]->rx_ready(pio, 0);
    }
    if ((status & PIO0_IRQ0_INTS_SM1_RXNEMPTY_Msk) && _sm_isr[pio][1]
                                                   && _sm_isr[pio][1]->rx_ready) {
        _sm_isr[pio][1]->rx_ready(pio, 1);
    }
    if ((status & PIO0_IRQ0_INTS_SM2_RXNEMPTY_Msk) && _sm_isr[pio][2]
                                                   && _sm_isr[pio][2]->rx_ready) {
        _sm_isr[pio][2]->rx_ready(pio, 2);
    }
    if ((status & PIO0_IRQ0_INTS_SM3_RXNEMPTY_Msk) && _sm_isr[pio][3]
                                                   && _sm_isr[pio][3]->rx_ready) {
        _sm_isr[pio][3]->rx_ready(pio, 3);
    }
    if ((status & PIO0_IRQ0_INTS_SM0_TXNFULL_Msk) && _sm_isr[pio][0]
                                                  && _sm_isr[pio][0]->tx_ready) {
        _sm_isr[pio][0]->tx_ready(pio, 0);
    }
    if ((status & PIO0_IRQ0_INTS_SM1_TXNFULL_Msk) && _sm_isr[pio][1]
                                                  && _sm_isr[pio][1]->tx_ready) {
        _sm_isr[pio][1]->tx_ready(pio, 1);
    }
    if ((status & PIO0_IRQ0_INTS_SM2_TXNFULL_Msk) && _sm_isr[pio][2]
                                                  && _sm_isr[pio][2]->tx_ready) {
        _sm_isr[pio][2]->tx_ready(pio, 2);
    }
    if ((status & PIO0_IRQ0_INTS_SM3_TXNFULL_Msk) && _sm_isr[pio][3]
                                                  && _sm_isr[pio][3]->tx_ready) {
        _sm_isr[pio][3]->tx_ready(pio, 3);
    }
    if ((status & PIO0_IRQ0_INTS_SM0_Msk) && _sm_isr[pio][0]
                                          && _sm_isr[pio][0]->sm) {
        _sm_isr[pio][0]->sm(pio, 0);
    }
    if ((status & PIO0_IRQ0_INTS_SM1_Msk) && _sm_isr[pio][1]
                                          && _sm_isr[pio][1]->sm) {
        _sm_isr[pio][1]->sm(pio, 1);
    }
    if ((status & PIO0_IRQ0_INTS_SM2_Msk) && _sm_isr[pio][2]
                                          && _sm_isr[pio][2]->sm) {
        _sm_isr[pio][2]->sm(pio, 2);
    }
    if ((status & PIO0_IRQ0_INTS_SM3_Msk) && _sm_isr[pio][3]
                                          && _sm_isr[pio][3]->sm) {
        _sm_isr[pio][3]->sm(pio, 3);
    }
}

void pio_init(pio_t pio)
{
    assert(pio <= PIO_NUMOF);
    (void)pio;
}

void pio_start_programs(void)
{
    if (IS_USED(MODULE_PIO_AUTOSTART_I2C)) {
        pio_i2c_init_programs();
        pio_i2c_start_programs();
    }
}

pio_sm_t pio_sm_lock(pio_t pio)
{
    assert(pio <= PIO_NUMOF);
    uint32_t pos = 0;
    bool exch;
    while ((pos < PIO_SM_NUMOF) && !(exch = _atomic_set_mask_u32(&_sm_mask[pio], 1u << pos))) {
        pos++;
    }
    return exch ? (pio_sm_t)pos : -1;
}

void pio_sm_unlock(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    _atomic_clear_mask_u32(&_sm_mask[pio], (1u << sm));
}

void pio_sm_start(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    io_reg_atomic_set(&dev->CTRL.reg,
                      ((1u << sm) << PIO0_CTRL_CLKDIV_RESTART_Pos) |
                      ((1u << sm) << PIO0_CTRL_SM_ENABLE_Pos));
}

void pio_sm_stop(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    io_reg_atomic_clear(&dev->CTRL.reg, (1u << sm) << PIO0_CTRL_SM_ENABLE_Pos);
}

int pio_alloc_program(pio_t pio, pio_program_t *prog)
{
    assert(pio <= PIO_NUMOF);
    if (!prog->instr_numof) {
        return 0;
    }
    if (prog->instr_numof > PIO_INSTR_NUMOF) {
        return -ENOMEM;
    }
    /* don´t ((uint32_t)1 << 32) */
    uint32_t mask = ((((uint32_t)1 << (prog->instr_numof - 1)) - 1) << 1) | 1;
    bool exch = false;
    unsigned i = 0;
    while ((i <= PIO_INSTR_NUMOF - prog->instr_numof) &&
           !(exch = _atomic_set_mask_u32(&_instr_mask[pio], mask << i))) {
        i++;
    }
    if (!exch) {
        return -ENOMEM;
    }
    prog->location = i;
    return 0;
}

int pio_alloc_program_sm_lock_any(pio_t *pio_ptr, pio_sm_t *sm_ptr, pio_program_t *program)
{
    pio_t pio;
    pio_sm_t sm = -1;
    int alloc = 0;
    if (!pio_ptr || !sm_ptr || !program) {
        return -EFAULT;
    }
    for (pio = 0; pio < PIO_NUMOF; pio++) {
        if ((alloc = pio_alloc_program(pio, program))) {
            continue;
        }
        if ((sm = pio_sm_lock(pio)) < 0) {
            pio_free_program(pio, program);
            continue;
        }
        break;
    }
    if (sm >= 0) {
        *pio_ptr = pio;
        *sm_ptr = sm;
        return 0;
    }
    return alloc ? alloc : (int)sm;
}

void pio_free_program(pio_t pio, const pio_program_t *prog)
{
    assert(pio <= PIO_NUMOF);
    if (!prog->instr_numof || prog->instr_numof > PIO_INSTR_NUMOF) {
        return;
    }
    /* don´t ((uint32_t)1 << 32) */
    uint32_t mask = ((((uint32_t)1 << (prog->instr_numof - 1)) - 1) << 1) | 1;
    mask <<= prog->location;
    _atomic_clear_mask_u32(&_instr_mask[pio], mask);
}

int pio_sm_exec(pio_t pio, pio_sm_t sm, pio_instr_t inst) {
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    if (ctrl->execctrl & PIO0_SM0_EXECCTRL_EXEC_STALLED_Msk) {
        return -EBUSY;
    }
    ctrl->instr = inst;
    return 0;
}

void pio_sm_exec_block(pio_t pio, pio_sm_t sm, pio_instr_t inst) {
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    while (ctrl->execctrl & PIO0_SM0_EXECCTRL_EXEC_STALLED_Msk);
    ctrl->instr = inst;
}

int pio_write_program(pio_t pio, const pio_program_t *prog, const pio_instr_t *prog_instr)
{
    assert(pio <= PIO_NUMOF);
    if (prog->location < 0 || prog->location > PIO_INSTR_NUMOF) {
        return -EFAULT;
    }
    PIO0_Type *dev = pio_config[pio].dev;
    for (unsigned i = 0; i < prog->instr_numof; i++) {
        pio_instr_t inst = prog_instr[i];
        /* JMPs are absolute addresses and must be adjusted to the program offset */
        if ((inst & PIO_INST_JMP_MASK) == PIO_INST_JMP) {
            inst += prog->location;
        }
        (&dev->INSTR_MEM0.reg + prog->location)[i] = inst;
    }
    return 0;
}

void pio_sm_reset(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    ctrl->clkdiv = 1u << PIO0_SM0_CLKDIV_INT_Pos;
    ctrl->execctrl = 0x1f << PIO0_SM0_EXECCTRL_WRAP_TOP_Pos;
    ctrl->shiftctrl = (1u << PIO0_SM0_SHIFTCTRL_OUT_SHIFTDIR_Pos) |
                      (1u << PIO0_SM0_SHIFTCTRL_IN_SHIFTDIR_Pos);
    ctrl->pinctrl = 5u << PIO0_SM0_PINCTRL_SET_COUNT_Pos;
}

void pio_sm_restart(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    io_reg_atomic_set(&dev->CTRL.reg,
                      ((1u << sm) << PIO0_CTRL_SM_RESTART_Pos) |
                      ((1u << sm) << PIO0_CTRL_CLKDIV_RESTART_Pos));
}

void pio_sm_set_isr_vec(pio_t pio, pio_sm_t sm, const pio_isr_vec_t *vec)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    unsigned irq = irq_disable();
    _sm_isr[pio][sm] = vec;
    irq_restore(irq);
}

void pio_sm_set_out_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base, unsigned pin_count)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->pinctrl,
                              (pin_base << PIO0_SM0_PINCTRL_OUT_BASE_Pos) |
                              (pin_count << PIO0_SM0_PINCTRL_OUT_COUNT_Pos),
                              PIO0_SM0_PINCTRL_OUT_BASE_Msk | PIO0_SM0_PINCTRL_OUT_COUNT_Msk);
}

void pio_sm_set_in_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->pinctrl,
                              pin_base << PIO0_SM0_PINCTRL_IN_BASE_Pos,
                              PIO0_SM0_PINCTRL_IN_BASE_Msk);
}

void pio_sm_set_set_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base, unsigned pin_count)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->pinctrl,
                              (pin_base << PIO0_SM0_PINCTRL_SET_BASE_Pos) |
                              (pin_count << PIO0_SM0_PINCTRL_SET_COUNT_Pos),
                              PIO0_SM0_PINCTRL_SET_BASE_Msk | PIO0_SM0_PINCTRL_SET_COUNT_Msk);
}

void pio_sm_set_sideset_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->pinctrl,
                              (pin_base << PIO0_SM0_PINCTRL_SIDESET_BASE_Pos),
                              PIO0_SM0_PINCTRL_SIDESET_BASE_Msk);
}

void pio_sm_set_sideset_count(pio_t pio, pio_sm_t sm, unsigned pin_count, bool enable)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->pinctrl,
                              ((pin_count + !!enable) << PIO0_SM0_PINCTRL_SIDESET_COUNT_Pos),
                              PIO0_SM0_PINCTRL_SIDESET_COUNT_Msk);
    io_reg_write_dont_corrupt(&ctrl->execctrl,
                              (!!enable) << PIO0_SM0_EXECCTRL_SIDE_EN_Pos,
                              PIO0_SM0_EXECCTRL_SIDE_EN_Msk);
}

void pio_sm_set_sideset_target(pio_t pio, pio_sm_t sm, bool pindir)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->execctrl,
                              (!!pindir) << PIO0_SM0_EXECCTRL_SIDE_PINDIR_Pos,
                              PIO0_SM0_EXECCTRL_SIDE_PINDIR_Msk);
}

pio_sm_clkdiv_t pio_sm_clkdiv(uint32_t f_hz)
{
    uint32_t div = CLOCK_CORECLOCK / f_hz;
    uint32_t frac = (((uint64_t)100 * CLOCK_CORECLOCK) / f_hz) - (100 * div);
    assert(div > 0);
    assert((div < PIO_SM_CLKDIV_MAX) || (div == PIO_SM_CLKDIV_MAX && frac == 0));
    if (div >= PIO_SM_CLKDIV_MAX) {
        div = 0;
        frac = 0;
    }
    return (pio_sm_clkdiv_t){ .div = div, .frac_100 = frac };
}

void pio_sm_set_clkdiv(pio_t pio, pio_sm_t sm, pio_sm_clkdiv_t clk)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    if (clk.div == 0) {
        clk.frac_100 = 0;
        /* 0 --> 65536 */
    }
    if (clk.frac_100 >= 100) {
        clk.frac_100 /= 10;
    }
    uint32_t frac = ((((uint16_t)clk.frac_100) * 256) + 50) / 100;
    uint32_t val = (clk.div << PIO0_SM0_CLKDIV_INT_Pos) | (frac << PIO0_SM0_CLKDIV_FRAC_Pos);
    uint32_t msk = PIO0_SM0_CLKDIV_INT_Msk | PIO0_SM0_CLKDIV_FRAC_Msk;
    io_reg_write_dont_corrupt(&ctrl->clkdiv, val, msk);
}

void pio_sm_clkdiv_restart(pio_t pio, unsigned sm_mask)
{
    assert(pio <= PIO_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    io_reg_atomic_set(&dev->CTRL.reg, (sm_mask & PIO_SM_ALL) << PIO0_CTRL_CLKDIV_RESTART_Pos);
}

void pio_sm_set_wrap(pio_t pio, pio_sm_t sm, unsigned prog_loc, uint8_t top, uint8_t bottom)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->execctrl,
                              ((prog_loc + bottom) << PIO0_SM0_EXECCTRL_WRAP_BOTTOM_Pos) |
                              ((prog_loc + top) << PIO0_SM0_EXECCTRL_WRAP_TOP_Pos),
                              PIO0_SM0_EXECCTRL_WRAP_BOTTOM_Msk | PIO0_SM0_EXECCTRL_WRAP_TOP_Msk);
}

void pio_sm_set_jmp_pin(pio_t pio, pio_sm_t sm, gpio_t pin)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->execctrl,
                              pin << PIO0_SM0_EXECCTRL_JMP_PIN_Pos,
                              PIO0_SM0_EXECCTRL_JMP_PIN_Msk);
}

void pio_sm_set_in_shift(pio_t pio, pio_sm_t sm, bool right, bool autopush, unsigned threshold)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->shiftctrl,
                              ((!!right) << PIO0_SM0_SHIFTCTRL_IN_SHIFTDIR_Pos) |
                              ((!!autopush) << PIO0_SM0_SHIFTCTRL_AUTOPUSH_Pos) |
                              ((threshold % 32) << PIO0_SM0_SHIFTCTRL_PUSH_THRESH_Pos),
                              PIO0_SM0_SHIFTCTRL_IN_SHIFTDIR_Msk |
                              PIO0_SM0_SHIFTCTRL_AUTOPUSH_Msk |
                              PIO0_SM0_SHIFTCTRL_PUSH_THRESH_Msk);
}

void pio_sm_set_out_shift(pio_t pio, pio_sm_t sm, bool right, bool autopull, unsigned threshold)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->shiftctrl,
                              ((!!right) << PIO0_SM0_SHIFTCTRL_OUT_SHIFTDIR_Pos) |
                              ((!!autopull) << PIO0_SM0_SHIFTCTRL_AUTOPULL_Pos) |
                              ((threshold % 32) << PIO0_SM0_SHIFTCTRL_PULL_THRESH_Pos),
                              PIO0_SM0_SHIFTCTRL_OUT_SHIFTDIR_Msk |
                              PIO0_SM0_SHIFTCTRL_AUTOPULL_Msk |
                              PIO0_SM0_SHIFTCTRL_PULL_THRESH_Msk);
}

void pio_sm_set_fifo_join_rx(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->shiftctrl,
                              1u << PIO0_SM0_SHIFTCTRL_FJOIN_RX_Pos,
                              PIO0_SM0_SHIFTCTRL_FJOIN_RX_Msk | PIO0_SM0_SHIFTCTRL_FJOIN_TX_Msk);
}

void pio_sm_set_fifo_join_tx(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_write_dont_corrupt(&ctrl->shiftctrl,
                              1u << PIO0_SM0_SHIFTCTRL_FJOIN_TX_Pos,
                              PIO0_SM0_SHIFTCTRL_FJOIN_RX_Msk | PIO0_SM0_SHIFTCTRL_FJOIN_TX_Msk);
}

void pio_sm_reset_fifos(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_atomic_clear(&ctrl->shiftctrl,
                        (1u << PIO0_SM0_SHIFTCTRL_FJOIN_TX_Pos) |
                        (1u << PIO0_SM0_SHIFTCTRL_FJOIN_RX_Pos));
}

void pio_sm_clear_fifos(pio_t pio, pio_sm_t sm)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    pio_sm_ctrl_regs_t *ctrl = &PIO_SM_CTRL_BASE(dev)[sm];
    io_reg_atomic_xor(&ctrl->shiftctrl, (1u << PIO0_SM0_SHIFTCTRL_FJOIN_RX_Pos));
    io_reg_atomic_xor(&ctrl->shiftctrl, (1u << PIO0_SM0_SHIFTCTRL_FJOIN_RX_Pos));
}

void pio_sm_irq_enable(pio_t pio, pio_sm_t sm, unsigned irq_mask)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    if (irq_mask & (PIO_SM_IRQ0_SM | PIO_SM_IRQ0_TXNFULL | PIO_SM_IRQ0_RXNEMPTY)) {
        NVIC_EnableIRQ(pio_config[pio].irqn0);
        io_reg_atomic_set(&dev->IRQ0_INTE.reg,
                          ((!!(irq_mask & PIO_SM_IRQ0_SM)) << (PIO0_IRQ0_INTE_SM0_Pos + sm)) |
                          ((!!(irq_mask & PIO_SM_IRQ0_TXNFULL)) << (PIO0_IRQ0_INTE_SM0_TXNFULL_Pos + sm)) |
                          ((!!(irq_mask & PIO_SM_IRQ0_RXNEMPTY)) << (PIO0_IRQ0_INTE_SM0_RXNEMPTY_Pos + sm)));
    }
    if (irq_mask & (PIO_SM_IRQ1_SM | PIO_SM_IRQ1_TXNFULL | PIO_SM_IRQ1_RXNEMPTY)) {
        NVIC_EnableIRQ(pio_config[pio].irqn1);
        io_reg_atomic_set(&dev->IRQ1_INTE.reg,
                          ((!!(irq_mask & PIO_SM_IRQ1_SM)) << (PIO0_IRQ1_INTE_SM0_Pos + sm)) |
                          ((!!(irq_mask & PIO_SM_IRQ1_TXNFULL)) << (PIO0_IRQ1_INTE_SM0_TXNFULL_Pos + sm)) |
                          ((!!(irq_mask & PIO_SM_IRQ1_RXNEMPTY)) << (PIO0_IRQ1_INTE_SM0_RXNEMPTY_Pos + sm)));
    }
}

void pio_sm_irq_disable(pio_t pio, pio_sm_t sm, unsigned irq_mask)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    if (irq_mask & (PIO_SM_IRQ0_SM | PIO_SM_IRQ0_TXNFULL | PIO_SM_IRQ0_RXNEMPTY)) {
        NVIC_DisableIRQ(pio_config[pio].irqn0);
        io_reg_atomic_clear(&dev->IRQ0_INTE.reg,
                            ((!!(irq_mask & PIO_SM_IRQ0_SM)) << (PIO0_IRQ0_INTE_SM0_Pos + sm)) |
                            ((!!(irq_mask & PIO_SM_IRQ0_TXNFULL)) << (PIO0_IRQ0_INTE_SM0_TXNFULL_Pos + sm)) |
                            ((!!(irq_mask & PIO_SM_IRQ0_RXNEMPTY)) << (PIO0_IRQ0_INTE_SM0_RXNEMPTY_Pos + sm)));
    }
    if (irq_mask & (PIO_SM_IRQ1_SM | PIO_SM_IRQ1_TXNFULL | PIO_SM_IRQ1_RXNEMPTY)) {
        NVIC_DisableIRQ(pio_config[pio].irqn1);
        io_reg_atomic_clear(&dev->IRQ1_INTE.reg,
                            ((!!(irq_mask & PIO_SM_IRQ1_SM)) << (PIO0_IRQ1_INTE_SM0_Pos + sm)) |
                            ((!!(irq_mask & PIO_SM_IRQ1_TXNFULL)) << (PIO0_IRQ1_INTE_SM0_TXNFULL_Pos + sm)) |
                            ((!!(irq_mask & PIO_SM_IRQ1_RXNEMPTY)) << (PIO0_IRQ1_INTE_SM0_RXNEMPTY_Pos + sm)));
    }
}

int pio_sm_transmit_word(pio_t pio, pio_sm_t sm, uint32_t word)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    if (pio_sm_tx_fifo_full(pio, sm)) {
        return -EBUSY;
    }
    (&dev->TXF0)[sm] = word;
    return 0;
}

void pio_sm_transmit_word_block(pio_t pio, pio_sm_t sm, uint32_t word)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    while (pio_sm_tx_fifo_full(pio, sm));
    (&dev->TXF0)[sm] = word;
}

void pio_sm_transmit_words_block(pio_t pio, pio_sm_t sm, const uint32_t *words, unsigned count)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    while (count--) {
        pio_sm_transmit_word_block(pio, sm, *words++);
    }
}

int pio_sm_receive_word(pio_t pio, pio_sm_t sm, uint32_t *word)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    if (pio_sm_rx_fifo_empty(pio, sm)) {
        return -ENODATA;
    }
    uint32_t w = (&dev->RXF0)[sm];
    if (word) {
        *word = w;
    }
    return 0;
}

void pio_sm_receive_word_block(pio_t pio, pio_sm_t sm, uint32_t *word)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    while (pio_sm_rx_fifo_empty(pio, sm));
    uint32_t w = (&dev->RXF0)[sm];
    if (word) {
        *word = w;
    }
}

void pio_sm_receive_words_block(pio_t pio, pio_sm_t sm, uint32_t *word, unsigned count)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    while (count--) {
        pio_sm_receive_word_block(pio, sm, !word ? word : word++);
    }
}

uint32_t pio_irq_get(pio_t pio)
{
    assert(pio <= PIO_NUMOF);
    return pio_config[pio].dev->IRQ.reg;
}

void pio_irq_clear(pio_t pio, unsigned irq_flags)
{
    assert(pio <= PIO_NUMOF);
    pio_config[pio].dev->IRQ.reg = irq_flags & ((1u << PIO_IRQ_NUMOF) - 1);
}

int pio_sm_init_common(pio_t pio, pio_sm_t sm,
                       const pio_program_t *prog,
                       const pio_program_conf_t *conf)
{
    assert(pio <= PIO_NUMOF);
    assert((unsigned)sm < PIO_SM_NUMOF);
    if (prog->location < 0                      ||
        prog->location >= PIO_INSTR_NUMOF       ||
        prog->instr_numof > PIO_INSTR_NUMOF     ||
        conf->pc_start >= prog->instr_numof     ||
        conf->wrap_top >= prog->instr_numof     ||
        conf->wrap_bottom >= prog->instr_numof  ||
        conf->wrap_bottom > conf->wrap_top) {
        return -ECANCELED;
    }
    pio_sm_reset(pio, sm);
    pio_sm_clear_debug_txstall(pio, PIO_SM_MASK(sm));
    pio_sm_clear_debug_txover(pio, PIO_SM_MASK(sm));
    pio_sm_clear_debug_rxunder(pio, PIO_SM_MASK(sm));
    pio_sm_clear_debug_rxstall(pio, PIO_SM_MASK(sm));
    pio_sm_set_wrap(pio, sm, prog->location, conf->wrap_top, conf->wrap_bottom);
    pio_sm_set_sideset_count(pio, sm, conf->sideset_count, conf->sideset_optional);
    pio_sm_set_sideset_target(pio, sm, conf->sideset_pindirs);
    pio_sm_set_isr_vec(pio, sm, NULL);
    pio_sm_irq_disable(pio, sm, PIO_SM_IRQ_ALL);
    pio_sm_exec(pio, sm, pio_inst_jmp(PIO_INST_JMP_COND_NONE, (prog->location + conf->pc_start)));
    return 0;
}

void pio_print_status(pio_t pio)
{
    assert(pio <= PIO_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    /* FIFO status */
    uint32_t stat = dev->FSTAT.reg;
    uint8_t set;
    set = (stat & (PIO_SM_ALL << PIO0_FSTAT_TXEMPTY_Pos) >> PIO0_FSTAT_TXEMPTY_Pos);
    printf("TXEMPTY: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (stat & (PIO_SM_ALL << PIO0_FSTAT_TXFULL_Pos) >> PIO0_FSTAT_TXFULL_Pos);
    printf("TXFULL: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (stat & (PIO_SM_ALL << PIO0_FSTAT_RXEMPTY_Pos) >> PIO0_FSTAT_RXEMPTY_Pos);
    printf("RXEMPTY: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (stat & (PIO_SM_ALL << PIO0_FSTAT_RXFULL_Pos) >> PIO0_FSTAT_RXFULL_Pos);
    printf("RXFULL: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    /* program status */
    printf("ADDR: SM0=%"PRIu32" SM1=%"PRIu32" SM2=%"PRIu32" SM3=%"PRIu32"\n",
           dev->SM0_ADDR.reg, dev->SM1_ADDR.reg, dev->SM2_ADDR.reg, dev->SM3_ADDR.reg);
    printf("INSTR: SM0=%"PRIu32" SM1=%"PRIu32" SM2=%"PRIu32" SM3=%"PRIu32"\n",
           dev->SM0_INSTR.reg, dev->SM1_INSTR.reg, dev->SM2_INSTR.reg, dev->SM3_INSTR.reg);
}

void pio_print_debug(pio_t pio)
{
    assert(pio <= PIO_NUMOF);
    PIO0_Type *dev = pio_config[pio].dev;
    uint32_t debug = dev->FDEBUG.reg;
    uint8_t set;
    set = (debug & (PIO_SM_ALL << PIO0_FDEBUG_TXSTALL_Pos) >> PIO0_FDEBUG_TXSTALL_Pos);
    printf("TXSTALL: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (debug & (PIO_SM_ALL << PIO0_FDEBUG_TXOVER_Pos) >> PIO0_FDEBUG_TXOVER_Pos);
    printf("TXOVER: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (debug & (PIO_SM_ALL << PIO0_FDEBUG_RXUNDER_Pos) >> PIO0_FDEBUG_RXUNDER_Pos);
    printf("RXUNDER: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
    set = (debug & (PIO_SM_ALL << PIO0_FDEBUG_RXSTALL_Pos) >> PIO0_FDEBUG_RXSTALL_Pos);
    printf("RXSTALL: SM0=%d SM1=%d SM2=%d SM3=%d\n",
           set & PIO_SM0, set & PIO_SM1, set & PIO_SM2, set & PIO_SM3);
}

void PIO_0_ISR0(void)
{
    uint32_t irq = PIO0->IRQ0_INTS.reg;
    _irq(0, irq);
    cortexm_isr_end();
}

void PIO_0_ISR1(void)
{
    uint32_t irq = PIO0->IRQ1_INTS.reg;
    _irq(0, irq);
    cortexm_isr_end();
}

void PIO_1_ISR0(void)
{
    uint32_t irq = PIO1->IRQ0_INTS.reg;
    _irq(1, irq);
    cortexm_isr_end();
}

void PIO_1_ISR1(void)
{
    uint32_t irq = PIO1->IRQ1_INTS.reg;
    _irq(1, irq);
    cortexm_isr_end();
}
