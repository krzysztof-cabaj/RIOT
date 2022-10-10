/*
 * Copyright (C) 2021 Otto-von-Guericke Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_rpx0xx
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       Low-level I2C driver implementation
 *
 * @note        This driver is so far only a dummy implementation
 *              to test the PIO I2C interface.
 *
 * @author      Fabian Hüßler <fabian.huessler@ovgu.de>
 * @}
 */

#include <errno.h>

#include "periph_conf.h"
#include "periph/i2c.h"
#include "periph/pio/i2c.h"

void i2c_acquire(i2c_t dev)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        if (i2c) {
            pio_i2c_acquire(&i2c->pro);
        }
    }
}

void i2c_release(i2c_t dev)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        if (i2c) {
            pio_i2c_release(&i2c->pro);
        }
    }
}

int i2c_read_bytes(i2c_t dev, uint16_t addr, void *data,
                   size_t len, uint8_t flags)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        return i2c ? pio_i2c_read_bytes(i2c->pio, i2c->sm, addr, data, len, flags) : -EINVAL;
    }
    return -EIO;
}

int i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                  void *data, size_t len, uint8_t flags)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        return i2c ? pio_i2c_read_regs(i2c->pio, i2c->sm, addr, reg, data, len, flags) : -EINVAL;
    }
    return -EIO;
}

int i2c_read_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                 void *data, uint8_t flags)
{
    return i2c_read_regs(dev, addr, reg, data, 1, flags);
}

int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
                    size_t len, uint8_t flags)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        return i2c ? pio_i2c_write_bytes(i2c->pio, i2c->sm, addr, data, len, flags) : -EINVAL;
    }
    return -EIO;
}

int i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                   const void *data, size_t len, uint8_t flags)
{
    if (IS_USED(MODULE_PIO_I2C) && (int)dev >= I2C_NUMOF) {
        pio_i2c_obj_t *i2c = pio_i2c_get(dev - I2C_NUMOF);
        return i2c ? pio_i2c_write_regs(i2c->pio, i2c->sm, addr, reg, data, len, flags) : -EINVAL;
    }
    return -EIO;
}

int i2c_write_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                  uint8_t data, uint8_t flags)
{
    return i2c_write_regs(dev, addr, reg, &data, 1, flags);
}
