/*
 * Copyright (C) 2019 Mesotic SAS
 *               2020 Gunar Schorcht
 *               2024 Antonio Galea
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     pkg_driver_bme68x
 * @ingroup     drivers_bme68x
 * @{
 *
 * @file
 * @brief       Abstraction layer for RIOT adaption
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bme68x_hal.h"

#include "ztimer.h"

#ifndef BME68X_SPI_SPEED
#define BME68X_SPI_SPEED    (SPI_CLK_1MHZ)
#endif /* BME68X_SPI_SPEED */

#ifndef BME68X_SPI_MODE
#define BME68X_SPI_MODE     (SPI_MODE_0)
#endif /* BME68X_SPI_MODE */

void bme68x_us_sleep(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    ztimer_sleep(ZTIMER_USEC, period);
}

#ifdef MODULE_PERIPH_I2C
int8_t bme68x_i2c_read_hal(uint8_t reg_addr, uint8_t *reg_data,
                            uint32_t length, void *intf_ptr)
{
    int8_t ret;
    bme68x_intf_i2c_t* intf = (bme68x_intf_i2c_t *)intf_ptr;
    assert(intf);

    i2c_acquire(intf->dev);
    ret = i2c_read_regs(intf->dev, intf->addr, reg_addr, reg_data, length, 0);
    i2c_release(intf->dev);
    return ret;
}

int8_t bme68x_i2c_write_hal(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t length, void *intf_ptr)
{
    int8_t ret;
    bme68x_intf_i2c_t* intf = (bme68x_intf_i2c_t *)intf_ptr;
    assert(intf);

    i2c_acquire(intf->dev);
    ret = i2c_write_regs(intf->dev, intf->addr, reg_addr, reg_data, length, 0);
    i2c_release(intf->dev);
    return ret;
}
#endif /* MODULE_PERIPH_I2C */

#ifdef MODULE_PERIPH_SPI
int8_t bme68x_spi_read_hal(uint8_t reg_addr, uint8_t *reg_data,
                            uint32_t length, void *intf_ptr)
{
    bme68x_intf_spi_t* intf = (bme68x_intf_spi_t *)intf_ptr;
    assert(intf);

    unsigned int cpsr = irq_disable();

    gpio_clear(intf->nss_pin);
    spi_acquire(intf->dev, SPI_CS_UNDEF, BME68X_SPI_MODE, BME68X_SPI_SPEED);
    spi_transfer_regs(intf->dev, SPI_CS_UNDEF, reg_addr, NULL, reg_data, length);
    gpio_set(intf->nss_pin);

    irq_restore(cpsr);
    spi_release(intf->dev);
    return 0;
}

int8_t bme68x_spi_write_hal(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t length, void *intf_ptr)
{
    bme68x_intf_spi_t* intf = (bme68x_intf_spi_t *)intf_ptr;
    assert(intf);

    unsigned int cpsr = irq_disable();

    gpio_clear(intf->nss_pin);
    spi_acquire(intf->dev, SPI_CS_UNDEF, BME68X_SPI_MODE, BME68X_SPI_SPEED);
    spi_transfer_regs(intf->dev, SPI_CS_UNDEF, reg_addr, reg_data, NULL, length);
    gpio_set(intf->nss_pin);

    irq_restore(cpsr);
    spi_release(intf->dev);
    return 0;
}
#endif /* MODULE_PERIPH_SPI */
