/*
 * Copyright (C) 2018 Mesotic SAS
 *               2024 Antonio Galea
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     pkg_driver_bme68x
 * @{
 *
 * @file
 * @brief       Abstraction layer for RIOT adaption
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef BME68X_HAL_H
#define BME68X_HAL_H

#include "periph/i2c.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_t dev;                    /**< I2C device which is used */
    uint8_t addr;                 /**< I2C address */
} bme68x_intf_i2c_t;

typedef struct {
    spi_t dev;                  /**< SPI device which is used */
    gpio_t nss_pin;             /**< Chip Select pin */
} bme68x_intf_spi_t;

void bme68x_us_sleep(uint32_t period, void *intf_ptr);

#ifdef MODULE_PERIPH_I2C

int8_t bme68x_i2c_read_hal(uint8_t reg_addr, uint8_t *reg_data,
                            uint32_t length, void *intf_ptr);
int8_t bme68x_i2c_write_hal(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t length, void *intf_ptr);
#endif

#ifdef MODULE_PERIPH_SPI

int8_t bme68x_spi_read_hal(uint8_t reg_addr, uint8_t *reg_data,
                            uint32_t length, void *intf_ptr);
int8_t bme68x_spi_write_hal(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t length, void *intf_ptr);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BME68X_HAL_H */
/** @} */
