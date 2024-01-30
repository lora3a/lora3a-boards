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
 * @ingroup     drivers_bme68x
 *
 * @{
 * @file
 * @brief       Default configuration for BME68X device driver
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef BME68X_PARAMS_H
#define BME68X_PARAMS_H

#include "board.h"
#include "bme68x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the BME68X
 * @{
 */

#if MODULE_PERIPH_I2C || DOXYGEN
#ifndef BME68X_PARAM_I2C_DEV
#define BME68X_PARAM_I2C_DEV        (I2C_DEV(0))
#endif

#ifndef BME68X_PARAM_I2C_ADDR
#define BME68X_PARAM_I2C_ADDR       (BME68X_I2C_ADDR_2)
#endif
#endif /* MODULE_PERIPH_I2C */

#if MODULE_PERIPH_SPI || DOXYGEN
#ifndef BME68X_PARAM_SPI_DEV
#define BME68X_PARAM_SPI_DEV        (SPI_DEV(0))
#endif

#ifndef BME68X_PARAM_SPI_NSS_PIN
#define BME68X_PARAM_SPI_NSS_PIN    GPIO_PIN(0, 5)
#endif
#endif /* MODULE_PERIPH_SPI */

/**
 * @brief   Defaults I2C parameters if none provided
 */
#ifndef BME68X_PARAMS_I2C
#define BME68X_PARAMS_I2C                               \
{                                                       \
        .ifsel          = BME68X_I2C_INTF,              \
        .intf.i2c.dev   = BME68X_PARAM_I2C_DEV,         \
        .intf.i2c.addr  = BME68X_PARAM_I2C_ADDR,        \
}
#endif

/**
 * @brief   Defaults SPI parameters if none provided
 */
#ifndef BME68X_PARAMS_SPI
#define BME68X_PARAMS_SPI                               \
{                                                       \
        .ifsel              = BME68X_SPI_INTF,          \
        .intf.spi.dev       = BME68X_PARAM_SPI_DEV,     \
        .intf.spi.nss_pin   = BME68X_PARAM_SPI_NSS_PIN, \
}
#endif

/**
 * @brief   Configure params for BME68X
 */
static const bme68x_params_t bme68x_params[] =
{
#if MODULE_BME68X_I2C || DOXYGEN
    BME68X_PARAMS_I2C,
#endif
#if MODULE_BME68X_SPI || DOXYGEN
    BME68X_PARAMS_SPI,
#endif
};

/**
 * @brief   The number of configured sensors
 */
#define BME68X_NUMOF    ARRAY_SIZE(bme68x_params)

#ifdef __cplusplus
}
#endif

#endif /* BME68X_PARAMS_H */
/** @} */
