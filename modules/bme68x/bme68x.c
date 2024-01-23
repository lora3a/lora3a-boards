/*
 * Copyright (C) 2019 Mesotic SAS
 *               2020 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_bme68x
 * @{
 * @file
 * @brief       Bosch BME68X sensor driver implementation
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @}
 */

#include <assert.h>

#include "bme68x.h"
#include "bme68x_hal.h"
#include "bme68x_params.h"

#include "log.h"

#ifdef MODULE_BME68X_I2C
#include "periph/i2c.h"
#endif

#ifdef MODULE_BME68X_SPI
#include "periph/spi.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

unsigned int bme68x_devs_numof = 0;

bme68x_t *bme68x_devs[BME68X_NUMOF] = { };

int bme68x_init(bme68x_t *dev, const bme68x_params_t *params)
{
    int8_t ret;

    assert(bme68x_devs_numof < BME68X_NUMOF);
    assert(dev);

    bme68x_devs[bme68x_devs_numof] = dev;
    unsigned int dev_id = bme68x_devs_numof++;

    /* store interface parameters in the device for the HAL functions */
    dev->intf = params->intf;

    /* Select device interface and apply needed params */
    if (params->ifsel == BME68X_I2C_INTF) {
#ifdef MODULE_BME68X_I2C
        BME68X_SENSOR(dev).intf = BME68X_I2C_INTF;
        BME68X_SENSOR(dev).read = bme68x_i2c_read_hal;
        BME68X_SENSOR(dev).write = bme68x_i2c_write_hal;
        BME68X_SENSOR(dev).intf_ptr = (void *)&(bme68x_devs[dev_id]->intf.i2c);
#else
        LOG_ERROR("[bme68x]: module bme68x_i2c not enabled\n");
        return BME68X_NO_DEV;
#endif
    }
    else {
#ifdef MODULE_BME68X_SPI
        BME68X_SENSOR(dev).intf = BME68X_SPI_INTF;
        BME68X_SENSOR(dev).read = bme68x_spi_read_hal;
        BME68X_SENSOR(dev).write = bme68x_spi_write_hal;
        BME68X_SENSOR(dev).intf_ptr = (void *)&(bme68x_devs[dev_id]->intf.spi);
        spi_init_cs(params->intf.spi.dev, params->intf.spi.nss_pin);
#else
        LOG_ERROR("[bme68x]: module bme68x_spi not enabled\n");
        return BME68X_NO_DEV;
#endif
    }

    BME68X_SENSOR(dev).delay_us = bme68x_us_sleep;

    /* call internal bme68x_init from Bosch Sensortech driver */
    ret = bme68x_init_internal(&BME68X_SENSOR(dev));
    if (ret != 0) {
        DEBUG("[bme68x]: Failed to get ID\n");
        return ret;
    }

    return ret;
}
