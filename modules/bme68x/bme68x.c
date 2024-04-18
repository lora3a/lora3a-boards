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
 * @{
 * @file
 * @brief       Bosch BME68X sensor driver implementation
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Antonio Galea <antonio.galea@gmail.com>
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
    DEBUG("\nbme68x_init: dev_id=%d, i2c_dev=%d, addr=0x%02x\n", dev_id, params->intf.i2c.dev, params->intf.i2c.addr);

    /* store interface parameters in the device for the HAL functions */
    dev->intf = params->intf;

    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    /* Select device interface and apply needed params */
    if (params->ifsel == BME68X_I2C_INTF) {
#ifdef MODULE_BME68X_I2C
        bme->intf = BME68X_I2C_INTF;
        bme->read = bme68x_i2c_read_hal;
        bme->write = bme68x_i2c_write_hal;
        bme->intf_ptr = (void *)&(dev->intf.i2c);
#else
        LOG_ERROR("[bme68x]: module bme68x_i2c not enabled\n");
        return BME68X_NO_DEV;
#endif
    }
    else {
#ifdef MODULE_BME68X_SPI
        bme->intf = BME68X_SPI_INTF;
        bme->read = bme68x_spi_read_hal;
        bme->write = bme68x_spi_write_hal;
        bme->intf_ptr = (void *)&(dev->intf.spi);
        spi_init_cs(params->intf.spi.dev, params->intf.spi.nss_pin);
#else
        LOG_ERROR("[bme68x]: module bme68x_spi not enabled\n");
        return BME68X_NO_DEV;
#endif
    }

    bme->delay_us = bme68x_us_sleep;

    /* call internal bme68x_init from Bosch Sensortech driver */
    ret = bme68x_init_internal(bme);
    if (ret != 0) {
        DEBUG("[bme68x]: Failed to get ID\n");
        return ret;
    }

    /* fetch current configuration for sensors */
    ret = bme68x_get_config(dev);
    if (ret != BME68X_OK) {
        DEBUG("[bme68x]: Failed to read sensors configuration\n");
        return ret;
    }

    DEBUG("[bme68x]: init done\n");
    return ret;
}

int bme68x_get_config(bme68x_t *dev)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    return bme68x_get_conf(&dev->config.sensors, bme);
}

int bme68x_apply_config(bme68x_t *dev)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    int8_t ret = bme68x_set_conf(&dev->config.sensors, bme);
    if (ret == BME68X_OK) {
        ret = bme68x_set_heatr_conf(dev->config.op_mode, &dev->config.heater, bme);
    }
    return ret;
}

int bme68x_start_measure(bme68x_t *dev)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    int8_t ret = bme68x_set_op_mode(dev->config.op_mode, bme);
    return ret;
}

uint32_t bme68x_get_measure_duration(bme68x_t *dev)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    return bme68x_get_meas_dur(dev->config.op_mode, &dev->config.sensors, bme);
}

int bme68x_get_measure_data(bme68x_t *dev, bme68x_data_t *data, uint8_t *n_data)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    int8_t ret = bme68x_get_data(dev->config.op_mode, data, n_data, bme);
    return ret;
}

void bme68x_wait_us(bme68x_t *dev, uint32_t del_period)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    bme->delay_us(del_period, bme->intf_ptr);
}

int bme68x_self_test(bme68x_t *dev)
{
    bme68x_dev_t *bme = &BME68X_SENSOR(dev);
    int8_t ret = bme68x_selftest_check(bme);
    return ret;
}
