/*
 * Copyright (C) 2023 ACME
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_hdc3020
 * @{
 *
 * @file
 * @brief       Device driver implementation for the hdc3020
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <debug.h>

#include "ztimer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "hdc3020.h"
#include "hdc3020_utils.h"
#include "hdc3020_regs.h"


int hdc3020_init(hdc3020_t *dev, const hdc3020_params_t *params)
{
    assert(dev && params);
    dev->params = *params;

    if (gpio_is_valid(dev->params.enable_pin)) {
        gpio_init(dev->params.enable_pin, GPIO_OUT);
        gpio_write(dev->params.enable_pin, dev->params.enable_on);
        ztimer_sleep(ZTIMER_USEC, dev->params.start_delay);
    }

    uint16_t manufacturer_id = 0;

    hdc3020_read_manufacturer_id(dev, &manufacturer_id);

    if (manufacturer_id != HDC3020_MANUFACTURER_ID) {
        return HDC3020_ERR_NODEV;
    }

    return HDC3020_OK;
}

void hdc3020_deinit(const hdc3020_t *dev)
{
    if (gpio_is_valid(dev->params.enable_pin)) {
        gpio_write(dev->params.enable_pin, 1 - dev->params.enable_on);
    }
}

int hdc3020_trigger_on_demand_measurement(const hdc3020_t *dev, uint8_t hdc3020_low_power_mode,
                                          uint16_t *relative_humidity, uint16_t *temperature)
{
    if (hdc3020_low_power_mode > HDC3020_MAX_LENGHT_TRIGGER_ON_DEMAND_MODE_MATRIX) {
        return HDC3020_ERR_LOW_POWER_MODE_CONFIG;
    }

    int status = 0, retry = 10;
    uint8_t command[2] =
    { hdc3020_trigger_on_demand_mode_matrix[hdc3020_low_power_mode][0],
      hdc3020_trigger_on_demand_mode_matrix[hdc3020_low_power_mode][1] };
    uint8_t data[6];


    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]
            || calculateCRC8(&data[3], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[5]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (temperature) {
        *temperature = (data[0] << 8) + data[1];
    }
    if (relative_humidity) {
        *relative_humidity =  (data[3] << 8) + data[4];
    }

    return HDC3020_OK;
}


int hdc3020_set_auto_measurement_mode(const hdc3020_t *dev, uint8_t auto_measurement_mode)
{
    if (auto_measurement_mode > HDC3020_MAX_LENGHT_AUTO_MEASUREMENT_MODE_MATRIX) {
        return HDC3020_ERR_LOW_POWER_MODE_CONFIG;
    }
    uint8_t command[2] =
    { auto_measurement_mode_matrix[auto_measurement_mode][0],
      auto_measurement_mode_matrix[auto_measurement_mode][1] };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_exit_auto_measurement_mode(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_EXIT_MSB, HDC3020_AUTO_MEAS_MODE_EXIT_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;

}

int hdc3020_auto_measurement_mode_read(const hdc3020_t *dev,
                                       double *temperature, double *relative_humidity )
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_READ_MSB, HDC3020_AUTO_MEAS_MODE_READ_LSB };
    uint8_t data[6];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]
            || calculateCRC8(&data[3], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[5]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (temperature) {
        *temperature = temp_hdc3020_to_temp((data[0] << 8) + data[1]);
    }
    if (relative_humidity) {
        *relative_humidity =  rh_hdc3020_to_rh((data[3] << 8) + data[4]);
    }

    return HDC3020_OK;

}


int hdc3020_minimum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *minimum_temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_MIN_T_MSB, HDC3020_AUTO_MEAS_MODE_MIN_T_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (minimum_temperature) {
        *minimum_temperature = temp_hdc3020_to_temp((data[0] << 8) + data[1]);
    }

    return HDC3020_OK;

}

int hdc3020_maximum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *maximum_temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_MAX_T_MSB, HDC3020_AUTO_MEAS_MODE_MAX_T_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (maximum_temperature) {
        *maximum_temperature = temp_hdc3020_to_temp((data[0] << 8) + data[1]);
    }

    return HDC3020_OK;
}

int hdc3020_minimum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *minimum_relative_humidity)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_MIN_RH_MSB, HDC3020_AUTO_MEAS_MODE_MIN_RH_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (minimum_relative_humidity) {
        *minimum_relative_humidity = rh_hdc3020_to_rh((data[0] << 8) + data[1]);
    }

    return HDC3020_OK;

}

int hdc3020_maximum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *maximum_relative_humidity)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_AUTO_MEAS_MODE_MAX_RH_MSB, HDC3020_AUTO_MEAS_MODE_MAX_RH_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (maximum_relative_humidity) {
        *maximum_relative_humidity = rh_hdc3020_to_rh((data[0] << 8) + data[1]);
    }

    return HDC3020_OK;
}

int hdc3020_set_low_alert(const hdc3020_t *dev, double relative_humidity, double temperature)
{
    uint16_t relative_humidity_rh3020 = rh_to_rh_hdc3020(relative_humidity);
    uint16_t temperature_rh3020 = temp_to_temp_hdc3020(temperature);

    uint16_t low_alert_relative_humidity_bit_mask = 0b1111111000000000;
    uint16_t low_alert_temperature_bit_mask =       0b1111111110000000;

    uint16_t data = (relative_humidity_rh3020 & low_alert_relative_humidity_bit_mask)
                    + ((temperature_rh3020 & low_alert_temperature_bit_mask) >> 7);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8),  (uint8_t)(data & 0x00FF) };

    uint8_t command_set_low_alert[5] =
    { HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_MSB, HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_LSB,
      data_unpacked[0], data_unpacked[1],
      calculateCRC8(data_unpacked, sizeof(data_unpacked)) };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_low_alert, sizeof(command_set_low_alert), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_set_high_alert(const hdc3020_t *dev, double relative_humidity, double temperature)
{
    uint16_t relative_humidity_rh3020 = rh_to_rh_hdc3020(relative_humidity);
    uint16_t temperature_rh3020 = temp_to_temp_hdc3020(temperature);

    uint16_t data = (relative_humidity_rh3020 & 0b1111111000000000)
                    + ((temperature_rh3020 & 0b1111111110000000) >> 7);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8),  (uint8_t)(data & 0x00FF) };

    uint8_t command_set_high_alert[5] =
    { HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_MSB, HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_LSB,
      data_unpacked[0], data_unpacked[1],
      calculateCRC8(data_unpacked, sizeof(data_unpacked)) };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_high_alert, sizeof(command_set_high_alert), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_clear_low_alert(const hdc3020_t *dev, double relative_humidity, double temperature)
{
    uint16_t relative_humidity_rh3020 = rh_to_rh_hdc3020(relative_humidity);
    uint16_t temperature_rh3020 = temp_to_temp_hdc3020(temperature);

    uint16_t low_alert_relative_humidity_bit_mask = 0b1111111000000000;
    uint16_t low_alert_temperature_bit_mask =       0b1111111110000000;

    uint16_t data = (relative_humidity_rh3020 & low_alert_relative_humidity_bit_mask)
                    + ((temperature_rh3020 & low_alert_temperature_bit_mask) >> 7);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8),  (uint8_t)(data & 0x00FF) };

    uint8_t command_set_low_alert[5] =
    { HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_MSB, HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_LSB,
      data_unpacked[0], data_unpacked[1],
      calculateCRC8(data_unpacked, sizeof(data_unpacked)) };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_low_alert, sizeof(command_set_low_alert), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_clear_high_alert(const hdc3020_t *dev, double relative_humidity, double temperature)
{
    uint16_t relative_humidity_rh3020 = rh_to_rh_hdc3020(relative_humidity);
    uint16_t temperature_rh3020 = temp_to_temp_hdc3020(temperature);

    uint16_t data = (relative_humidity_rh3020 & 0b1111111000000000)
                    + ((temperature_rh3020 & 0b1111111110000000) >> 7);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8),  (uint8_t)(data & 0x00FF) };

    uint8_t command_set_high_alert[5] =
    { HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_MSB, HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_LSB,
      data_unpacked[0], data_unpacked[1],
      calculateCRC8(data_unpacked, sizeof(data_unpacked)) };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_high_alert, sizeof(command_set_high_alert), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_transfert_alert_into_nvm(const hdc3020_t *dev)
{
    uint8_t command[5] = { HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_MSB,
                           HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_deactivate_alert(const hdc3020_t *dev)
{
    uint8_t command_set_low_alert[5] = { HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_MSB,
                                         HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_LSB,
                                         0xFF, 0xFF,
                                         0xAC };
    uint8_t command_set_high_alert[5] = { HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_MSB,
                                          HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_LSB,
                                          0x00, 0x00,
                                          0x81 };


    i2c_acquire(dev->params.i2c_dev);
    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_low_alert, sizeof(command_set_low_alert), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_set_high_alert, sizeof(command_set_high_alert), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_read_set_low_alert(const hdc3020_t *dev,
                               double *low_alert_relative_humidity, double *low_alert_temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_MSB,
                           HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_LSB };
    uint8_t data[4];

    i2c_acquire(dev->params.i2c_dev);
    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                           data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    uint16_t low_alert = (data[0] << 8) + data[1];

    *low_alert_relative_humidity = rh_hdc3020_to_rh(low_alert & 0b1111111000000000);
    *low_alert_temperature = temp_hdc3020_to_temp((low_alert & 0b0000000111111111) << 7);

    return HDC3020_OK;
}

int hdc3020_read_set_high_alert(
    const hdc3020_t *dev,
    double *high_alert_relative_humidity,
    double *high_alert_temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_MSB,
                           HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_LSB };
    uint8_t data[4];

    i2c_acquire(dev->params.i2c_dev);
    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                           data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);
    i2c_release(dev->params.i2c_dev);

    uint16_t high_alert = (data[0] << 8) + data[1];

    *high_alert_relative_humidity = rh_hdc3020_to_rh(high_alert & 0b1111111000000000);
    *high_alert_temperature = temp_hdc3020_to_temp((high_alert & 0b0000000111111111) << 7);

    return HDC3020_OK;
}

int hdc3020_read_set_alert(const hdc3020_t *dev,
                           double *low_alert_relative_humidity, double *low_alert_temperature,
                           double *high_alert_relative_humidity, double *high_alert_temperature)
{
    int error = 0;

    error = hdc3020_read_set_low_alert(dev, low_alert_relative_humidity, low_alert_temperature);

    if (error != HDC3020_OK) {
        return error;
    }

    error = hdc3020_read_set_high_alert(dev, high_alert_relative_humidity, high_alert_temperature);

    if (error != HDC3020_OK) {
        return error;
    }

    return HDC3020_OK;
}


int hdc3020_enable_heater(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_INTEGRETED_HEATER_ENABLE_MSB,
                           HDC3020_INTEGRETED_HEATER_ENABLE_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command) / sizeof(command[0]), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_disable_heater(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_INTEGRETED_HEATER_DISABLE_MSB,
                           HDC3020_INTEGRETED_HEATER_DISABLE_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command) / sizeof(command[0]), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_configure_heater(const hdc3020_t *dev, uint8_t n_heater)
{
    if (n_heater < 1 || n_heater > 14) {
        return HDC3020_ERR_HEATER_CONFIG;
    }

    uint16_t heater_config = (1 << n_heater) - 1;
    uint8_t data[2] = { (heater_config >> 8) & 0xFF, (heater_config & 0xFF) };
    uint8_t crc = calculateCRC8(data, sizeof(data) / sizeof(data[0]));

    uint8_t command[5] = { HDC3020_INTEGRETED_HEATER_CONFIGURE_MSB,
                           HDC3020_INTEGRETED_HEATER_CONFIGURE_LSB,
                           data[0], data[1], crc };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command) / sizeof(command[0]), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}


int hdc3020_get_status_register(const hdc3020_t *dev, uint16_t *status_register)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_STATUS_REGISTER_READ_CONTENT_MSB,
                           HDC3020_STATUS_REGISTER_READ_CONTENT_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    *status_register = (data[0] << 8) + data[1];

    return HDC3020_OK;
}

int hdc3020_clear_status_register(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_STATUS_REGISTER_CLEAR_CONTENT_MSB,
                           HDC3020_STATUS_REGISTER_CLEAR_CONTENT_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}


int hdc3020_set_rh_temp_offset(
    const hdc3020_t *dev, double relative_humidity_offset, double temperature_offset)
{
    uint16_t rh_offset_16bit = rh_to_rh_hdc3020(relative_humidity_offset);
    uint16_t temp_offset_16bit = temp_to_temp_hdc3020(temperature_offset);

    uint8_t rh_offset = ((rh_offset_16bit >> 7) & 0x7F);

    if (relative_humidity_offset > 0) {
        rh_offset |= 0x80;
    }

    uint8_t temp_offset = ((temp_offset_16bit >> 6) & 0x7F);

    if (temperature_offset > 0) {
        temp_offset |= 0x80;
    }

    uint8_t offset[2] = { rh_offset, temp_offset };

    uint8_t command[5] = { HDC3020_PROG_READ_TEMP_RH_OFFSET_MSB,
                           HDC3020_PROG_READ_TEMP_RH_OFFSET_LSB,
                           offset[0], offset[1],
                           calculateCRC8(offset, sizeof(offset) / sizeof(offset[0])) };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_get_rh_temp_offset(
    const hdc3020_t *dev, double *relative_humidity_offset, double *temperature_offset)
{
    uint8_t command[5] = { HDC3020_PROG_READ_TEMP_RH_OFFSET_MSB,
                           HDC3020_PROG_READ_TEMP_RH_OFFSET_LSB };
    int status = 0, retry = 10;
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(&data[0], (sizeof(data) / sizeof(data[0]) / 2) - 1) != data[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    uint8_t relative_humidity_offset_8bit = (data[0] << 7);

    *relative_humidity_offset = rh_hdc3020_to_rh(relative_humidity_offset_8bit);

    if (data[0] >> 7 != 1) {
        *relative_humidity_offset *= -1;
    }

    uint8_t temperature_offset_8bit = (data[1] << 6);

    *temperature_offset = temp_hdc3020_to_temp(temperature_offset_8bit);

    if (data[1] >> 7 != 1) {
        *temperature_offset *= -1;
    }

    return HDC3020_OK;
}


int hdc3020_soft_reset(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_SOFT_RESET_MSB, HDC3020_SOFT_RESET_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_i2c_general_call_reset(const hdc3020_t *dev)
{
    uint8_t command[2] = { HDC3020_GENERAL_RESET_MSB, HDC3020_GENERAL_RESET_LSB };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}


int hdc3020_read_nist_id(const hdc3020_t *dev, uint8_t data[6])
{
    int status = 0, retry = 10;

    uint8_t command_read_nist_id_5_4[2] =  { HDC3020_READ_NIST_ID_5_4_BYTES_MSB,
                                             HDC3020_READ_NIST_ID_5_4_BYTES_LSB };
    uint8_t command_read_nist_id_3_2[2] =  { HDC3020_READ_NIST_ID_3_2_BYTES_MSB,
                                             HDC3020_READ_NIST_ID_3_2_BYTES_LSB };
    uint8_t command_read_nist_id_1_0[2] =  { HDC3020_READ_NIST_ID_1_0_BYTES_MSB,
                                             HDC3020_READ_NIST_ID_1_0_BYTES_LSB };

    uint8_t data_nist_id_5_4[3];
    uint8_t data_nist_id_3_2[3];
    uint8_t data_nist_id_1_0[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_read_nist_id_5_4,
                        sizeof(command_read_nist_id_5_4),
                        0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                           data_nist_id_5_4, sizeof(data_nist_id_5_4), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(data_nist_id_5_4,
                          sizeof(data_nist_id_5_4) / sizeof(data_nist_id_5_4[0]) - 1) !=
            data_nist_id_5_4[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_read_nist_id_3_2,
                        sizeof(command_read_nist_id_3_2),
                        0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                           data_nist_id_3_2, sizeof(data_nist_id_3_2), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(data_nist_id_3_2,
                          sizeof(data_nist_id_3_2) / sizeof(data_nist_id_3_2) - 1) !=
            data_nist_id_3_2[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_read_nist_id_1_0,
                        sizeof(command_read_nist_id_1_0),
                        0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                           &data_nist_id_1_0[3], sizeof(data_nist_id_1_0), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }

        if (calculateCRC8(data_nist_id_1_0,
                          sizeof(data_nist_id_1_0) / sizeof(data_nist_id_1_0[0]) - 1) !=
            data_nist_id_1_0[2]) {
            return HDC3020_ERR_CRC_COMPARISON;
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (data != NULL) {
        memcpy(data, (uint8_t[]){
            data_nist_id_5_4[0], data_nist_id_5_4[1],
            data_nist_id_3_2[0], data_nist_id_3_2[1],
            data_nist_id_1_0[0], data_nist_id_1_0[1]
        }, 6);
    }
    else {
        return HDC3020_ERR_NULL_POINTER;
    }


    return HDC3020_OK;
}


int hdc3020_read_manufacturer_id(const hdc3020_t *dev, uint16_t *manufacturer_id)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { HDC3020_READ_MANUFACTURER_ID_MSB, HDC3020_READ_MANUFACTURER_ID_LSB };
    uint8_t data[3];

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0) != HDC3020_OK) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                                data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (calculateCRC8(data, sizeof(data) - 1) != data[2]) {
        return HDC3020_ERR_CRC_COMPARISON;
    }

    *manufacturer_id = (data[0] << 8) + data[1];

    return HDC3020_OK;
}

int hdc3020_read(const hdc3020_t *dev, double *temp, double *hum)
{
    uint16_t relative_humidity, temperature = 0;

    hdc3020_trigger_on_demand_measurement(dev, 0, &relative_humidity, &temperature);

    if (temp) {
        *temp = temp_hdc3020_to_temp(temperature);
    }
    if (hum) {
        *hum =  rh_hdc3020_to_rh(relative_humidity);
    }

    return HDC3020_OK;
}
