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
#include "hdc3020_regs.h"

/////////////////////////////////////////////////////////////////////
// Utility functions

#define HDC3020_CRC8_POLYNOMIAL     0x31
#define HDC3020_CRC8_INITIALIZATION 0xFF
#define HDC3020_CRC8_FINAL_XOR      0x00

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = HDC3020_CRC8_INITIALIZATION;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ HDC3020_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc ^ HDC3020_CRC8_FINAL_XOR;
}

static int _write(const hdc3020_t *dev, uint8_t *cmd, size_t len)
{
    int status;
    i2c_acquire(dev->params.i2c_dev);
    status = i2c_write_bytes(
        dev->params.i2c_dev, dev->params.i2c_addr, cmd, len, 0
    );
    i2c_release(dev->params.i2c_dev);
    if (status)
        return HDC3020_ERR_WRITE;
    return HDC3020_OK;
}

static int _read(const hdc3020_t *dev, uint8_t *data, size_t len)
{
    int status;
    i2c_acquire(dev->params.i2c_dev);
    status = i2c_read_bytes(
        dev->params.i2c_dev, dev->params.i2c_addr, data, len, 0
    );
    i2c_release(dev->params.i2c_dev);
    if (status)
        return HDC3020_ERR_READ;
    for (size_t i=0; i < len - 2; i+=3)
        if (crc8(&data[i], 2) != data[i + 2])
            return HDC3020_ERR_CRC;
    return HDC3020_OK;
}

static uint16_t rh_to_hdc3020(const double hum)
{
    return hum * (0xFFFF) / 100.;
}

static uint16_t temp_to_hdc3020(const double temp)
{
    return (temp + 45) * (0xFFFF) / 175.;
}

static double rh_from_hdc3020(const uint16_t data)
{
    return (data) * 100. / (0xFFFF);
}

static double temp_from_hdc3020(const uint16_t data)
{
    return (data) * 175. / (0xFFFF) - 45;
}
/////////////////////////////////////////////////////////////////////
static void _alert_callback(void *arg) {
    hdc3020_t *dev = arg;
    if (dev->callback)
        dev->callback(arg);
}

int hdc3020_init(hdc3020_t *dev, const hdc3020_params_t *params)
{
    uint16_t manufacturer_id = 0;

    assert(dev && params);
    dev->params = *params;
    dev->callback = NULL;

    if (gpio_is_valid(dev->params.alert_pin)) {
        gpio_init_int(dev->params.alert_pin, GPIO_IN, GPIO_BOTH, _alert_callback, (void *)dev);
    }

    hdc3020_read_manufacturer_id(dev, &manufacturer_id);
    if (manufacturer_id != HDC3020_MANUFACTURER_ID)
        return HDC3020_ERR_NODEV;

    hdc3020_clear_status(dev);
    return HDC3020_OK;
}

void hdc3020_deinit(const hdc3020_t *dev)
{
    if (gpio_is_valid(dev->params.alert_pin))
        gpio_irq_disable(dev->params.alert_pin);
}

void hdc3020_set_alert_callback(hdc3020_t *dev, hdc3020_callback_t cb) {
    dev->callback = cb;
}

int hdc3020_trigger_on_demand_measurement(const hdc3020_t *dev,
    hdc3020_low_power_mode_t mode)
{
    uint8_t command[2] = {
        hdc3020_trigger_on_demand_mode_matrix[mode][0],
        hdc3020_trigger_on_demand_mode_matrix[mode][1]
    };
    return _write(dev, command, sizeof(command));
}

int hdc3020_fetch_on_demand_measurement(const hdc3020_t *dev, double *temp,
    double *rh)
{
    int status;
    uint8_t data[6];

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    if (temp)
        *temp = temp_from_hdc3020((data[0] << 8) + data[1]);

    if (rh)
        *rh =  rh_from_hdc3020((data[3] << 8) + data[4]);

    return HDC3020_OK;
}

int hdc3020_set_auto_measurement_mode(const hdc3020_t *dev,
    hdc3020_auto_measurement_mode_t mode)
{
    uint8_t command[2] = {
        hdc3020_auto_measurement_mode_matrix[mode][0],
        hdc3020_auto_measurement_mode_matrix[mode][1]
    };
    return _write(dev, command, sizeof(command));
}

int hdc3020_exit_auto_measurement_mode(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_EXIT_MSB,
        HDC3020_AUTO_MEAS_MODE_EXIT_LSB
    };
    return _write(dev, command, sizeof(command));
}

int hdc3020_auto_measurement_mode_read(const hdc3020_t *dev,
    double *temp, double *rh)
{
    int status;
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_READ_MSB,
        HDC3020_AUTO_MEAS_MODE_READ_LSB
    };
    uint8_t data[6];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if (temp)
        *temp = temp_from_hdc3020((data[0] << 8) + data[1]);

    if (rh)
        *rh =  rh_from_hdc3020((data[3] << 8) + data[4]);

    return HDC3020_OK;
}

int hdc3020_minimum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *temp)
{
    int status;
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_MIN_T_MSB,
        HDC3020_AUTO_MEAS_MODE_MIN_T_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if (temp)
        *temp = temp_from_hdc3020((data[0] << 8) + data[1]);

    return HDC3020_OK;
}

int hdc3020_maximum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *temp)
{
    int status;
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_MAX_T_MSB,
        HDC3020_AUTO_MEAS_MODE_MAX_T_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if (temp)
        *temp = temp_from_hdc3020((data[0] << 8) + data[1]);

    return HDC3020_OK;
}

int hdc3020_minimum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *rh)
{
    int status;
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_MIN_RH_MSB,
        HDC3020_AUTO_MEAS_MODE_MIN_RH_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if (rh)
        *rh = rh_from_hdc3020((data[0] << 8) + data[1]);

    return HDC3020_OK;
}

int hdc3020_maximum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *rh)
{
    int status;
    uint8_t command[2] = {
        HDC3020_AUTO_MEAS_MODE_MAX_RH_MSB,
        HDC3020_AUTO_MEAS_MODE_MAX_RH_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return HDC3020_ERR_MEAS;

    if (rh)
        *rh = rh_from_hdc3020((data[0] << 8) + data[1]);

    return HDC3020_OK;
}

int hdc3020_set_low_alert(const hdc3020_t *dev, double temp, double rh)
{
    uint16_t _temp = temp_to_hdc3020(temp);
    uint16_t _rh = rh_to_hdc3020(rh);
    uint16_t data = ((_rh & 0xFE00) + ((_temp & 0xFF80) >> 7));
    uint8_t data_unpacked[2] = {
        (uint8_t)(data >> 8), (uint8_t)(data & 0x00FF)
    };

    uint8_t command[5] = {
        HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_LSB,
        data_unpacked[0], data_unpacked[1],
        crc8(data_unpacked, sizeof(data_unpacked))
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_set_high_alert(const hdc3020_t *dev, double temp, double rh)
{
    uint16_t _temp = temp_to_hdc3020(temp);
    uint16_t _rh = rh_to_hdc3020(rh);
    uint16_t data = ((_rh & 0xFE00) + ((_temp & 0xFF80) >> 7));
    uint8_t data_unpacked[2] = {
        (uint8_t)(data >> 8), (uint8_t)(data & 0x00FF)
    };
    uint8_t command[5] = {
        HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_LSB,
        data_unpacked[0], data_unpacked[1],
        crc8(data_unpacked, sizeof(data_unpacked))
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_clear_low_alert(const hdc3020_t *dev, double temp, double rh)
{
    uint16_t _temp = temp_to_hdc3020(temp);
    uint16_t _rh = rh_to_hdc3020(rh);
    uint16_t data = ((_rh & 0xFE00) + ((_temp & 0xFF80) >> 7));
    uint8_t data_unpacked[2] = {
        (uint8_t)(data >> 8), (uint8_t)(data & 0x00FF)
    };
    uint8_t command[5] = {
        HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_LSB,
        data_unpacked[0], data_unpacked[1],
        crc8(data_unpacked, sizeof(data_unpacked))
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_clear_high_alert(const hdc3020_t *dev, double temp, double rh)
{
    uint16_t _temp = temp_to_hdc3020(temp);
    uint16_t _rh = rh_to_hdc3020(rh);
    uint16_t data = ((_rh & 0xFE00) + ((_temp & 0xFF80) >> 7));
    uint8_t data_unpacked[2] = {
        (uint8_t)(data >> 8), (uint8_t)(data & 0x00FF)
    };
    uint8_t command[5] = {
        HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_LSB,
        data_unpacked[0], data_unpacked[1],
        crc8(data_unpacked, sizeof(data_unpacked))
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_transfert_alert_into_nvm(const hdc3020_t *dev)
{
    uint8_t command[5] = {
        HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_MSB,
        HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_deactivate_alert(const hdc3020_t *dev)
{
    int status;
    uint8_t low_alert[5] = {
        HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_LSB,
        0xFF, 0xFF, 0xAC
    };
    uint8_t high_alert[5] = {
        HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_MSB,
        HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_LSB,
        0x00, 0x00, 0x81
    };

    if((status = _write(dev, low_alert, sizeof(low_alert))) != HDC3020_OK)
        return status;

    if((status = _write(dev, high_alert, sizeof(high_alert))) != HDC3020_OK)
        return status;

    return HDC3020_OK;
}

int hdc3020_read_set_low_alert(const hdc3020_t *dev, double *temp, double *rh)
{
    int status;
    uint8_t command[2] = {
        HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_MSB,
        HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_LSB
    };
    uint8_t data[4];
    uint16_t alert;

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    alert = (data[0] << 8) + data[1];

    if (rh)
        *rh = rh_from_hdc3020(alert & 0xFE00);

    if (temp)
        *temp = temp_from_hdc3020((alert & 0x01FF) << 7);

    return HDC3020_OK;
}

int hdc3020_read_set_high_alert(const hdc3020_t *dev, double *temp, double *rh)
{
    int status = 0;
    uint8_t command[2] = {
        HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_MSB,
        HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_LSB
    };
    uint8_t data[4];
    uint16_t alert;

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    alert = (data[0] << 8) + data[1];

    if (rh)
        *rh = rh_from_hdc3020(alert & 0xFE00);

    if (temp)
        *temp = temp_from_hdc3020((alert & 0x01FF) << 7);

    return HDC3020_OK;
}

int hdc3020_read_clear_low_alert(const hdc3020_t *dev, double *temp, double *rh)
{
    int status;
    uint8_t command[2] = {
        HDC3020_READ_THRESHOLDS_CLEAR_LOW_ALERT_MSB,
        HDC3020_READ_THRESHOLDS_CLEAR_LOW_ALERT_LSB
    };
    uint8_t data[4];
    uint16_t alert;

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    alert = (data[0] << 8) + data[1];

    if (rh)
        *rh = rh_from_hdc3020(alert & 0xFE00);

    if (temp)
        *temp = temp_from_hdc3020((alert & 0x01FF) << 7);

    return HDC3020_OK;
}

int hdc3020_read_clear_high_alert(const hdc3020_t *dev, double *temp, double *rh)
{
    int status = 0;
    uint8_t command[2] = {
        HDC3020_READ_THRESHOLDS_CLEAR_HIGH_ALERT_MSB,
        HDC3020_READ_THRESHOLDS_CLEAR_HIGH_ALERT_LSB
    };
    uint8_t data[4];
    uint16_t alert;

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    alert = (data[0] << 8) + data[1];

    if (rh)
        *rh = rh_from_hdc3020(alert & 0xFE00);

    if (temp)
        *temp = temp_from_hdc3020((alert & 0x01FF) << 7);

    return HDC3020_OK;
}

int hdc3020_enable_heater(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_INTEGRATED_HEATER_ENABLE_MSB,
        HDC3020_INTEGRATED_HEATER_ENABLE_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_disable_heater(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_INTEGRATED_HEATER_DISABLE_MSB,
        HDC3020_INTEGRATED_HEATER_DISABLE_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_configure_heater(const hdc3020_t *dev, uint8_t n_heater)
{
    if (n_heater < 1 || n_heater > 14) {
        return HDC3020_ERR_CONFIG;
    }
    uint16_t heater_config = (1 << n_heater) - 1;
    uint8_t data[2] = {
         (heater_config >> 8) & 0xFF, (heater_config & 0xFF)
    };

    uint8_t command[5] = {
        HDC3020_INTEGRATED_HEATER_CONFIGURE_MSB,
        HDC3020_INTEGRATED_HEATER_CONFIGURE_LSB,
        data[0], data[1], crc8(data, sizeof(data))
    };

    return _write(dev, command, sizeof(command));
}


int hdc3020_get_status(const hdc3020_t *dev, uint16_t *status_reg)
{
    int status;
    uint8_t command[2] = {
        HDC3020_STATUS_REGISTER_READ_CONTENT_MSB,
        HDC3020_STATUS_REGISTER_READ_CONTENT_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    if (status_reg)
        *status_reg = (data[0] << 8) + data[1];

    return HDC3020_OK;
}

int hdc3020_clear_status(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_STATUS_REGISTER_CLEAR_CONTENT_MSB,
        HDC3020_STATUS_REGISTER_CLEAR_CONTENT_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_set_temp_rh_offset(const hdc3020_t *dev, double temp, double rh)
{
    uint16_t _temp = temp_to_hdc3020(temp);
    uint16_t _rh = rh_to_hdc3020(rh);

    uint8_t rh_offset = ((_rh >> 7) & 0x7F);
    if (rh_offset > 0)
        rh_offset |= 0x80;

    uint8_t temp_offset = ((_temp >> 6) & 0x7F);
    if (temp_offset > 0)
        temp_offset |= 0x80;

    uint8_t offset[2] = { rh_offset, temp_offset };
    uint8_t command[5] = {
        HDC3020_PROG_READ_TEMP_RH_OFFSET_MSB,
        HDC3020_PROG_READ_TEMP_RH_OFFSET_LSB,
        offset[0], offset[1],
        crc8(offset, sizeof(offset))
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_get_temp_rh_offset(const hdc3020_t *dev, double *temp, double *rh)
{
    int status;
    uint8_t command[5] = {
        HDC3020_PROG_READ_TEMP_RH_OFFSET_MSB,
        HDC3020_PROG_READ_TEMP_RH_OFFSET_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    if (rh) {
        *rh = rh_from_hdc3020((uint8_t)(data[0] << 7));
        if (data[0] >> 7 != 1)
            *rh *= -1;
    }

    if (temp) {
        *temp = temp_from_hdc3020((uint8_t)(data[1] << 6));
        if (data[1] >> 7 != 1)
            *temp *= -1;
    }

    return HDC3020_OK;
}

int hdc3020_soft_reset(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_SOFT_RESET_MSB, HDC3020_SOFT_RESET_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_i2c_general_call_reset(const hdc3020_t *dev)
{
    uint8_t command[2] = {
        HDC3020_GENERAL_RESET_MSB, HDC3020_GENERAL_RESET_LSB
    };

    return _write(dev, command, sizeof(command));
}

int hdc3020_read_nist_id(const hdc3020_t *dev, uint8_t *data, size_t len)
{
    int status;
    uint8_t command_read_nist_id[3][2] = {
        {HDC3020_READ_NIST_ID_5_4_BYTES_MSB,
            HDC3020_READ_NIST_ID_5_4_BYTES_LSB},
        {HDC3020_READ_NIST_ID_3_2_BYTES_MSB,
            HDC3020_READ_NIST_ID_3_2_BYTES_LSB},
        {HDC3020_READ_NIST_ID_1_0_BYTES_MSB,
            HDC3020_READ_NIST_ID_1_0_BYTES_LSB}
    };
    uint8_t buffer[3];

    if (!data || len < 6)
        return HDC3020_ERR_CONFIG;

    for (int i = 0; i < 3; i++) {
        if((status = _write(dev, command_read_nist_id[i], 2)) != HDC3020_OK)
            return status;

        if ((status = _read(dev, buffer, sizeof(buffer))) != HDC3020_OK)
            return status;

        data[i * 2] = buffer[0];
        data[i * 2 + 1] = buffer[1];
    }

    return HDC3020_OK;
}

int hdc3020_read_manufacturer_id(const hdc3020_t *dev,
    uint16_t *manufacturer_id)
{
    int status;
    uint8_t command[2] = {
        HDC3020_READ_MANUFACTURER_ID_MSB, HDC3020_READ_MANUFACTURER_ID_LSB
    };
    uint8_t data[3];

    if((status = _write(dev, command, sizeof(command))) != HDC3020_OK)
        return status;

    if ((status = _read(dev, data, sizeof(data))) != HDC3020_OK)
        return status;

    if (manufacturer_id)
        *manufacturer_id = (data[0] << 8) + data[1];

    return HDC3020_OK;
}

int hdc3020_read(const hdc3020_t *dev, double *temp, double *hum)
{
    int status = hdc3020_trigger_on_demand_measurement(dev, 0);
    if (status != HDC3020_OK)
        return status;

    ztimer_sleep(ZTIMER_USEC, HDC3020_MEAS_DELAY);

    return hdc3020_fetch_on_demand_measurement(dev, temp, hum);
}
