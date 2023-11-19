#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <debug.h>

#include "ztimer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "hdc3020.h"
#include "hdc3020_utils.h"


int hdc3020_init(hdc3020_t *dev, const hdc3020_params_t *params)
{
    assert(dev && params);
    dev->params = *params;

    if (gpio_is_valid(dev->params.enable_pin)) {
        gpio_init(dev->params.enable_pin, GPIO_OUT);
        gpio_write(dev->params.enable_pin, dev->params.enable_on);
        ztimer_sleep(ZTIMER_USEC, dev->params.start_delay);
    }
    // TODO: check for sensor

    return HDC3020_OK;
}

void hdc3020_deinit(const hdc3020_t *dev)
{
    if (gpio_is_valid(dev->params.enable_pin)) {
        gpio_write(dev->params.enable_pin, 1 - dev->params.enable_on);
    }
}



int hdc3020_read(const hdc3020_t *dev, double *temp, double *hum)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { 0x24, 0x00 };
    uint8_t data[6];

    i2c_acquire(dev->params.i2c_dev);
    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr, command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr, data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);

    i2c_release(dev->params.i2c_dev);

    if (temp) {
        *temp = ((data[0] << 8) + data[1]) * 175. / (1 << 16) - 45;
    }
    if (hum) {
        *hum =  ((data[3] << 8) + data[4]) * 100. / (1 << 16);
    }
    return HDC3020_OK;
}

int hdc3020_deactivate_alert(const hdc3020_t *dev)
{
    uint8_t command_set_low_alert[5] = { 0x61, 0x00, 0xFF, 0xFF, 0xAC };
    uint8_t command_set_high_alert[5] = { 0x61, 0x1D, 0x00, 0x00, 0x81 };

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

int hdc3020_set_high_alert(const hdc3020_t *dev, double relative_humidity, double temperature)
{
    uint16_t relative_humidity_rh3020 = rh_to_rh_hdc3020(relative_humidity);
    uint16_t temperature_rh3020 = temp_to_temp_hdc3020(temperature);

    uint16_t data = (relative_humidity_rh3020 & 0b1111111000000000)
                    + ((temperature_rh3020 & 0b1111111110000000) >> 7);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8),  (uint8_t)(data & 0x00FF) };

    uint8_t command_set_high_alert[5] =
    { 0x61, 0x1D, data_unpacked[0], data_unpacked[1],
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
    { 0x61, 0x00, data_unpacked[0], data_unpacked[1],
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
    { 0x61, 0x16, data_unpacked[0], data_unpacked[1],
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
    { 0x61, 0x0B, data_unpacked[0], data_unpacked[1],
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

int hdc3020_transfert_alert_into_nvm(const hdc3020_t *dev)
{
    uint8_t command[5] = { 0x61, 0x55 };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_enable_heater(const hdc3020_t *dev)
{
    uint8_t command[2] = { 0x30, 0x6D };

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
    uint8_t command[2] = { 0x30, 0x66 };

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

    uint8_t command[5] = { 0x30, 0x6E, data[0], data[1], crc };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command) / sizeof(command[0]), 0)) {
        return HDC3020_ERR_BUS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_read_alert(const hdc3020_t *dev,
                       double *low_alert_relative_humidity, double *low_alert_temperature,
                       double *high_alert_relative_humidity, double *high_alert_temperature)
{
    int status = 0, retry = 10;
    uint8_t command_read_thresholds_for_set_low_alert[2] =  { 0xe1, 0x02 };
    uint8_t command_read_thresholds_for_set_high_alert[2] = { 0xe1, 0x1f };
    uint8_t data[4];


    i2c_acquire(dev->params.i2c_dev);
    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_read_thresholds_for_set_low_alert,
                        sizeof(command_read_thresholds_for_set_low_alert),
                        0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr, data,
                           sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
                return HDC3020_ERR_MEAS;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);


    uint16_t low_alert = (data[0] << 8) + data[1];

    *low_alert_relative_humidity =
        rh_hdc3020_to_rh(low_alert & 0b1111111000000000);
    *low_alert_temperature =
        temp_hdc3020_to_temp((low_alert & 0b0000000111111111) << 7);


    status = 0;
    retry = 10;


    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command_read_thresholds_for_set_high_alert,
                        sizeof(command_read_thresholds_for_set_high_alert),
                        0)) {
        return HDC3020_ERR_MEAS;
    }
    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
    do {
        status =
            i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr, data,
                           sizeof(data), 0);
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

int hdc3020_read_nist_id(const hdc3020_t *dev, uint8_t data[6])
{
    int status = 0, retry = 10;

    uint8_t command_read_nist_id_5_4[2] =  { 0x36, 0x83 };
    uint8_t command_read_nist_id_3_2[2] =  { 0x36, 0x84 };
    uint8_t command_read_nist_id_1_0[2] =  { 0x36, 0x85 };

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
        }, sizeof(data));
    }
    else {
        return HDC3020_ERR_NULL_POINTER;
    }


    return HDC3020_OK;
}

int hdc3020_soft_reset(const hdc3020_t *dev)
{
    uint8_t command[2] = { 0x30, 0xA2 };

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
    uint8_t command[2] = { 0x00, 0x06 };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }
    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}

int hdc3020_trigger_on_demand_measurement(const hdc3020_t *dev, int hdc3020_low_power_mode,
                                          double *relative_humidity, double *temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { 0x24, 0x00 };
    uint8_t data[6];

    switch (hdc3020_low_power_mode) {
    case 0:
        break;
    case 1:
        command[1] = 0x0B;
        break;
    case 2:
        command[1] = 0x16;
        break;
    case 3:
        command[1] = 0xFF;
        break;
    default:
        return HDC3020_ERR_LOW_POWER_MODE_CONFIG;
    }

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


int hdc3020_set_auto_measurement_mode(const hdc3020_t *dev, uint8_t auto_measurement_mode)
{
    uint8_t command[2] = { 0x00, 0x81 };

    switch (auto_measurement_mode) {
    case HDC3020_DEFAULT_LPM_MPS:
        break;
    case HDC3020_LPM_0_MPS_0_5:
        command[0] = 0x03;
        command[1] = 0xB0;
        break;
    case HDC3020_LPM_0_MPS_1:
        command[0] = 0x05;
        command[1] = 0xD2;
        break;
    case HDC3020_LPM_0_MPS_2:
        command[0] = 0x07;
        command[1] = 0x74;
        break;
    case HDC3020_LPM_0_MPS_4:
        command[0] = 0x09;
        command[1] = 0x16;
        break;
    case HDC3020_LPM_0_MPS_10:
        command[0] = 0x0B;
        command[1] = 0x09;
        break;

    case HDC3020_LPM_1_MPS_0_5:
        command[0] = 0x13;
        command[1] = 0xF3;
        break;
    case HDC3020_LPM_1_MPS_1:
        command[0] = 0x15;
        command[1] = 0x91;
        break;
    case HDC3020_LPM_1_MPS_2:
        command[0] = 0x17;
        command[1] = 0x37;
        break;
    case HDC3020_LPM_1_MPS_4:
        command[0] = 0x19;
        command[1] = 0x55;
        break;
    case HDC3020_LPM_1_MPS_10:
        command[0] = 0x1B;
        command[1] = 0x4A;
        break;

    case HDC3020_LPM_2_MPS_0_5:
        command[0] = 0x23;
        command[1] = 0x36;
        break;
    case HDC3020_LPM_2_MPS_1:
        command[0] = 0x25;
        command[1] = 0x54;
        break;
    case HDC3020_LPM_2_MPS_2:
        command[0] = 0x27;
        command[1] = 0xF2;
        break;
    case HDC3020_LPM_2_MPS_4:
        command[0] = 0x29;
        command[1] = 0x90;
        break;
    case HDC3020_LPM_2_MPS_10:
        command[0] = 0x2B;
        command[1] = 0x8F;
        break;

    case HDC3020_LPM_3_MPS_0_5:
        command[0] = 0x33;
        command[1] = 0x75;
        break;
    case HDC3020_LPM_3_MPS_1:
        command[0] = 0x35;
        command[1] = 0x17;
        break;
    case HDC3020_LPM_3_MPS_2:
        command[0] = 0x37;
        command[1] = 0xB1;
        break;
    case HDC3020_LPM_3_MPS_4:
        command[0] = 0x39;
        command[1] = 0xD3;
        break;
    case HDC3020_LPM_3_MPS_10:
        command[0] = 0x3B;
        command[1] = 0xCC;
        break;
    }

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
    uint8_t command[2] = { 0xE0, 0x00 };
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

int hdc3020_exit_auto_measurement_mode(const hdc3020_t *dev)
{
    uint8_t command[2] = { 0x30, 0x93 };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;

}

int hdc3020_minimum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *minimum_temperature)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { 0xE0, 0x01 };
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
    uint8_t command[2] = { 0xE0, 0x02 };
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
    uint8_t command[2] = { 0xE0, 0x03 };
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
    uint8_t command[2] = { 0xE0, 0x04 };
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

    uint8_t command[5] = { 0xA0, 0x04,
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
    uint8_t command[5] = { 0xA0, 0x04 };
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

int hdc3020_get_status_register(const hdc3020_t *dev, uint16_t *status_register)
{
    int status = 0, retry = 10;
    uint8_t command[2] = { 0xF3, 0x2D };
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

int hdc3020_clear_status_register(const hdc3020_t *dev, uint16_t *status_register)
{
    uint8_t command[2] = { 0x30, 0x41 };

    i2c_acquire(dev->params.i2c_dev);

    if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr,
                        command, sizeof(command), 0)) {
        return HDC3020_ERR_MEAS;
    }

    i2c_release(dev->params.i2c_dev);

    return HDC3020_OK;
}
