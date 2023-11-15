#include <stdio.h>
#include <inttypes.h>

#include "hdc3020.h"
#include "ztimer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

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

// int hdc3020_verify_alert(const hdc3020_t *dev, double *temp, double *hum)
// {
//     int status = 0, retry = 10;
//     uint8_t command[2] = { 0xe1, 0x02 };
//     uint8_t command_read_thresholds_for_set_low_alert[2] =      { 0xe1, 0x02 };
//     uint8_t command_read_thresholds_for_set_high_alert[2] =     { 0xe1, 0x1f };
//     uint8_t command_read_thresholds_for_clear_low_alert[2] =    { 0xe1, 0x09 };
//     uint8_t command_read_thresholds_for_clear_high_alert[2] =   { 0xe1, 0x14 };
//     uint8_t command_enable_integreted_heater[2] =               { 0x30, 0x6d };
//     uint8_t command_disable_integreted_heater[2] =              { 0x30, 0x66 };
//     uint8_t data[6];

//     i2c_acquire(dev->params.i2c_dev);
//     if (i2c_write_bytes(dev->params.i2c_dev, dev->params.i2c_addr, command, sizeof(command), 0)) {
//         return HDC3020_ERR_MEAS;
//     }
//     ztimer_sleep(ZTIMER_USEC, dev->params.measure_delay);
//     do {
//         status = i2c_read_bytes(dev->params.i2c_dev, dev->params.i2c_addr, data, sizeof(data), 0);
//         if (status) {
//             retry--;
//             if (retry < 0) {
//                 return HDC3020_ERR_MEAS;
//             }
//             ztimer_sleep(ZTIMER_USEC, 100);
//         }
//     } while(status);

//     i2c_release(dev->params.i2c_dev);

//     if (temp) {
//         *temp = ((data[0] << 8) + data[1]) * 175. / (1 << 16) - 45;
//     }
//     if (hum) {
//         *hum =  ((data[3] << 8) + data[4]) * 100. / (1 << 16);
//     }
//     return HDC3020_OK;
// }

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

// CRC-8 parameters
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INITIAL     0xFF
#define CRC8_FINAL_XOR   0x00

// Function to calculate CRC-8
uint8_t calculateCRC8(const uint8_t *data, size_t size)
{
    uint8_t crc = CRC8_INITIAL;

    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else {
                crc <<= 1;
            }
        }
    }

    return crc ^ CRC8_FINAL_XOR;
}

int hdc3020_set_high_alert(const hdc3020_t *dev, uint16_t relative_humidity, int16_t temperature)
{
    uint16_t relative_humidity_bit_mask = 0b0000000001111111;
    uint16_t temperature_bit_mask =       0b0000000011111111;
    uint16_t temperature_sign_bit_mask =  0b1000000000000000;

    uint16_t relative_humidity_data = (relative_humidity & relative_humidity_bit_mask) << 9;
    uint16_t temperature_data = temperature & temperature_bit_mask;

    if (temperature & temperature_sign_bit_mask) {
        temperature_data |= 0b0000000100000000;
    }

    uint16_t data = (relative_humidity_data) + (temperature_data);

    uint8_t data_unpacked[2] = { (uint8_t)(data >> 8), (uint8_t)(data) };

    // uint8_t command_set_low_alert[5] = { 0x61, 0x00, 0xFF, 0xFF, 0xAC };
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


int hdc3020_read_alert(const hdc3020_t *dev,
                       uint16_t *low_alert, uint16_t *high_alert,
                       uint16_t *low_alert_relative_humidity, int16_t *low_alert_temperature,
                       uint16_t *high_alert_relative_humidity, int16_t *high_alert_temperature)
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

    if (low_alert) {
        *low_alert = (data[0] << 8) + data[1];
        uint16_t low_alert_relative_humidity_bit_mask = 0b1111111000000000;
        uint16_t low_alert_temperature_bit_mask =       0b0000000111111111;
        *low_alert_relative_humidity = (*low_alert & low_alert_relative_humidity_bit_mask) >> 9;
        *low_alert_temperature = (*low_alert & low_alert_temperature_bit_mask);
        if (*low_alert_temperature & 0x0100) {
            // If the sign bit is 1, the temperature is negative
            *low_alert_temperature |= 0xFF00; // Sign-extend for negative values
        }
    }

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

    if (high_alert) {
        *high_alert = (data[0] << 8) + data[1];
        uint16_t high_alert_relative_humidity_bit_mask = 0b1111111000000000;
        uint16_t high_alert_temperature_bit_mask = 0b0000000111111111;
        *high_alert_relative_humidity = (*high_alert & high_alert_relative_humidity_bit_mask) >> 9;
        *high_alert_temperature = (*high_alert & high_alert_temperature_bit_mask);
        if (*high_alert_temperature & 0x0100) {
            // If the sign bit is 1, the temperature is negative
            *high_alert_temperature |= 0xFF00; // Sign-extend for negative values
        }
    }
    return HDC3020_OK;
}
