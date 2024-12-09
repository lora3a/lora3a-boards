/*
 * Copyright (C) 2023 ACME
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_hdc3020 hdc3020
 * @ingroup     drivers_sensors
 * @brief       hdc3020 Drivers
 *
 * @{
 *
 * @file
 *
 */

#ifndef HDC3020_H
#define HDC3020_H

#include "saul.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CONFIG_HDC1000_I2C_ADDRESS
#define CONFIG_HDC1000_I2C_ADDRESS           (0x44)
#endif

/**
 * @name    Manufacturer ID (Texas Instruments)
 * @{
 */
#define HDC3020_MANUFACTURER_ID (0x3000)
/** @} */

#ifndef HDC3020_MEAS_DELAY
#define HDC3020_MEAS_DELAY      22000
#endif

typedef void (hdc3020_callback_t)(void *dev);

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    i2c_t i2c_dev;                      /**< I2C device which is used */
    uint8_t i2c_addr;                   /**< I2C address */
    gpio_t alert_pin;
} hdc3020_params_t;

/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
    hdc3020_params_t params;  /**< Device Parameters */
    hdc3020_callback_t *callback; /**< Alert callback */
} hdc3020_t;

extern const saul_driver_t hdc3020_saul_temp_driver;
extern const saul_driver_t hdc3020_saul_hum_driver;

enum {
    HDC3020_OK         =  0,   /**< everything was fine */
    HDC3020_ERR_BUS    = -1,   /**< bus error */
    HDC3020_ERR_NODEV  = -2,   /**< did not detect device */
    HDC3020_ERR_WRITE  = -3,   /**< write error */
    HDC3020_ERR_READ   = -4,   /**< read erro */
    HDC3020_ERR_CRC    = -5,   /**< CRC Comparison */
    HDC3020_ERR_CONFIG = -6,   /**< config error */
    HDC3020_ERR_MEAS   = -7,   /**< could not complete measure */
};

typedef enum {
    HDC3020_LOW_POWER_MODE_0 = 0,
    HDC3020_LOW_POWER_MODE_1 = 1,
    HDC3020_LOW_POWER_MODE_2 = 2,
    HDC3020_LOW_POWER_MODE_3 = 3
} hdc3020_low_power_mode_t;

typedef enum {
    HDC3020_MPS_0_5_LPM_0   = 0,
    HDC3020_MPS_0_5_LPM_1   = 1,
    HDC3020_MPS_0_5_LPM_2   = 2,
    HDC3020_MPS_0_5_LPM_3   = 3,

    HDC3020_MPS_1_LPM_0     = 4,
    HDC3020_MPS_1_LPM_1     = 5,
    HDC3020_MPS_1_LPM_2     = 6,
    HDC3020_MPS_1_LPM_3     = 7,

    HDC3020_MPS_2_LPM_0     = 8,
    HDC3020_MPS_2_LPM_1     = 9,
    HDC3020_MPS_2_LPM_2     = 10,
    HDC3020_MPS_2_LPM_3     = 11,

    HDC3020_MPS_4_LPM_0     = 12,
    HDC3020_MPS_4_LPM_1     = 13,
    HDC3020_MPS_4_LPM_2     = 14,
    HDC3020_MPS_4_LPM_3     = 15,

    HDC3020_MPS_10_LPM_0    = 16,
    HDC3020_MPS_10_LPM_1    = 17,
    HDC3020_MPS_10_LPM_2    = 18,
    HDC3020_MPS_10_LPM_3    = 19,
} hdc3020_auto_measurement_mode_t;

enum {
    HDC3020_THRESHOLDS_SET_LOW_ALERT    = 0,
    HDC3020_THRESHOLDS_SET_HIGH_ALERT   = 1,
    HDC3020_THRESHOLDS_CLEAR_LOW_ALERT  = 2,
    HDC3020_THRESHOLDS_CLEAR_HIGH_ALERT = 3,
};

typedef struct {
    uint8_t overall_alert_status : 1;
    uint8_t heater_status : 1;
    uint8_t relative_humidity_tracking_alert : 1;
    uint8_t temperature_tracking_alert : 1;
    uint8_t relative_humidity_high_tracking_alert : 1;
    uint8_t relative_humidity_low_tracking_alert : 1;
    uint8_t temperature_high_tracking_alert : 1;
    uint8_t temperature_low_tracking_alert : 1;
    uint8_t device_reset : 1;
    uint8_t checksum_verification : 1;
} hdc3020_status_register;

/**
 * @brief               Initialize the HDC302X device
 * @param[out]  dev     Device descriptor to initialized
 * @param[in]   params  Device initialization parameters
 *
 * @return      0       Success
 * @return      -ENXIO  No HDC3020 device connected to the I2C bus
 * @return      -ENODEV Device using the foo I2C address is not a HDC3020 device
 */
int hdc3020_init(hdc3020_t *dev, const hdc3020_params_t *params);

void hdc3020_deinit(const hdc3020_t *dev);

void hdc3020_set_alert_callback(hdc3020_t *dev, hdc3020_callback_t cb);

int hdc3020_trigger_on_demand_measurement(const hdc3020_t *dev,
    hdc3020_low_power_mode_t mode);

int hdc3020_fetch_on_demand_measurement(const hdc3020_t *dev,
    double *temp, double *rh);

int hdc3020_set_auto_measurement_mode(const hdc3020_t *dev,
    hdc3020_auto_measurement_mode_t mode);

int hdc3020_exit_auto_measurement_mode(const hdc3020_t *dev);

int hdc3020_auto_measurement_mode_read(const hdc3020_t *dev, double *temp,
    double *rh);

int hdc3020_minimum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *temp);

int hdc3020_maximum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *temp);

int hdc3020_minimum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *rh);

int hdc3020_maximum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *rh);

int hdc3020_set_low_alert(const hdc3020_t *dev, double temp, double rh);

int hdc3020_set_high_alert(const hdc3020_t *dev, double temp, double rh);

int hdc3020_clear_high_alert(const hdc3020_t *dev, double temp, double rh);

int hdc3020_clear_low_alert(const hdc3020_t *dev, double temp, double rh);

int hdc3020_read_set_low_alert(const hdc3020_t *dev, double *temp,
    double *rh);

int hdc3020_read_clear_low_alert(const hdc3020_t *dev, double *temp,
    double *rh);

int hdc3020_read_clear_high_alert(const hdc3020_t *dev, double *temp,
    double *rh);

int hdc3020_transfert_alert_into_nvm(const hdc3020_t *dev);

int hdc3020_deactivate_alert(const hdc3020_t *dev);

int hdc3020_enable_heater(const hdc3020_t *dev);

int hdc3020_disable_heater(const hdc3020_t *dev);

int hdc3020_configure_heater(const hdc3020_t *dev, uint8_t n_heater);

int hdc3020_get_status(const hdc3020_t *dev, uint16_t *status);

int hdc3020_clear_status(const hdc3020_t *dev);

int hdc3020_set_temp_rh_offset( const hdc3020_t *dev, double temp,
    double rh);

int hdc3020_get_temp_rh_offset(const hdc3020_t *dev, double *temp,
    double *rh);

int hdc3020_soft_reset(const hdc3020_t *dev);

int hdc3020_i2c_general_call_reset(const hdc3020_t *dev);

int hdc3020_read_nist_id(const hdc3020_t *dev, uint8_t *data, size_t len);

int hdc3020_read_manufacturer_id(const hdc3020_t *dev, uint16_t *man_id);

int hdc3020_read(const hdc3020_t *dev, double *temp, double *hum);

#ifdef __cplusplus
}
#endif

#endif /* HDC3020_H */
/** @} */
