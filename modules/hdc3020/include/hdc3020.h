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

typedef struct {
    i2c_t i2c_dev;                      /**< I2C device which is used */
    uint8_t i2c_addr;                   /**< I2C address */
    gpio_t enable_pin;
    uint8_t enable_on;
    uint32_t start_delay;
    uint32_t measure_delay;
} hdc3020_params_t;

typedef struct {
    hdc3020_params_t params;  /**< Device Parameters */
} hdc3020_t;

extern const saul_driver_t hdc3020_saul_temp_driver;
extern const saul_driver_t hdc3020_saul_hum_driver;

enum {
    HDC3020_OK                          =  0,   /**< everything was fine */
    HDC3020_ERR_BUS                     = -1,   /**< bus error */
    HDC3020_ERR_NODEV                   = -2,   /**< did not detect device */
    HDC3020_ERR_MEAS                    = -3,   /**< could not start measure */
    HDC3020_ERR_HEATER_CONFIG           = -4,   /**< heater config (0 - 14) */
    HDC3020_ERR_CRC_COMPARISON          = -5,   /**< CRC Comparison */
    HDC3020_ERR_NULL_POINTER            = -6,   /**< Output data pointer is NULL */
    HDC3020_ERR_LOW_POWER_MODE_CONFIG   = -7    /**< Low Power Mode can be only 0, 1, 2 or 3 */
};

enum {
    HDC3020_LOW_POWER_MODE_0    = 0,
    HDC3020_LOW_POWER_MODE_1    = 1,
    HDC3020_LOW_POWER_MODE_2    = 2,
    HDC3020_LOW_POWER_MODE_3    = 3
};

enum {
    HDC3020_DEFAULT_LPM_MPS = 0,
    HDC3020_LPM_0_MPS_0_5,
    HDC3020_LPM_0_MPS_1,
    HDC3020_LPM_0_MPS_2,
    HDC3020_LPM_0_MPS_4,
    HDC3020_LPM_0_MPS_10,

    HDC3020_LPM_1_MPS_0_5,
    HDC3020_LPM_1_MPS_1,
    HDC3020_LPM_1_MPS_2,
    HDC3020_LPM_1_MPS_4,
    HDC3020_LPM_1_MPS_10,

    HDC3020_LPM_2_MPS_0_5,
    HDC3020_LPM_2_MPS_1,
    HDC3020_LPM_2_MPS_2,
    HDC3020_LPM_2_MPS_4,
    HDC3020_LPM_2_MPS_10,

    HDC3020_LPM_3_MPS_0_5,
    HDC3020_LPM_3_MPS_1,
    HDC3020_LPM_3_MPS_2,
    HDC3020_LPM_3_MPS_4,
    HDC3020_LPM_3_MPS_10,
};

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
}StatusRegister;

int hdc3020_init(hdc3020_t *dev, const hdc3020_params_t *params);
void hdc3020_deinit(const hdc3020_t *dev);

int hdc3020_trigger_on_demand_measurement(
    const hdc3020_t *dev, int hdc3020_low_power_mode,
    uint16_t *relative_humidity, uint16_t *temperature);

int hdc3020_set_auto_measurement_mode(
    const hdc3020_t *dev, uint8_t auto_measurement_mode);

int hdc3020_exit_auto_measurement_mode(const hdc3020_t *dev);

int hdc3020_auto_measurement_mode_read(
    const hdc3020_t *dev,
    double *temperature, double *relative_humidity );

int hdc3020_minimum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *minimum_temperature);


int hdc3020_maximum_temperature_auto_measurement_mode_read(
    const hdc3020_t *dev, double *maximum_temperature);

int hdc3020_minimum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *minimum_relative_humidity);

int hdc3020_maximum_relative_humidity_auto_measurement_mode_read(
    const hdc3020_t *dev, double *maximum_relative_humidity);

int hdc3020_set_high_alert(
    const hdc3020_t *dev,
    double relative_humidity, double temperature);

int hdc3020_set_low_alert(
    const hdc3020_t *dev,
    double relative_humidity, double temperature);

int hdc3020_clear_high_alert(const hdc3020_t *dev, double relative_humidity, double temperature);

int hdc3020_clear_low_alert(const hdc3020_t *dev, double relative_humidity, double temperature);

int hdc3020_transfert_alert_into_nvm(const hdc3020_t *dev);
int hdc3020_deactivate_alert(const hdc3020_t *dev);

int hdc3020_read_alert(const hdc3020_t *dev,
                       double *low_alert_relative_humidity, double *low_alert_temperature,
                       double *high_alert_relative_humidity, double *high_alert_temperature);

int hdc3020_enable_heater(const hdc3020_t *dev);

int hdc3020_disable_heater(const hdc3020_t *dev);

int hdc3020_configure_heater(const hdc3020_t *dev, uint8_t n_heater);

int hdc3020_get_status_register(const hdc3020_t *dev, uint16_t *status_register);

int hdc3020_clear_status_register(const hdc3020_t *dev);

int hdc3020_set_rh_temp_offset(
    const hdc3020_t *dev, double relative_humidity_offset, double temperature_offset);

int hdc3020_get_rh_temp_offset(
    const hdc3020_t *dev, double *relative_humidity_offset, double *temperature_offset);
int hdc3020_soft_reset(const hdc3020_t *dev);
int hdc3020_i2c_general_call_reset(const hdc3020_t *dev);

int hdc3020_read_nist_id(const hdc3020_t *dev, uint8_t data[6]);

int hdc3020_read_manufacturer_id(const hdc3020_t *dev, uint16_t *manufacturer_id);


int hdc3020_read(const hdc3020_t *dev, double *temp, double *hum);

#ifdef __cplusplus
}
#endif

#endif /* HDC3020_H */
