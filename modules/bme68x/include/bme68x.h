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
 * @defgroup    drivers_bme68x BME68X Temperature/Humidity/Pressure/Gas sensor
 * @ingroup     drivers_sensors
 * @brief       Driver for the Bosch BME68X sensor
 *
 * ### Introduction
 *
 * The driver allows the use of BME68X sensors connected either via I2C or SPI.
 * Instead of implementing a complete driver, it simply uses the
 * [BME68X vendor driver](https://github.com/boschsensortec/BME68x-Sensor-API)
 * written and maintained by Bosch Sensortec as a package.
 *
 * Even though this driver interface provides an easy-to-use API, the vendor's
 * driver API can still be used directly.
 *
 * All functions of the vendor's driver API require a reference to a
 * sensor device structure of type `struct bme68x_dev`. Use macro
 * @ref BME68X_SENSOR(dev) for a given device descriptor of type
 * @ref bme68x_t to the according sensor device structure of type
 * `struct bme68x_dev`, for example:
 *
 * ```c
 * bme68x_t dev;
 * ...
 * BME68X_SENSOR(dev).amb_temp = value_from_other_sensor;
 * bme68x_set_sensor_settings(BME68X_GAS_MEAS_SEL, &BME68X_SENSOR(dev));
 * ```
 *
 * Refer to the code documentation at
 * [GitHub](https://github.com/boschsensortec/BME68x-Sensor-API)
 * for detailed information on the API of the vendor driver.
 *
 * ### Sensor Operation Modes
 *
 * The BME68x sensor supports four modes: sleep mode, forced mode,
 * sequential mode and parallel mode in which measurements are taken.
 * After the power-on sequence, the sensor automatically starts in sleep mode.
 * To start a measurement, the sensor must switch to one of the other modes.
 *
 * In forced mode it performs exactly one measurement of temperature, pressure,
 * humidity and as in this order, the so-called TPHG measurement cycle. After
 * executing this TPHG measurement cycle, the raw data from the sensor is
 * available and the sensor automatically returns to sleep mode.
 *
 * In sequential mode, you can provide a heating profile comprising up to 10
 * heater steps,(each defined by a target temperature and a duration. The
 * sensor will make a THPG cycle for each step in the profile, then start
 * again from the beginning.
 *
 * Parallel mode is similar to sequential mode, but the THPG measures will be
 * done in parallel; thus when reading data one has to check if the measure was
 * indeed complete, by testing for the following bits in the status byte:
 *   status & BME68X_NEW_DATA_MSK   - new THP data
 *   status & BME68X_GASM_VALID_MSK - gas measurement valid
 *   status & BME68X_HEAT_STAB_MSK  - heater stability reached
 *
 * ### Driver Configuration
 *
 * BME68X sensors are connected either via I2C or SPI. Which interface is used
 * by which BME68X sensor is defined by the parameters in @ref bme68x_params.
 * The respective driver implementation is enabled by the modules `bme68x_i2c`
 * and `bme68x_spi`. Several BME68X sensors and a mixed configuration of I2C
 * and SPI can be used in one application.
 * ```
 * USEMODULE='bme68x_spi bme68x_i2c' make BOARD=... -C tests/drivers/bme68x
 * ```
 *
 * The vendor driver allows the use of floating point conversions. In order
 * to use these floating point conversions, module `bme68x_fp` has to
 * be enabled:
 * ```
 * USEMODULE='bme68x_fp bme68x_i2c' make BOARD=... -C tests/drivers/bme68x
 * ```
 *
 * @{
 * @file
 * @brief       Interface definition for the Bosch BME68X sensor
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef BME68X_H
#define BME68X_H

#ifdef MODULE_BME68X_FP
#define BME68X_USE_FPU
#else
#define BME68X_DO_NOT_USE_FPU
#endif

#include "bme68x_hal.h"
#include "bme68x_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief    I2C address when SDO pin is LOW
 */
#define BME68X_I2C_ADDR_1   (0x76)

/**
 * @brief    I2C address when SDO pin is HIGH
 */
#define BME68X_I2C_ADDR_2   (0x77)

/**
 * @brief   Converts a BME68X device descriptor to the BME68X sensor device
 *          structure for the vendor BME68X device driver.
 */
#define BME68X_SENSOR(d)    (*((struct bme68x_dev *)d))

/**
 * @brief   Named return values
 */
enum {
    BME68X_NULL_PTR     = -1,   /**< NULL pointer check failed. */
    BME68X_COM_FAILED   = -2,   /**< Communication with the device failed. */
    BME68X_NO_DEV       = -3,   /**< Device doesn't exist. */
    BME68X_INVALID      = -4,   /**< Invalid value or length. */
    BME68X_NO_NEW_DATA  = -5,   /**< No new data. */
};

/**
 * @brief   Shortcut type definition for BME68X sensor device structure
 * @see [struct bme68x_dev](https://github.com/boschsensortec/BME68x-Sensor-API/blob/master/bme68x_defs.h#L919-L969)
 */
typedef struct bme68x_dev bme68x_dev_t;

/**
 * @brief   BME68X Hardware interface parameters union
 */
typedef union {
    bme68x_intf_i2c_t i2c;        /**< I2C specific interface parameters */
    bme68x_intf_spi_t spi;        /**< SPI specific interface parameters */
} bme68x_intf_t;

/**
 * @brief   BME68X device configuration
 */
typedef struct {
  uint8_t op_mode;
  struct bme68x_conf sensors;
  struct bme68x_heatr_conf heater;
} bme68x_config_t;

/**
 * @brief   BME68X device initialization parameters
 */
typedef struct {
    uint8_t ifsel;              /**< Interface selection */
    bme68x_intf_t intf;         /**< Hardware interface parameters */
} bme68x_params_t;

/**
 * @brief   BME68X device descriptor
 */
typedef struct {
    struct bme68x_dev sensor; /**< Inherited device structure from vendor API */
    bme68x_intf_t intf;       /**< Device interface */
    bme68x_config_t config;   /**< Configuration */
} bme68x_t;

/**
 * @brief   BME68X data type
 * @see [struct bme68x_dev](https://github.com/boschsensortec/BME68x-Sensor-API/blob/master/bme68x_defs.h#L718-L766)
 */
typedef struct bme68x_data bme68x_data_t;

/**
 * @brief   References to BME68X sensor devices used by the HAL functions
 */
extern bme68x_t *bme68x_devs[];

/**
 * @brief   Number of initialized BME68X sensor devices in bme68x_devs
 */
extern unsigned int bme68x_devs_numof;

/**
 * @brief   Initialize the BME68X sensor.
 *
 * @param[in,out]   dev     device descriptor of the sensor to initialize
 * @param[in]       params  configuration parameters
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_init(bme68x_t *dev, const bme68x_params_t *params);

/**
 * @brief   Configure the BME68X sensor for a measure.
 *
 * @param[in]       dev     device descriptor of the sensor
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_apply_config(bme68x_t *dev);

/**
 * @brief   Start measure.
 *
 * @param[in]       dev     device descriptor of the sensor
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_start_measure(bme68x_t *dev);

/**
 * @brief   Compute measure duration
 *
 * @param[in]       dev     device descriptor of the sensor
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_get_measure_duration(bme68x_t *dev);

/**
 * @brief   Read out measure data
 *
 * @param[in]       dev     device descriptor of the sensor
 * @param[out]      data    measure data
 * @param[out]      n_data  number of measures
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_get_measure_data(bme68x_t *dev, bme68x_data_t *data, uint8_t *n_data);

/**
 * @brief   Waiting routine
 *
 * @param[in]       dev         device descriptor of the sensor
 * @param[in]       del_period  delay in microseconds
  */
void bme68x_wait_us(bme68x_t *dev, uint32_t del_period);

/**
 * @brief   Sensor self-test
 *
 * @param[in]       dev     device descriptor of the sensor
 *
 * @return 0 on success
 * @return < 0 on error
  */
int bme68x_self_test(bme68x_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BME68X_H */
/** @} */
