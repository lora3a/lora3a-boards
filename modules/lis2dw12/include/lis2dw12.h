#ifndef LIS2DW12_H
#define LIS2DW12_H

#include "saul.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "lis2dw12_reg.h"

typedef struct {
    i2c_t i2c_dev;                      /**< I2C device which is used */
    uint8_t i2c_addr;                   /**< I2C address */
    gpio_t power_pin;
} lis2dw12_params_t;

typedef struct {
    lis2dw12_params_t params;  /**< Device Parameters */
    stmdev_ctx_t ctx;
} lis2dw12_t;

enum {
    LIS2DW12_OK = 0,        /**< everything was fine */
    LIS2DW12_ERR_NO_DEV,    /**< did no detect device */
};

extern const saul_driver_t lis2dw12_saul_driver;
extern const saul_driver_t lis2dw12_saul_temp_driver;

int lis2dw12_init(lis2dw12_t *dev, const lis2dw12_params_t *params);
int lis2dw12_read(lis2dw12_t *dev, float *x, float *y, float *z, float *t);

#endif /* LIS2DW12_H */
