#ifndef SONICLIB_PARAMS_H
#define SONICLIB_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph/i2c.h"
#include "periph/gpio.h"

typedef struct {
    uint8_t i2c_bus;
    uint8_t i2c_addr;
    gpio_t prog_pin;
    gpio_t io_pin;
} soniclib_params_t;

#ifndef SONICLIB_I2C_BUS
#define SONICLIB_I2C_BUS I2C_DEV(0)
#endif

#ifndef SONICLIB_I2C_ADDR
#define SONICLIB_I2C_ADDR (41)
#endif

#ifndef SONICLIB_PROG_PIN
#define SONICLIB_PROG_PIN GPIO_PIN(0,0)
#endif

#ifndef SONICLIB_IO_PIN
#define SONICLIB_IO_PIN GPIO_PIN(0,0)
#endif

#ifndef SONICLIB_RTC_CAL_PULSE_MS
#define SONICLIB_RTC_CAL_PULSE_MS (100)
#endif

#ifndef SONICLIB_PARAMS
#define SONICLIB_PARAMS           \
{                                 \
   .i2c_bus  = SONICLIB_I2C_BUS,  \
   .i2c_addr = SONICLIB_I2C_ADDR, \
   .prog_pin = SONICLIB_PROG_PIN, \
   .io_pin   = SONICLIB_IO_PIN,   \
}
#endif

static const soniclib_params_t soniclib_params[] = {
    SONICLIB_PARAMS,
};
#define SONICLIB_NUMOF  ARRAY_SIZE(soniclib_params)

#ifdef __cplusplus
}
#endif

#endif /* SONICLIB_PARAMS_H */
