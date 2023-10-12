#ifndef LIS2DW12_PARAMS_H
#define LIS2DW12_PARAMS_H

#include "board.h"
#include "lis2dw12.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LIS2DW12_I2C_DEV
#define LIS2DW12_I2C_DEV I2C_DEV(0)
#endif

#ifndef LIS2DW12_I2C_ADDR
#define LIS2DW12_I2C_ADDR   0x19
#endif

#ifndef LIS2DW12_POWER_PIN
#define LIS2DW12_POWER_PIN GPIO_UNDEF
#endif

#ifndef LIS2DW12_PARAMS
#define LIS2DW12_PARAMS { .i2c_dev = LIS2DW12_I2C_DEV,     \
                          .i2c_addr = LIS2DW12_I2C_ADDR,    \
                          .power_pin = LIS2DW12_POWER_PIN }
#endif

#ifndef LIS2DW12_SAULINFO
#define LIS2DW12_SAULINFO   { .name = "accel" }, { .name = "temperature" }
#endif

static const lis2dw12_params_t lis2dw12_params[] = {
    LIS2DW12_PARAMS
};

static const saul_reg_info_t lis2dw12_saul_info[] =
{
    LIS2DW12_SAULINFO
};

#ifdef __cplusplus
}
#endif


#endif /* LIS2DW12_PARAMS_H */
