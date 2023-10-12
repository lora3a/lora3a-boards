#include <stdio.h>
#include <string.h>

#include "lis2dw12.h"
#include "lis2dw12_params.h"
#include "periph/i2c.h"
#include "ztimer.h"

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    lis2dw12_t *dev = (lis2dw12_t *)handle;

    return i2c_write_regs(dev->params.i2c_dev, dev->params.i2c_addr, reg, (uint8_t *)bufp, len, 0);
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    lis2dw12_t *dev = (lis2dw12_t *)handle;

    return i2c_read_regs(dev->params.i2c_dev, dev->params.i2c_addr, reg, bufp, len, 0);
}

int lis2dw12_init(lis2dw12_t *dev, const lis2dw12_params_t *params)
{
    uint8_t whoamI, rst;

    assert(dev && params);
    dev->params = *params;

    if (gpio_is_valid(dev->params.power_pin)) {
        gpio_init(dev->params.power_pin, GPIO_OUT);
        gpio_set(dev->params.power_pin);
    }

    /* Initialize mems driver interface */
    dev->ctx.write_reg = platform_write;
    dev->ctx.read_reg = platform_read;
    dev->ctx.handle = (void *)dev;
    /* Wait sensor boot time */
    ztimer_sleep(ZTIMER_MSEC, 20);
    /* Check device ID */
    lis2dw12_device_id_get(&dev->ctx, &whoamI);

    if (whoamI != LIS2DW12_ID) {
        return -LIS2DW12_ERR_NO_DEV;
    }

    /* Restore default configuration */
    lis2dw12_reset_set(&dev->ctx, PROPERTY_ENABLE);

    do {
        lis2dw12_reset_get(&dev->ctx, &rst);
    } while (rst);

    return LIS2DW12_OK;
}

int lis2dw12_read(lis2dw12_t *dev, float *x_mg, float *y_mg, float *z_mg, float *t_c)
{
    int16_t data_raw_acceleration[3], data_raw_temperature;

    /* Enable Block Data Update */
    lis2dw12_block_data_update_set(&dev->ctx, PROPERTY_ENABLE);
    /* Set full scale */
    //lis2dw12_full_scale_set(&dev->ctx, LIS2DW12_8g);
    lis2dw12_full_scale_set(&dev->ctx, LIS2DW12_2g);
    /* Configure filtering chain
     * Accelerometer - filter path / bandwidth
     */
    lis2dw12_filter_path_set(&dev->ctx, LIS2DW12_LPF_ON_OUT);
    lis2dw12_filter_bandwidth_set(&dev->ctx, LIS2DW12_ODR_DIV_4);
    /* Configure power mode */
    lis2dw12_power_mode_set(&dev->ctx, LIS2DW12_HIGH_PERFORMANCE);
    //lis2dw12_power_mode_set(&dev->ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);
    /* Set Output Data Rate */
    lis2dw12_data_rate_set(&dev->ctx, LIS2DW12_XL_ODR_25Hz);

    uint8_t reg = 0;
    while (!reg) {
        /* Read output only if new value is available */
        lis2dw12_flag_data_ready_get(&dev->ctx, &reg);
    }

    /* Read acceleration data */
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lis2dw12_acceleration_raw_get(&dev->ctx, data_raw_acceleration);
    *x_mg = lis2dw12_from_fs2_to_mg(data_raw_acceleration[0]);
    *y_mg = lis2dw12_from_fs2_to_mg(data_raw_acceleration[1]);
    *z_mg = lis2dw12_from_fs2_to_mg(data_raw_acceleration[2]);
    /* Read temperature data */
    data_raw_temperature = 0;
    lis2dw12_temperature_raw_get(&dev->ctx, &data_raw_temperature);
    *t_c = lis2dw12_from_lsb_to_celsius(data_raw_temperature);

    return 0;
}
