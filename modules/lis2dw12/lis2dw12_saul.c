#include "saul.h"
#include "lis2dw12.h"

static int read_accel(const void *dev, phydat_t *res)
{
    float values[4];
    int32_t int_values[3];
    if (lis2dw12_read((lis2dw12_t *)dev, &values[0], &values[1], &values[2], &values[3])) {
        return -ECANCELED;
    }
    for(size_t i=0; i<3; i++) { int_values[i] = (int32_t)values[i]; }
    res->unit = UNIT_G;
    res->scale = -3;
    phydat_fit(res, int_values, 3);
    return 3;
}

static int read_temperature(const void *dev, phydat_t *res)
{
    float values[4];
    int32_t int_value;
    if (lis2dw12_read((lis2dw12_t *)dev, &values[0], &values[1], &values[2], &values[3])) {
        return -ECANCELED;
    }
    int_value = (int32_t)values[3];
    res->unit = UNIT_TEMP_C;
    res->scale = 0;
    phydat_fit(res, &int_value, 1);
    return 1;
}

const saul_driver_t lis2dw12_saul_driver = {
    .read = read_accel,
    .write = saul_write_notsup,
    .type = SAUL_SENSE_ACCEL,
};

const saul_driver_t lis2dw12_saul_temp_driver = {
    .read = read_temperature,
    .write = saul_write_notsup,
    .type = SAUL_SENSE_TEMP,
};
