#ifndef GPIO_PARAMS_H
#define GPIO_PARAMS_H

#include "board.h"
#include "saul/periph.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SAUL_GPIO_INIT_VAL(x)    (x ? SAUL_GPIO_INIT_SET : SAUL_GPIO_INIT_CLEAR)

/**
 * @brief    GPIO pin configuration
 */
static const saul_gpio_params_t saul_gpio_params[] =
{
    {
        .name = "Button(SW0)",
        .pin = BTN0_PIN,
        .mode = BTN0_MODE,
        .flags = SAUL_GPIO_INVERTED,
    },
    {
        .name = "Enable ACME Sensor 0",
        .pin = ACME0_POWER_PIN,
        .mode = GPIO_OUT,
        .flags = SAUL_GPIO_INIT_VAL(ACME0_POWER_PIN_INITIAL_VALUE),
    },
    {
        .name = "Enable ACME Sensor 1",
        .pin = ACME1_POWER_PIN,
        .mode = GPIO_OUT,
        .flags = SAUL_GPIO_INIT_VAL(ACME1_POWER_PIN_INITIAL_VALUE),
    },
    {
        .name = "Enable ACME Sensor 2",
        .pin = ACME2_POWER_PIN,
        .mode = GPIO_OUT,
        .flags = SAUL_GPIO_INIT_VAL(ACME2_POWER_PIN_INITIAL_VALUE),
    },
};

#ifdef __cplusplus
}
#endif

#endif /* GPIO_PARAMS_H */
/** @} */
