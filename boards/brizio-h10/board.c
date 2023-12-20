#include <stdio.h>

#include "board.h"
#include "periph/gpio.h"
#include "acme_bus_mode.h"

void board_init(void)
{
    /* add pullups to UART0 pins */
    PORT->Group[PA].PINCFG[22].bit.PULLEN = 1;
    PORT->Group[PA].PINCFG[23].bit.PULLEN = 1;

    /* initialize board specific pins for LoRa */
#if defined(MODULE_SX127X)
    gpio_init(TCXO_PWR_PIN, GPIO_OUT);
    gpio_set(TCXO_PWR_PIN);
    gpio_init(TX_OUTPUT_SEL_PIN, GPIO_OUT);
    gpio_write(TX_OUTPUT_SEL_PIN, !SX127X_PARAM_PASELECT);
#endif /* USEMODULE_SX127X */

#if defined(ACME0_BUS_MODE)
    printf("Enable ACME Sensor 0 as %s\n", BUS_MODE_TO_STR(ACME0_BUS_MODE));

    #if !IS_USED(MODULE_SAUL_GPIO)
    gpio_init(ACME0_POWER_PIN, GPIO_OUT);
    gpio_write(ACME0_POWER_PIN, ACME0_POWER_PIN_INITIAL_VALUE);
    #endif

#else
    gpio_init(GPIO_PIN(PA, 16), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 17), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 18), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 19), GPIO_IN_PU);
#endif

#if defined(ACME1_BUS_MODE)
    printf("Enable ACME Sensor 1 as %s\n", BUS_MODE_TO_STR(ACME1_BUS_MODE));

    #if !IS_USED(MODULE_SAUL_GPIO)
    gpio_init(ACME1_POWER_PIN, GPIO_OUT);
    gpio_write(ACME1_POWER_PIN, ACME1_POWER_PIN_INITIAL_VALUE);
    #endif
#else
    gpio_init(GPIO_PIN(PB,  2), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PB,  3), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PB, 22), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PB, 23), GPIO_IN_PU);
#endif

#if defined(ACME2_BUS_MODE)
    printf("Enable ACME Sensor 2 as %s\n", BUS_MODE_TO_STR(ACME2_BUS_MODE));

    #if !IS_USED(MODULE_SAUL_GPIO)
    gpio_init(ACME2_POWER_PIN, GPIO_OUT);
    gpio_write(ACME2_POWER_PIN, ACME2_POWER_PIN_INITIAL_VALUE);
    #endif
#else
    gpio_init(GPIO_PIN(PA, 4), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 5), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 6), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 7), GPIO_IN_PU);
#endif
}
