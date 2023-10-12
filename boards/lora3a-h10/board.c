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

#if ACME1_POWER
    printf("Enable ACME Sensor 1 as %s\n", BUS_MODE_TO_STR(ACME1_BUS_MODE));
#else
    gpio_init(GPIO_PIN(PB, 2), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PB, 3), GPIO_IN_PU);
#endif

#if ACME2_POWER
    printf("Enable ACME Sensor 2 as %s\n", BUS_MODE_TO_STR(ACME2_BUS_MODE));
#else
    gpio_init(GPIO_PIN(PA, 4), GPIO_IN_PU);
    gpio_init(GPIO_PIN(PA, 5), GPIO_IN_PU);
#endif
}
