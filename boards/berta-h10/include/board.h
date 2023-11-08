#ifndef BOARD_H
#define BOARD_H

#include "sx127x.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SX127X_PARAM_PASELECT
// use SX127X_PA_RFO for no boost
#define SX127X_PARAM_PASELECT   SX127X_PA_BOOST
#endif

#define SX127X_PARAMS  { .spi = (SPI_DEV(0)),               \
                         .nss_pin = GPIO_PIN(PB, 31),   /* D10 */ \
                         .reset_pin = GPIO_PIN(PB, 15), /* A0 */  \
                         .dio0_pin = GPIO_PIN(PB, 16),  /* D2 */  \
                         .dio1_pin = GPIO_PIN(PA, 11),  /* D3 */  \
                         .dio2_pin = GPIO_PIN(PA, 12),  /* D4 */  \
                         .dio3_pin = GPIO_PIN(PB, 17),  /* D5 */  \
                         .paselect = (SX127X_PARAM_PASELECT) }

#define TCXO_PWR_PIN        GPIO_PIN(PA, 9)     /**< 32 MHz TCXO 1ppm oscillator for radio enable */
#define TX_OUTPUT_SEL_PIN   GPIO_PIN(PA, 13)    /**< BAND_SEL */

#define BTN0_PIN       GPIO_PIN(PA, 6)
#ifndef BTN0_MODE
#define BTN0_MODE      GPIO_IN_PU
#endif

#define ACME0_POWER_PIN    GPIO_PIN(PA, 27)
#define ACME1_POWER_PIN    GPIO_PIN(PA, 28)
#define ACME2_POWER_PIN    GPIO_PIN(PA, 31)

#ifndef ACME0_POWER_PIN_INITIAL_VALUE
    #ifdef ACME0_BUS_MODE
        #define ACME0_POWER_PIN_INITIAL_VALUE 1
    #else
        #define ACME0_POWER_PIN_INITIAL_VALUE 0
    #endif
#endif

#ifndef ACME1_POWER_PIN_INITIAL_VALUE
    #ifdef ACME1_BUS_MODE
        #define ACME1_POWER_PIN_INITIAL_VALUE 1
    #else
        #define ACME1_POWER_PIN_INITIAL_VALUE 0
    #endif
#endif

#ifndef ACME2_POWER_PIN_INITIAL_VALUE
    #ifdef ACME2_BUS_MODE
        #define ACME2_POWER_PIN_INITIAL_VALUE 1
    #else
        #define ACME2_POWER_PIN_INITIAL_VALUE 0
    #endif
#endif

void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
