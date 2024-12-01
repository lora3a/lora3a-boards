#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

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
