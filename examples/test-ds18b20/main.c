#include <stdio.h>
#include <string.h>
#include "saml21_backup_mode.h"

// #include "periph_conf.h"
#include "periph/adc.h"
#include "periph/cpuid.h"
#include "fmt.h"

#include "ds18.h"
#include "ds18_params.h"

#define FW_VERSION "01.00.00"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
        .pin = EXTWAKE_PIN6, \
        .polarity = EXTWAKE_HIGH, \
        .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

static saml21_extwake_t extwake = EXTWAKE;
static ds18_t ds18;


void ds18b20_sensor_read(void)
{
    int16_t temperature;

    gpio_init(DS18_PWR_PIN, GPIO_OUT);
    gpio_set(DS18_PWR_PIN);

    if (ds18_init(&ds18, &ds18_params[0]) != DS18_OK) {
        puts("[SENSOR DS18B20] FAIL INIT.");
        return;
    }

    if (ds18_get_temperature(&ds18, &temperature) != DS18_OK) {
        puts("[SENSOR DS18B20] FAIL READ.");
        return;
    }

    gpio_clear(DS18_PWR_PIN);

    printf("[DATA] TEMP: %d\n",temperature);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    ds18b20_sensor_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    ds18b20_sensor_read();
}


int main(void)
{
    switch (saml21_wakeup_cause()) {
    case BACKUP_EXTWAKE:
        wakeup_task();
        break;
    case BACKUP_RTC:
        periodic_task();
        break;
    default:
        printf("\n");
        printf("-------------------------------------\n");
        printf("-      Test DS18  For Berta-H10     -\n");
        printf("-          by Acme Systems          -\n");
        printf("-  Version  : %s              -\n", FW_VERSION);
        printf("-  Compiled : %s %s  -\n", __DATE__, __TIME__);
        printf("-------------------------------------\n");
        printf("\n");

        if (extwake.pin != EXTWAKE_NONE) {
            printf("PUSH BUTTON P2 will wake the board.\n");
        }
        if (SLEEP_TIME > -1) {
            printf("Periodic task running every %d seconds.\n", SLEEP_TIME);
        }
        ds18b20_sensor_read();
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
