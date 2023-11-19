#include <stdio.h>
#include <string.h>

#include "periph/i2c.h"
#include "periph_conf.h"
#include "saml21_backup_mode.h"

#include "hdc3020.h"
#include "hdc3020_params.h"

#define FW_VERSION "01.00.00"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE \
    { .pin = EXTWAKE_PIN6, .polarity = EXTWAKE_HIGH, .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */


static saml21_extwake_t extwake = EXTWAKE;
static hdc3020_t hdc3020;


void hdc3020_sensor_read(void)
{
    double temperature, humidity = -999;

    if (hdc3020_init(&hdc3020, hdc3020_params) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL INIT.");
        return;
    }

    if (hdc3020_deactivate_alert(&hdc3020) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL DEACTIVATE ALERT.");
        return;
    }

    if (hdc3020_read(&hdc3020, &temperature, &humidity) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL READ.");
        return;
    }
    hdc3020_deinit(&hdc3020);

    printf(
        "[DATA] TEMP: % 3.2f, HUM: % 3.2f\n",
        temperature, humidity);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    hdc3020_sensor_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    hdc3020_sensor_read();
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
        printf("-     Test HDC3020 For Berta-H10    -\n");
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
        hdc3020_sensor_read();
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
