#include <stdio.h>
#include <string.h>

#include "periph/i2c.h"
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/rtc.h"
#include "rtc_utils.h"

#include "saml21_backup_mode.h"
#include "fram.h"
#include "senseair.h"
#include "senseair_params.h"

#define FW_VERSION "01.00.00"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE \
    { .pin = EXTWAKE_PIN6, .polarity = EXTWAKE_HIGH, .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */


static saml21_extwake_t extwake = EXTWAKE;

static senseair_t dev;
static senseair_abc_data_t abc_data;

#define SENSEAIR_STATE_FRAM_ADDR    0

void seaseair_sensor_read(void)
{
    uint16_t conc_ppm;
    int16_t temp_cC;

    if (gpio_init(ACME_BUS_POWER_PIN, GPIO_OUT)) {
        puts("ACME Bus enable failed.");
        return;
    }
    gpio_set(ACME_BUS_POWER_PIN);

    if (senseair_init(&dev, &senseair_params[0]) != SENSEAIR_OK) {
        puts("Senseair init failed.");
        gpio_clear(ACME_BUS_POWER_PIN);
        return;
    }

    memset(&abc_data, 0, sizeof(abc_data));
    if (fram_read(SENSEAIR_STATE_FRAM_ADDR, &abc_data, sizeof(abc_data))) {
        puts("FRAM read failed.");
    }
    else {
        if (senseair_write_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
            puts("ABC data restored to sensor.");
        }
        else {
            puts("ABC data not available.");
        }
    }

    if (senseair_read(&dev, &conc_ppm, &temp_cC) == SENSEAIR_OK) {
        printf("Concentration: %d ppm\n", conc_ppm);
        printf("Temperature: %4.2f °C\n", (temp_cC / 100.));
    }

    if (senseair_read_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
        puts("Saving sensor calibration data to FRAM.");
        if (fram_write(SENSEAIR_STATE_FRAM_ADDR, (uint8_t *)&abc_data, sizeof(abc_data))) {
            puts("FRAM write failed.");
        }
    }

    gpio_clear(ACME_BUS_POWER_PIN);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    seaseair_sensor_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    seaseair_sensor_read();
}

void boot_task(void)
{
    struct tm time;

    puts("Boot task.");
    rtc_localtime(0, &time);
    rtc_set_time(&time);
    fram_erase();
}

int main(void)
{
    fram_init();

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

        boot_task();

        break;
    }

    seaseair_sensor_read();

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
