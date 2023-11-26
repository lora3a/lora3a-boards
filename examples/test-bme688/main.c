#include <stdio.h>
#include <string.h>

#include "periph/i2c.h"
#include "periph_conf.h"
#include "saml21_backup_mode.h"

#include "bme680.h"
#include "bme680_params.h"

#define FW_VERSION "01.00.00"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE \
    { .pin = EXTWAKE_PIN6, \
      .polarity = EXTWAKE_HIGH, \
      .flags = EXTWAKE_IN_PU \
    }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */


static saml21_extwake_t extwake = EXTWAKE;
static bme680_t dev;

static bool _temp_valid = false;
static bool _press_valid = false;
static bool _hum_valid = false;
static bool _gas_valid = false;

static int16_t _temp;
static int16_t _press;
static int16_t _hum;
static uint32_t _gas;

void bme688_sensor_read(void)
{
    int duration;

    if (bme680_init(&dev, &bme680_params[0]) != BME680_OK) {
        puts("[ERROR] Initialization of BME688.");
        return;
    }

    if (bme680_force_measurement(&dev) != BME680_OK) {
        puts("[ERROR] Forse Measurement.");
        return;
    }

    if ((duration = bme680_get_duration(&dev)) < 0) {
        puts("[ERROR] Get Duration.");
        return;
    }
    ztimer_sleep(ZTIMER_MSEC, duration);

    bme680_field_data_t data;

    if (bme680_get_data(&dev, &data) != BME680_OK) {
        puts("[ERROR] Get Data.");
        return;
    }

#if MODULE_BME680_FP
    _temp = data.temperature * 100;
    _press = data.pressure / 100;
    _hum = data.humidity * 100;
#else
    _temp = data.temperature;
    _press = data.pressure / 100;
    _hum = data.humidity / 10;
#endif
    _gas = (data.status & BME680_GASM_VALID_MSK) ? data.gas_resistance : 0;

    /* mark sensor values as valid */
    _temp_valid = true;
    _press_valid = true;
    _hum_valid = true;
    _gas_valid = true;

    printf("[DATA] TEMP: %.2f °C, PRESS: %d Pa, HUM: %.1f %%, GAS: %ld Ohm.\n",
           _temp / 100., _press, _hum / 100., _gas);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    bme688_sensor_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    bme688_sensor_read();
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
        printf("-      Test BME688 For Berta-H10    -\n");
        printf("-           by Acme Systems         -\n");
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
        bme688_sensor_read();
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
