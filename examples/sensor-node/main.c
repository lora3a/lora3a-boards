#include <stdio.h>
#include <string.h>
#include "saml21_backup_mode.h"

#include "periph_conf.h"
#include "periph/i2c.h"
#include "hdc3020.h"
#include "hdc3020_params.h"
#include "acme_lora.h"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
        .pin = EXTWAKE_PIN6, \
        .polarity = EXTWAKE_HIGH, \
        .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

static saml21_extwake_t extwake = EXTWAKE;
static hdc3020_t hdc3020;

void sensors_init(void)
{
    puts("Sensors init.");
    if (hdc3020_init(&hdc3020, hdc3020_params) == HDC3020_OK) {
        puts("HDC3020 init.");
    }
}

void sensors_read(void)
{
    char payload[64];
    double temp, hum;
    lora_state_t lora;

    memset(&payload, 0, sizeof(payload));

    puts("Sensors read.");
    if (hdc3020_init(&hdc3020, hdc3020_params) == HDC3020_OK) {
        if (hdc3020_read(&hdc3020, &temp, &hum) == HDC3020_OK) {
            snprintf(payload, sizeof(payload), "Temp: %.1f °C, RH: %.1f %%", temp, hum);
            snprintf(payload, sizeof(payload), "{\"temp\": %.1f, \"rh\": %.1f}", temp, hum);
            puts(payload);
        }
        hdc3020_deinit(&hdc3020);
    }
    else {
        return;
    }

    memset(&lora, 0, sizeof(lora));
    lora.bandwidth = DEFAULT_LORA_BANDWIDTH;
    lora.spreading_factor = DEFAULT_LORA_SPREADING_FACTOR;
    lora.coderate = DEFAULT_LORA_CODERATE;
    lora.channel = DEFAULT_LORA_CHANNEL;
    lora.power = DEFAULT_LORA_POWER;


    if (lora_init(&(lora)) != 0) {
        return;
    }

    if (payload[0] == 0) {
        return;
    }

    iolist_t packet = {
        .iol_base = payload,
        .iol_len = (strlen(payload))
    };

    lora_write(&packet);
    puts("Lora send command.");
    lora_off();
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    sensors_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    sensors_read();
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
        if (extwake.pin != EXTWAKE_NONE) {
            printf("GPIO PA%d will wake the board.\n", extwake.pin);
        }
        if (SLEEP_TIME > -1) {
            printf("Periodic task running every %d seconds.\n", SLEEP_TIME);
        }
        sensors_init();
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
