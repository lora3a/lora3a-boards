#include <stdio.h>
#include <string.h>
#include "saml21_backup_mode.h"

#include "periph_conf.h"
#include "periph/i2c.h"
#include "hdc3020.h"
#include "hdc3020_params.h"
#include "acme_lora.h"
#include "periph/adc.h"
#include "periph/cpuid.h"
#include "fmt.h"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
        .pin = EXTWAKE_PIN6, \
        .polarity = EXTWAKE_HIGH, \
        .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

#define ADC_VCC 0
#define ADC_VPANEL 1
#define VPANEL_ENABLE GPIO_PIN(PA, 27)

static saml21_extwake_t extwake = EXTWAKE;
static hdc3020_t hdc3020;

static struct {
    uint8_t cpuid[CPUID_LEN];
    int32_t vcc;
    int32_t vpanel;
    double temp;
    double hum;
} measures;




void sensors_read(void)
{
    char payload[MAX_PACKET_LEN];
    lora_state_t lora;

    memset(&payload, 0, sizeof(payload));
    memset(&measures, 0, sizeof(measures));

    cpuid_get((void *)(measures.cpuid));

    if (hdc3020_init(&hdc3020, hdc3020_params) != HDC3020_OK) {
        return;
    }
    if (hdc3020_read(&hdc3020, &measures.temp, &measures.hum) != HDC3020_OK) {
        return;
    }
    puts("[SENSOR hdc3020] READ.");
    hdc3020_deinit(&hdc3020);

    gpio_init(VPANEL_ENABLE, GPIO_OUT);
    gpio_set(VPANEL_ENABLE);

    // read vcc
    measures.vcc = adc_sample(ADC_VCC, ADC_RES_12BIT) * 4000 / 4095;  // corrected value (1V = 4095 counts)
    puts("[SENSOR vcc] READ.");

    // read vpanel
    ztimer_sleep(ZTIMER_MSEC, 10);
    measures.vpanel = adc_sample(ADC_VPANEL, ADC_RES_12BIT) * 3933 / 4095; // adapted to real resistor partition value (75k over 220k)
    puts("[SENSOR vpanel] READ.");

    gpio_clear(VPANEL_ENABLE);

    char cpuid[CPUID_LEN * 2 + 1];
    fmt_bytes_hex(cpuid, measures.cpuid, CPUID_LEN);
    cpuid[CPUID_LEN * 2] = 0;

    snprintf(payload, sizeof(payload),
             "{\"cpuid\": \"%s\", \"temp\": %.1f, \"hum\": %.1f, \"vcc\": %ld, \"vpanel\": %ld}",
             cpuid,
             measures.temp, measures.hum,
             measures.vcc, measures.vpanel
             );

    puts(payload);

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
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
