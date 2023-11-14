#include <stdio.h>
#include <string.h>

#include "acme_lora.h"
#include "fmt.h"
#include "hdc3020.h"
#include "hdc3020_params.h"
#include "periph/adc.h"
#include "periph/cpuid.h"
#include "periph/i2c.h"
#include "periph_conf.h"
#include "saml21_backup_mode.h"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE \
    { .pin = EXTWAKE_PIN6, .polarity = EXTWAKE_HIGH, .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 30 /* in seconds; -1 to disable */

#define ADC_VCC 0
#define ADC_VPANEL 1
#define VPANEL_ENABLE GPIO_PIN(PA, 27)

static saml21_extwake_t extwake = EXTWAKE;
static hdc3020_t hdc3020;

static uint8_t cpuid[CPUID_LEN];

static struct {
    int32_t vcc;
    int32_t vpanel;
    double temperature;
    double humidity;
} measures;

void read_vcc_vpanel(void)
{
    gpio_init(VPANEL_ENABLE, GPIO_OUT);
    gpio_set(VPANEL_ENABLE);

    // read vcc
    int32_t vcc = adc_sample(ADC_VCC, ADC_RES_12BIT) * 4000 / 4095; // corrected value (1V = 4095 counts)

    measures.vcc = (uint16_t)(vcc);
    puts("[SENSOR vcc] READ.");

    // read vpanel
    ztimer_sleep(ZTIMER_MSEC, 10);
    int32_t vpanel = adc_sample(ADC_VPANEL, ADC_RES_12BIT) * 3933 / 4095; // adapted to real resistor partition value (75k over 220k)

    measures.vpanel = (uint16_t)(vpanel);
    puts("[SENSOR vpanel] READ.");

    gpio_clear(VPANEL_ENABLE);
}

void read_hdc3020(void)
{
    if (hdc3020_init(&hdc3020, hdc3020_params) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL INIT.");
        return;
    }
    if (hdc3020_read(&hdc3020, &measures.temperature, &measures.humidity) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL READ.");
        return;
    }
    hdc3020_deinit(&hdc3020);

}

void sensors_read(void)
{
    memset(&measures, 0, sizeof(measures));
    cpuid_get((void *)(cpuid));

    read_hdc3020();
    read_vcc_vpanel();


    char cpuid_hex[CPUID_LEN * 2 + 1];

    fmt_bytes_hex(cpuid_hex, cpuid, CPUID_LEN);
    cpuid_hex[CPUID_LEN * 2] = 0;

    printf(
        "CPUID: %s, VCC: %04ld, VPANEL: %04ld, TEMP: % 3.2f, HUM: % 3.2f\n",
        cpuid_hex,
        measures.vcc, measures.vpanel,
        measures.temperature, measures.humidity);
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
