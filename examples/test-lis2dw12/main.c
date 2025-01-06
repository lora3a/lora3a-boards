#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "saml21_backup_mode.h"

#include "periph_conf.h"
#include "periph/i2c.h"
#include "periph/adc.h"
#include "fmt.h"

#include "lis2dw12.h"
#include "lis2dw12_params.h"

#define FW_VERSION "01.00.00"

#define M_PI   3.14159265358979323846264338327950288
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)


/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
        .pin = EXTWAKE_PIN6, \
        .polarity = EXTWAKE_HIGH, \
        .flags = EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

static saml21_extwake_t extwake = EXTWAKE;
static lis2dw12_t lis2dw12;


void lis2dw12_sensor_read(void)
{
    float acc_x, acc_y, acc_z, acc_t, module, angleX, angleY, angleZ;

    if (lis2dw12_init(&lis2dw12, &lis2dw12_params[0]) != LIS2DW12_OK) {
        puts("[SENSOR lis2dw12] INIT FAILED.");
        return;
    }

    if (lis2dw12_read(&lis2dw12, &acc_x, &acc_y, &acc_z, &acc_t) != LIS2DW12_OK) {
        puts("[SENSOR lis2dw12] READ FAILED.");
        return;
    }
	module = sqrtf((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
	angleX = radToDeg(acosf(acc_x/abs(module)));
	angleY = radToDeg(acosf(acc_y/abs(module)));
	angleZ = radToDeg(acosf(acc_z/abs(module)));
	
    printf(
        "[DATA] X: %.3f, Y: %.3f, Z: %.3f, TEMP: %.2f\n",
        acc_x, acc_y, acc_z,
        acc_t);
    printf(
        "MOD: %.3f, angleX: %.3f, angleY: %.3f, angleZ: %.3f\n",
        module, angleX, angleY, angleZ
		);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    lis2dw12_sensor_read();
}

void periodic_task(void)
{
    puts("Periodic task.");
    lis2dw12_sensor_read();
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
        printf("-    Test LIS2DW12  For Berta-H10   -\n");
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
        lis2dw12_sensor_read();
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
