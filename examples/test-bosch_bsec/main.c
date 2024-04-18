#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bme68x.h"
#include "bme68x_params.h"

#include "bsec_interface_multi.h"

static bme68x_t dev[BME68X_NUMOF];
static void *inst[BME68X_NUMOF];

bsec_library_return_t setup_for_iaq(void *instance) {
    bsec_sensor_configuration_t virtual_sensors[13];
    bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

	virtual_sensors[0].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_GAS;
    virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_ULP;
	virtual_sensors[4].sensor_id = BSEC_OUTPUT_IAQ;
    virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[5].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[6].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    virtual_sensors[6].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    virtual_sensors[7].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[8].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    virtual_sensors[8].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[9].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    virtual_sensors[9].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[10].sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS;
    virtual_sensors[10].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[11].sensor_id = BSEC_OUTPUT_RUN_IN_STATUS;
    virtual_sensors[11].sample_rate = BSEC_SAMPLE_RATE_ULP;
    virtual_sensors[12].sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE;
    virtual_sensors[12].sample_rate = BSEC_SAMPLE_RATE_ULP;

    return bsec_update_subscription_m(instance, virtual_sensors, ARRAY_SIZE(virtual_sensors), sensor_settings, &n_sensor_settings);
}

int main(void)
{
    size_t i;
    bsec_version_t version;
    bsec_library_return_t res;

    puts("Bosch BSEC library test");

    for (i = 0; i < BME68X_NUMOF; i++) {
        if (bme68x_init(&dev[i], &bme68x_params[i]) != BME68X_OK) {
            printf("[ERROR] Initialization failed for dev[%d].\n", i);
            return 1;
        }
        if ((inst[i] = malloc(bsec_get_instance_size_m())) == NULL) {
            printf("[ERROR] Memory allocation failed for inst[%d].\n", i);
            return 1;
        }
        if ((res = bsec_init_m(inst[i])) != BSEC_OK) {
            printf("[ERROR] Initialization failed for inst[%d]: %d.\n", i, res);
            return 1;
        }

        if ((res = bsec_get_version_m(inst[i], &version)) != BSEC_OK) {
            printf("[ERROR] Read BSEC version failed for inst[%d]: %d.\n", i, res);
            return 1;
        }
        printf("BSEC version for inst[%d]: %d.%d.%d.%d\n", i, version.major, version.minor, version.major_bugfix, version.minor_bugfix);
        if ((res = setup_for_iaq(inst[i])) != BSEC_OK) {
            printf("[ERROR] Setup for IAQ failed for inst[%d]: %d.\n", i, res);
            return 1;
        }
    }

    puts("Init done.");

    int64_t time_stamp = 0;
    bsec_bme_settings_t sensor_settings[BME68X_NUMOF];
    for (i = 0; i < BME68X_NUMOF; i++) {
        memset(&sensor_settings[i], 0, sizeof(sensor_settings[i]));
    }

    while (1) {
        for (i = 0; i < BME68X_NUMOF; i++) {
            time_stamp = ztimer_now(ZTIMER_USEC);
            if (time_stamp >= sensor_settings[i].next_call)
            {
                res = bsec_sensor_control_m(inst[i], time_stamp, &sensor_settings[i]);
                if (res != BSEC_OK) {
                    if (res < BSEC_OK) {
                        printf("[ERROR] bsec_sensor_control_m for inst[%d]: %d\n", i, res);
                        return 1;
                    } else {
                        printf("[WARNING] bsec_sensor_control for inst[%d]: %d\n", i, res);
                    }
                }
                // TODO:
                // - configure sensor as dictated by sensor_settings[i]
                // - read sensor data
                // - error check data
                // - process data with bsec_do_steps_m(...)
                // - print results
            }
        }
    }

    return 0;
}
