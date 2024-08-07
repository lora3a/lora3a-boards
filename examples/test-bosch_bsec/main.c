#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "od.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "rtc_utils.h"
#include "ztimer64.h"
#include "fram.h"
#include "bme68x.h"
#include "bme68x_params.h"
#include "saml21_backup_mode.h"

#include "bosch_bsec.h"

#ifndef SLEEP_SECS
#define SLEEP_SECS 3
#endif

#define BSEC_FRAM_STATE_OFFSET 0

void print_llu(char *label, int64_t value)
{
    printf("%s 0x", label);
    uint8_t *ptr = (uint8_t *)&value;
    for(int k=7;k>=0;k--) printf("%02X", *(ptr+k));
    printf("\n");
}

typedef struct
{
	int64_t timestamp;
	float gas_estimate_1;
	float gas_estimate_2;
	float gas_estimate_3;
	float gas_estimate_4;
	float raw_pressure;
	float raw_temp;
	float raw_humidity;
	float raw_gas;
	uint8_t raw_gas_index;
	uint8_t gas_accuracy_1;
	uint8_t gas_accuracy_2;
	uint8_t gas_accuracy_3;
	uint8_t gas_accuracy_4;
	uint8_t sens_no;
	float iaq;
	uint8_t iaq_accuracy;
	float temperature;
	float humidity;
	float static_iaq;
	bool stabStatus;
	bool runInStatus;
	float co2_equivalent;
	float breath_voc_equivalent;
	float gas_percentage;
} output_t;

static bme68x_t dev[BME68X_NUMOF];
static bsec_bme_settings_t sensor_settings[BME68X_NUMOF];
static bme68x_data_t sensor_data[3];

bsec_library_return_t setup_for_iaq(void *instance) {
    bsec_sensor_configuration_t virtual_sensors[13];
    bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    float sample_rate = BSEC_SAMPLE_RATE;

	virtual_sensors[0].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    virtual_sensors[0].sample_rate = sample_rate;
    virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    virtual_sensors[1].sample_rate = sample_rate;
    virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    virtual_sensors[2].sample_rate = sample_rate;
    virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_GAS;
    virtual_sensors[3].sample_rate = sample_rate;
	virtual_sensors[4].sensor_id = BSEC_OUTPUT_IAQ;
    virtual_sensors[4].sample_rate = sample_rate;
    virtual_sensors[5].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    virtual_sensors[5].sample_rate = sample_rate;
    virtual_sensors[6].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    virtual_sensors[6].sample_rate = sample_rate;
    virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    virtual_sensors[7].sample_rate = sample_rate;
    virtual_sensors[8].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    virtual_sensors[8].sample_rate = sample_rate;
    virtual_sensors[9].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    virtual_sensors[9].sample_rate = sample_rate;
    virtual_sensors[10].sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS;
    virtual_sensors[10].sample_rate = sample_rate;
    virtual_sensors[11].sensor_id = BSEC_OUTPUT_RUN_IN_STATUS;
    virtual_sensors[11].sample_rate = sample_rate;
    virtual_sensors[12].sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE;
    virtual_sensors[12].sample_rate = sample_rate;

    return bsec_update_subscription_m(instance, virtual_sensors, ARRAY_SIZE(virtual_sensors), sensor_settings, &n_sensor_settings);
}

bsec_library_return_t process_data(int64_t tstamp_ns, bme68x_data_t data, int32_t process_data, uint8_t i)
{
	bsec_library_return_t bsec_status;
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
    uint8_t index = 0;
	output_t output = {0};
    bsec_status = bsec_process_data(tstamp_ns, data, process_data, &dev[i], i, bsec_outputs, &num_bsec_outputs);
    if (bsec_status == BSEC_OK) {
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                case BSEC_OUTPUT_GAS_ESTIMATE_1:
                    output.gas_estimate_1 = bsec_outputs[index].signal;
					output.gas_accuracy_1 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_GAS_ESTIMATE_2:
                    output.gas_estimate_2 = bsec_outputs[index].signal;
					output.gas_accuracy_2 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_GAS_ESTIMATE_3:
                    output.gas_estimate_3 = bsec_outputs[index].signal;
					output.gas_accuracy_3 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_GAS_ESTIMATE_4:
                    output.gas_estimate_4 = bsec_outputs[index].signal;
					output.gas_accuracy_4 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    output.raw_pressure = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_TEMPERATURE:
                    output.raw_temp = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    output.raw_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    output.raw_gas = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS_INDEX:
                    output.raw_gas_index = (uint8_t)bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_REGRESSION_ESTIMATE_1:
                    output.gas_estimate_1 = bsec_outputs[index].signal;
					output.gas_accuracy_1 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_REGRESSION_ESTIMATE_2:
                    output.gas_estimate_2 = bsec_outputs[index].signal;
					output.gas_accuracy_2 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_REGRESSION_ESTIMATE_3:
                    output.gas_estimate_3 = bsec_outputs[index].signal;
					output.gas_accuracy_3 = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_REGRESSION_ESTIMATE_4:
                    output.gas_estimate_4 = bsec_outputs[index].signal;
					output.gas_accuracy_4 = bsec_outputs[index].accuracy;
                    break;
				case BSEC_OUTPUT_IAQ:
                    output.iaq = bsec_outputs[index].signal;
                    output.iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_STATIC_IAQ:
                    output.static_iaq = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    output.co2_equivalent = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    output.breath_voc_equivalent = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    output.temperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    output.humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_STABILIZATION_STATUS:
                    output.stabStatus = bsec_outputs[index].signal ? 1 : 0;
                    break;
				case BSEC_OUTPUT_RUN_IN_STATUS:
                    output.runInStatus = bsec_outputs[index].signal ? 1 : 0;
                    break;
                case BSEC_OUTPUT_GAS_PERCENTAGE:
                    output.gas_percentage = bsec_outputs[index].signal;
                    break;
                default:
                    continue;
            }
            /* Assume that all the returned timestamps are the same */
            output.timestamp = bsec_outputs[index].time_stamp;
        }
		output.sens_no = i;
        puts("----------------------------------------");
        printf("SENSOR %d\n", output.sens_no);
        printf("IAQ: %.2f\n", output.iaq);
        printf("IAQ ACC: %d\n", output.iaq_accuracy);
        printf("STATIC IAQ: %.2f\n", output.static_iaq);
        printf("RAW TEMP: %.2f\n", output.raw_temp);
        printf("RAW HUM: %.2f\n", output.raw_humidity);
        printf("TEMP: %.2f\n", output.temperature);
        printf("HUM: %.2f\n", output.humidity);
        printf("PRESS: %.2f\n", output.raw_pressure);
        printf("RAW GAS: %.2f\n", output.raw_gas);
        printf("GAS %%: %.2f\n", output.gas_percentage);
        printf("CO2 EQUIV: %.2f\n", output.co2_equivalent);
        printf("BREATH VOC EQUIV: %.2f\n", output.breath_voc_equivalent);
        printf("STAB STATUS: %d\n", output.stabStatus);
        printf("RUN IN STATUS: %d\n", output.runInStatus);
        printf("BSEC STATUS: %d\n", bsec_status);
        puts("");
    }
    return bsec_status;
}

int main(void)
{
    size_t i, j;
    bsec_library_return_t res;
    int64_t time_start, time_stamp;
    uint8_t measures, n_data;
    struct tm time;

    rtc_get_time(&time);
    printf("RTC time: %04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    time_start = (int64_t)rtc_mktime(&time) * 1000000000;
    print_llu("time_start: ", time_start);

    fram_init();
    puts("Bosch BSEC library test.");

    for (i = 0; i < BME68X_NUMOF; i++) {
        if (bme68x_init(&dev[i], &bme68x_params[i]) != BME68X_OK) {
            printf("[ERROR] Initialization failed for dev[%d].\n", i);
            return 1;
        }
    }
    memset(sensor_settings, 0, sizeof(sensor_settings));
    fram_read(BSEC_FRAM_STATE_OFFSET, bsec_state, sizeof(bsec_state));
    res = bsec_init();
    if (res != BSEC_OK) {
        printf("[ERROR] BSEC init failed: %s.\n", bsec_errno(res));
        return 1;
    }

    for (i = 0; i < BME68X_NUMOF; i++) {
        if ((res = setup_for_iaq(bsec_inst[i])) != BSEC_OK) {
            printf("[ERROR] setup_for_iaq failed for inst[%d]: %s.\n", i, bsec_errno(res));
            return 1;
        }
    }

    puts("Init done.");

    measures = 0;
    while (measures < BME68X_NUMOF) {
        for (i = 0; i < BME68X_NUMOF; i++) {
            time_stamp = time_start + ztimer64_now(ZTIMER64_USEC) * 1000;
            if (time_stamp >= sensor_settings[i].next_call) {
                // ask BSEC for sensor settings
                res = bsec_sensor_control_m(bsec_inst[i], time_stamp, &sensor_settings[i]);
                if (res != BSEC_OK) {
                    if (res < BSEC_OK) {
                        printf("[ERROR] bsec_sensor_control_m for inst[%d]: %s.\n", i, bsec_errno(res));
                        return 1;
                    } else {
                        printf("[WARNING] bsec_sensor_control for inst[%d]: %s.\n", i, bsec_errno(res));
                    }
                }
                // configure sensor as dictated
                if ((sensor_settings[i].op_mode == BME68X_FORCED_MODE) || (dev[i].config.op_mode != sensor_settings[i].op_mode)) {
                    if (bsec_apply_configuration(&sensor_settings[i], &dev[i], i) != BME68X_OK) {
                        printf("[ERROR] apply configuration to sensor %d failed\n", i);
                        return 1;
                    }
                    printf("applied configuration to sensor %d\n", i);
			    }
            }
            // - read sensor data
			if (sensor_settings[i].trigger_measurement && sensor_settings[i].op_mode != BME68X_SLEEP_MODE) {
                bme68x_get_measure_data(&dev[i], sensor_data, &n_data);
                for (j = 0; j < n_data; j++) {
                    bme68x_data_t data = sensor_data[j];
                    if (data.status & BME68X_GASM_VALID_MSK) {
                        if ((res = process_data(time_stamp, data, sensor_settings[i].process_data, i)) == BSEC_OK) {
                            measures++;
                        } else {
                            printf("[ERROR] process_data failed for inst[%d]: %s.\n", i, bsec_errno(res));
                            return 1;
                        }
                    }
                    if (sensor_settings[i].op_mode == BME68X_FORCED_MODE) break;
                }
            }
        }
    }

    // save current library state to FRAM
    if ((res = bsec_deinit()) == BSEC_OK) {
        fram_write(BSEC_FRAM_STATE_OFFSET, (uint8_t *)bsec_state, sizeof(bsec_state));
    } else {
        printf("[ERROR] BSEC deinit failed: %s.\n", bsec_errno(res));
    }
    for (i = 0; i < BME68X_NUMOF; i++) {
        bme68x_dev_t *bme = &BME68X_SENSOR(&dev[i]);
        if (bme68x_set_op_mode(BME68X_SLEEP_MODE, bme) != BME68X_OK) {
            printf("[ERROR]: Failed to put sensor %d in sleep mode\n", i);
        }
    }

    fram_off();
#ifdef BME68X_POWER_OFF_DURING_SLEEP
    gpio_clear(BME68X_POWER_PIN);
#endif
    i2c_deinit_pins(ACME0_I2C_DEV);
    i2c_deinit_pins(ACME2_I2C_DEV);
    uart_deinit_pins(UART_DEV(0));

    saml21_extwake_t extwake = { .pin=EXTWAKE_NONE };
    saml21_backup_mode_enter(1, extwake, SLEEP_SECS, 0);

    // never reached
    return 0;
}
