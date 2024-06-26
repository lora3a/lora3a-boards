#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "od.h"
#include "ztimer64.h"
#include "bme68x.h"
#include "bme68x_params.h"

#include "bsec_interface_multi.h"
#include "bsec_selectivity.h"

#define BSEC_CHECK_INPUT(x, shift)  (x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR         UINT16_C(140)

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
	float stabStatus;
	float runInStatus;
	float co2_equivalent;
	float breath_voc_equivalent;
	float gas_percentage;
}output_t;

static bme68x_t dev[BME68X_NUMOF];
static void *inst[BME68X_NUMOF];
static bsec_bme_settings_t sensor_settings[BME68X_NUMOF];
static bme68x_data_t sensor_data[3];
static uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};
static int32_t bsec_config_len;
static float temp_offset = 0.0f;

uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    (void)config_buffer;
    (void)n_buffer;
    size_t n = sizeof(bsec_config_selectivity);
    if (n <= n_buffer) {
        memcpy(config_buffer, bsec_config_selectivity, n_buffer);
    } else {
        puts("ERROR: Config too large");
        n = 0;
    }
    return n;
}

bsec_library_return_t setup_for_iaq(void *instance) {
    bsec_sensor_configuration_t virtual_sensors[13];
    bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    float sample_rate = BSEC_SAMPLE_RATE_LP;

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

int apply_configuration(bsec_bme_settings_t *sensor_settings, uint8_t i)
{
    int ret;

    dev[i].config.op_mode = sensor_settings->op_mode;
    if (sensor_settings->op_mode != BME68X_SLEEP_MODE) {
        ret = bme68x_get_config(&dev[i]);
        if (ret != BME68X_OK) {
            printf("[ERROR]: Failed to read sensor %d configuration\n", i);
            return ret;
        }
        dev[i].config.sensors.os_hum = sensor_settings->humidity_oversampling;
        dev[i].config.sensors.os_temp = sensor_settings->temperature_oversampling;
        dev[i].config.sensors.os_pres = sensor_settings->pressure_oversampling;
        dev[i].config.heater.enable = BME68X_ENABLE;
        if (sensor_settings->op_mode == BME68X_FORCED_MODE) {
            dev[i].config.heater.heatr_temp = sensor_settings->heater_temperature;
            dev[i].config.heater.heatr_dur = sensor_settings->heater_duration;
        } else {
            uint16_t sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (bme68x_get_measure_duration(&dev[i]) / INT64_C(1000));
            dev[i].config.heater.heatr_temp_prof = sensor_settings->heater_temperature_profile;
            dev[i].config.heater.heatr_dur_prof = sensor_settings->heater_duration_profile;
            dev[i].config.heater.shared_heatr_dur = sharedHeaterDur;
            dev[i].config.heater.profile_len = sensor_settings->heater_profile_len;
        }
        if ((ret = bme68x_apply_config(&dev[i])) != BME68X_OK) {
            printf("[ERROR]: Failed to apply sensor %d configuration\n", i);
        }
    }
    if ((ret = bme68x_start_measure(&dev[i])) != BME68X_OK) {
        printf("[ERROR]: Failed to set op_mode for sensor %d\n", i);
    }
    return ret;
}

bsec_library_return_t process_data(int64_t tstamp_ns, bme68x_data_t data, int32_t bsec_process_data, uint8_t i)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
	bsec_library_return_t bsec_status = BSEC_OK;
    uint8_t n = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[n].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n].signal = temp_offset;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_TEMPERATURE))
    {
#ifdef BME68X_USE_FPU
        inputs[n].sensor_id = BSEC_INPUT_TEMPERATURE;
#else
        inputs[n].sensor_id = BSEC_INPUT_TEMPERATURE / 100.0f;
#endif
        inputs[n].signal = data.temperature;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HUMIDITY))
    {
#ifdef BME68X_USE_FPU
        inputs[n].sensor_id = BSEC_INPUT_HUMIDITY;
#else
        inputs[n].sensor_id = BSEC_INPUT_HUMIDITY / 1000.0f;
#endif
        inputs[n].signal = data.humidity;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[n].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n].signal = data.pressure;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_GASRESISTOR) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n].signal = data.gas_resistance;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PROFILE_PART) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n].signal = (dev[i].config.op_mode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }

    if (n > 0)
    {
        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
        bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
        uint8_t num_bsec_outputs = 0;
        uint8_t index = 0;
	    output_t output = {0};
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        /* Perform processing of the data by BSEC
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme68x_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps_m(inst[i], inputs, n, bsec_outputs, &num_bsec_outputs);
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
                    output.stabStatus = bsec_outputs[index].signal;
                    break;
				case BSEC_OUTPUT_RUN_IN_STATUS:
                    output.runInStatus = bsec_outputs[index].signal;
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
        printf("IAQ: %f\n", output.iaq);
        printf("IAQ ACC: %d\n", output.iaq_accuracy);
        printf("STATIC IAQ: %f\n", output.static_iaq);
        printf("RAW TEMP: %f\n", output.raw_temp);
        printf("RAW HUM: %f\n", output.raw_humidity);
        printf("TEMP: %f\n", output.temperature);
        printf("HUM: %f\n", output.humidity);
        printf("PRESS: %f\n", output.raw_pressure);
        printf("RAW GAS: %f\n", output.raw_gas);
        printf("RAW GAS %%: %f\n", output.gas_percentage);
        printf("CO2 EQUIV: %f\n", output.co2_equivalent);
        printf("BREATH VOC EQUIV: %f\n", output.breath_voc_equivalent);
        printf("STAB STATUS: %f\n", output.stabStatus);
        printf("RUN IN STATUS: %f\n", output.runInStatus);
        printf("BSEC STATUS: %d\n", bsec_status);
        puts("");
    }
    return bsec_status;
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
        bsec_config_len = config_load(bsec_config, sizeof(bsec_config));
        if (bsec_config_len > 0) {
            if ((res = bsec_set_configuration_m(inst[i], bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer))) != BSEC_OK) {
                printf("[ERROR] Configuration failed for inst[%d]: %d.\n", i, res);
                return 1;
            }
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

    uint64_t time_stamp = 0;
    for (i = 0; i < BME68X_NUMOF; i++) {
        memset(&sensor_settings[i], 0, sizeof(sensor_settings[i]));
    }

    puts("Init done.");

    while (1) {
        for (i = 0; i < BME68X_NUMOF; i++) {
            time_stamp = ztimer64_now(ZTIMER64_USEC) * 1000;
            if (time_stamp >= (uint64_t)sensor_settings[i].next_call) {
                // ask BSEC for sensor settings
                res = bsec_sensor_control_m(inst[i], time_stamp, &sensor_settings[i]);
                if (res != BSEC_OK) {
                    if (res < BSEC_OK) {
                        printf("[ERROR] bsec_sensor_control_m for inst[%d]: %d\n", i, res);
                        return 1;
                    } else {
                        printf("[WARNING] bsec_sensor_control for inst[%d]: %d\n", i, res);
                    }
                }
                // configure sensor as dictated
                if ((sensor_settings[i].op_mode == BME68X_FORCED_MODE) || (dev[i].config.op_mode != sensor_settings[i].op_mode)) {
                    apply_configuration(&sensor_settings[i], i);
			    }
                // - read sensor data
				if (sensor_settings[i].trigger_measurement && sensor_settings[i].op_mode != BME68X_SLEEP_MODE) {
                    uint8_t n_data = 0;
                    bme68x_get_measure_data(&dev[i], sensor_data, &n_data);
                    for (int j = 0; j < n_data; j++) {
                        bme68x_data_t data = sensor_data[j];
                        if (data.status & BME68X_GASM_VALID_MSK) {
                            if ((res = process_data(time_stamp, data, sensor_settings[i].process_data, i)) != BSEC_OK) {
                                printf("[ERROR] process_data failed for inst[%d]: %d\n", i, res);
                                return 1;
                            }
                        }
                        if (sensor_settings[i].op_mode == BME68X_FORCED_MODE) break;
                    }
                }
            }
        }
    }

    return 0;
}
