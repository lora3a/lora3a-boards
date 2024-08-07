#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bosch_bsec.h"

#define BSEC_CHECK_INPUT(x, shift)  (x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR         UINT16_C(140)

#define ENABLE_DEBUG 0
#include "debug.h"

void *bsec_inst[BME68X_NUMOF];
uint8_t bsec_state[BME68X_NUMOF][BSEC_MAX_STATE_BLOB_SIZE] = {0};

static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};
static float temp_offset = 0.0f;

bsec_library_return_t bsec_init(void)
{
    bsec_library_return_t res;
    bsec_version_t version;
    size_t i;

    for (i = 0; i < BME68X_NUMOF; i++) {
        if ((bsec_inst[i] = malloc(bsec_get_instance_size_m())) == NULL) {
            DEBUG("[ERROR] Memory allocation failed for inst[%d].\n", i);
            return -1;
        }
        if ((res = bsec_init_m(bsec_inst[i])) != BSEC_OK) {
            DEBUG("[ERROR] Initialization failed for inst[%d]: %s.\n", i, bsec_errno(res));
            return res;
        }
        if ((res = bsec_set_configuration_m(bsec_inst[i], bsec_config_selectivity, sizeof(bsec_config_selectivity), work_buffer, sizeof(work_buffer))) != BSEC_OK) {
            DEBUG("[ERROR] Configuration failed for inst[%d]: %s.\n", i, bsec_errno(res));
            return res;
        }
        res = bsec_set_state_m(bsec_inst[i], bsec_state[i], sizeof(bsec_state[i]), work_buffer, sizeof(work_buffer));
        if (res != BSEC_OK) {
            DEBUG("[WARNING] Restoring BSEC state inst[%d]: %s.\n", i, bsec_errno(res));
        }
    }
    if ((res = bsec_get_version_m(bsec_inst[0], &version)) != BSEC_OK) {
        DEBUG("[ERROR] Read BSEC version failed: %s.\n", bsec_errno(res));
        return res;
    }
    DEBUG("BSEC version: %d.%d.%d.%d\n", version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    return BSEC_OK;
}

bsec_library_return_t bsec_deinit(void)
{
    bsec_library_return_t res;
    uint32_t actual_size;

    memset(&bsec_state, 0, sizeof(bsec_state));
    res = BSEC_OK;
    for (size_t i = 0; i < BME68X_NUMOF; i++) {
        if (bsec_inst[i]) {
            res = bsec_get_state_m(bsec_inst[i], 0, bsec_state[i], sizeof(bsec_state[i]), work_buffer, sizeof(work_buffer), &actual_size);
            if (res != BSEC_OK) {
                DEBUG("[ERROR] Reading current BSEC state failed for inst[%d]: %s.\n", i, bsec_errno(res));
            }
            free(bsec_inst[i]);
        }
    }
    return res;
}

int bsec_apply_configuration(bsec_bme_settings_t *sensor_settings, bme68x_t *dev, int i)
{
    int ret;
    dev->config.op_mode = sensor_settings->op_mode;
    if (sensor_settings->op_mode != BME68X_SLEEP_MODE) {
        ret = bme68x_get_config(dev);
        if (ret != BME68X_OK) {
            DEBUG("[ERROR]: Failed to read sensor %d configuration\n", i);
            return ret;
        }
        dev->config.sensors.os_hum = sensor_settings->humidity_oversampling;
        dev->config.sensors.os_temp = sensor_settings->temperature_oversampling;
        dev->config.sensors.os_pres = sensor_settings->pressure_oversampling;
        dev->config.heater.enable = BME68X_ENABLE;
        if (sensor_settings->op_mode == BME68X_FORCED_MODE) {
            dev->config.heater.heatr_temp = sensor_settings->heater_temperature;
            dev->config.heater.heatr_dur = sensor_settings->heater_duration;
        } else {
            uint16_t sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (bme68x_get_measure_duration(dev) / INT64_C(1000));
            dev->config.heater.heatr_temp_prof = sensor_settings->heater_temperature_profile;
            dev->config.heater.heatr_dur_prof = sensor_settings->heater_duration_profile;
            dev->config.heater.shared_heatr_dur = sharedHeaterDur;
            dev->config.heater.profile_len = sensor_settings->heater_profile_len;
        }
        if ((ret = bme68x_apply_config(dev)) != BME68X_OK) {
            DEBUG("[ERROR]: Failed to apply sensor %d configuration\n", i);
        }
    }
    if ((ret = bme68x_start_measure(dev)) != BME68X_OK) {
        DEBUG("[ERROR]: Failed to set op_mode for sensor %d\n", i);
    }
    return ret;
}

bsec_library_return_t bsec_process_data(
  int64_t tstamp_ns, bme68x_data_t data, int32_t process_data, bme68x_t *dev, int i, bsec_output_t *bsec_outputs, uint8_t *num_bsec_outputs
)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
    bsec_library_return_t bsec_status = BSEC_OK;
    uint8_t n = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[n].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n].signal = temp_offset;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_TEMPERATURE))
    {
        inputs[n].sensor_id = BSEC_INPUT_TEMPERATURE;
#ifdef BME68X_USE_FPU
        inputs[n].signal = data.temperature;
#else
        inputs[n].signal = data.temperature / 100.0f;
#endif
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_HUMIDITY))
    {
        inputs[n].sensor_id = BSEC_INPUT_HUMIDITY;
#ifdef BME68X_USE_FPU
        inputs[n].signal = data.humidity;
#else
        inputs[n].signal = data.humidity / 1000.0f;
#endif
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[n].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n].signal = data.pressure;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_GASRESISTOR) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n].signal = data.gas_resistance;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (BSEC_CHECK_INPUT(process_data, BSEC_INPUT_PROFILE_PART) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n].signal = (dev->config.op_mode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[n].time_stamp = tstamp_ns;
        n++;
    }
    if (n > 0)
    {
        /* Perform processing of the data by BSEC
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme68x_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps_m(bsec_inst[i], inputs, n, bsec_outputs, num_bsec_outputs);
    }
    return bsec_status;
}
