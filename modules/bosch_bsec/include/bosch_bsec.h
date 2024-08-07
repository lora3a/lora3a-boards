#ifndef BOSCH_BSEC_H
#define BOSCH_BSEC_H
#include "bme68x.h"
#include "bme68x_params.h"
#include "bsec_interface_multi.h"
#include "bsec_selectivity.h"
#include "bsec_errno.h"

#ifndef BSEC_SAMPLE_RATE
#define BSEC_SAMPLE_RATE BSEC_SAMPLE_RATE_LP
#endif

extern void *bsec_inst[BME68X_NUMOF];
extern uint8_t bsec_state[BME68X_NUMOF][BSEC_MAX_STATE_BLOB_SIZE];

bsec_library_return_t bsec_init(void);
bsec_library_return_t bsec_deinit(void);
int bsec_apply_configuration(bsec_bme_settings_t *sensor_settings, bme68x_t *dev, int i);
bsec_library_return_t bsec_process_data(
  int64_t tstamp_ns, bme68x_data_t data, int32_t process_data, bme68x_t *dev, int i, bsec_output_t *bsec_outputs, uint8_t *num_bsec_outputs
);

#endif
