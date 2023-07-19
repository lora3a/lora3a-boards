#include "log.h"
#include "assert.h"
#include "saul_reg.h"
#include "lis2dw12.h"
#include "lis2dw12_params.h"

#define LIS2DW12_NUM    ARRAY_SIZE(lis2dw12_params)

#define LIS2DW12_SAULINFO_NUM   (sizeof(lis2dw12_saul_info) / \
                                 sizeof(lis2dw12_saul_info[0]))

static lis2dw12_t lis2dw12_devs[LIS2DW12_NUM];
static saul_reg_t saul_entries[LIS2DW12_NUM * 2];

void auto_init_lis2dw12(void)
{
    assert(LIS2DW12_NUM * 2 == LIS2DW12_SAULINFO_NUM);

    for (unsigned int i = 0; i < LIS2DW12_NUM; i++) {
        int res;

        LOG_DEBUG("[auto_init_saul] initializing lis2dw12 #%u\n", i);

        res = lis2dw12_init(&lis2dw12_devs[i], &lis2dw12_params[i]);
        if (res < 0) {
            LOG_ERROR("[auto_init_saul] error initializing lis2dw12 #%u\n", i);
            continue;
        }

        saul_entries[2 * i].dev = &lis2dw12_devs[i];
        saul_entries[2 * i].name = lis2dw12_saul_info[i].name;
        saul_entries[2 * i].driver = &lis2dw12_saul_driver;
        saul_reg_add(&saul_entries[2 * i]);

        saul_entries[2 * i + 1].dev = &lis2dw12_devs[i];
        saul_entries[2 * i + 1].name = lis2dw12_saul_info[i + 1].name;
        saul_entries[2 * i + 1].driver = &lis2dw12_saul_temp_driver;
        saul_reg_add(&saul_entries[2 * i + 1]);
    }
}
