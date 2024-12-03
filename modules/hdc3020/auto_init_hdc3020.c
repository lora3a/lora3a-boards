#include "log.h"
#include "assert.h"
#include "saul_reg.h"
#include "hdc3020.h"
#include "hdc3020_params.h"

static hdc3020_t hdc3020_devs[HDC3020_NUMOF];
static saul_reg_t saul_entries[HDC3020_NUMOF * 2];

void auto_init_hdc3020(void)
{
    if (HDC3020_NUMOF!=HDC3020_SAULINFO_NUMOF) {
        puts("HDC3020: configuration error, HDC3020_NUMOF!=HDC3020_SAULINFO_NUMOF");
        assert(0);
    }

    for (unsigned int i = 0; i < HDC3020_NUMOF; i++) {
        int res;

        LOG_DEBUG("[auto_init_saul] initializing hdc3020 #%u\n", i);

        res = hdc3020_init(&hdc3020_devs[i], &hdc3020_params[i]);
        if (res < 0) {
            LOG_ERROR("[auto_init_saul] error initializing hdc3020 #%u\n", i);
            continue;
        }

        saul_entries[2 * i].dev = &hdc3020_devs[i];
        saul_entries[2 * i].name = hdc3020_saul_info[i].name;
        saul_entries[2 * i].driver = &hdc3020_saul_temp_driver;
        saul_reg_add(&saul_entries[2 * i]);

        saul_entries[2 * i + 1].dev = &hdc3020_devs[i];
        saul_entries[2 * i + 1].name = hdc3020_saul_info[i].name;
        saul_entries[2 * i + 1].driver = &hdc3020_saul_hum_driver;
        saul_reg_add(&saul_entries[2 * i + 1]);
    }
}

