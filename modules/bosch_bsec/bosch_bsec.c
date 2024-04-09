#include <stdio.h>
#include "bosch_bsec.h"

bsec_library_return_t bosch_bsec_init(void) {
    bsec_version_t  version;
    bsec_library_return_t res;
    bsec_get_version(&version);
    printf("BSEC version: %d.%d.%d.%d\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    if((res = bsec_init()) != BSEC_OK) {
        printf("BSEC init returned %d\n", res);
    }
    return res;
}
