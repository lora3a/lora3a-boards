#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "od.h"
#include "fmt.h"
#include "thread.h"
#include "acme_lora.h"
#include "stdio_base.h"
#include "base64.h"

uint16_t uint16(const char *ptr)
{
    uint16_t result = ptr[0] + (ptr[1] << 8);

    return result;
}

void dump_message(const char *message, size_t len, int16_t *rssi, int8_t *snr)
{
    size_t size_data = base64_estimate_encode_size(len);
    char *base64_data = malloc(size_data + 1);

    base64_data[size_data] = 0;
    int error_code = base64_encode(message, len, base64_data, &size_data);

    if (error_code != BASE64_SUCCESS) {
        printf("ERROR: %d\n", error_code);
        free(base64_data);
        return;
    }

    printf("{\"rssi\": %d, \"snr\": %d, \"payload\": \"%s\" }\n",
           *rssi, *snr, base64_data);

    free(base64_data);
}

int main(void)
{
    lora_state_t lora;

    memset(&lora, 0, sizeof(lora));
    lora.bandwidth = DEFAULT_LORA_BANDWIDTH;
    printf("LORA.BW %d\n", lora.bandwidth);
    lora.spreading_factor = DEFAULT_LORA_SPREADING_FACTOR;
    lora.coderate = DEFAULT_LORA_CODERATE;
    lora.channel = DEFAULT_LORA_CHANNEL;
    lora.power = DEFAULT_LORA_POWER;
    lora.data_cb = *dump_message;
    if (lora_init(&(lora)) != 0) {
        return 1;
    }
    lora_listen();
    puts("Listen mode set; waiting for messages");
    thread_sleep();
    return 0;
}
