#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "od.h"
#include "fmt.h"
#include "thread.h"
#include "acme_lora.h"
#include "stdio_base.h"

uint16_t uint16(const char *ptr)
{
    uint16_t result = ptr[0] + (ptr[1] << 8);

    return result;
}

void dump_message(const char *message, size_t len, int16_t *rssi, int8_t *snr)
{
    (void)rssi;
    (void)snr;

    stdio_write(message, len);
    puts("");
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
