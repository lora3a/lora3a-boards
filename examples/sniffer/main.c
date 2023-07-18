#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "od.h"
#include "fmt.h"
#include "thread.h"
#include "acme_lora.h"

uint16_t uint16(const char *ptr)
{
    uint16_t result = ptr[0] + (ptr[1] << 8);

    return result;
}

void dump_message(const char *message, size_t len, int16_t *rssi, int8_t *snr)
{
    uint16_t cnt, net;
    uint8_t src, dst;
    char *ptr = (char *)message;
    size_t n = len;

    printf("{Received: %u bytes, RSSI: %i, SNR: %i}\n", len, *rssi, *snr);
    if (len > 8 && uint16(message) == ACME_SIGNATURE) {
        ptr += 2; n -= 2;
        cnt = uint16(ptr);
        ptr += 2; n -= 2;
        net = uint16(ptr);
        ptr += 2; n -= 2;
        dst = (uint8_t)ptr[0];
        ptr++; n--;
        src = (uint8_t)ptr[0];
        ptr++; n--;
        printf("ACME packet: counter=%04x, network=%04x, dst=%02x, src=%02x. Payload:\n", cnt, net,
               dst, src);
    }
    else {
        printf("RAW packet:\n");
    }
    od_hex_dump(ptr, n < MAX_PACKET_LEN ? n : MAX_PACKET_LEN, 0);
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
