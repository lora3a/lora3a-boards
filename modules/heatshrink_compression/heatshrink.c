#include <string.h>
#include <stdio.h>

#include "heatshrink_encoder.h"
#include "heatshrink_decoder.h"

#ifndef HEATSHRINK_BUFSIZE
#define HEATSHRINK_BUFSIZE (256U)
#endif

static uint8_t _buf[HEATSHRINK_BUFSIZE];

static heatshrink_encoder _encoder;

int heatshrink_compress(const uint8_t *input, size_t input_len, uint8_t *output, size_t output_len)
{
    memset(_buf, 0, sizeof(_buf));

    heatshrink_encoder_reset(&_encoder);

    size_t n = input_len;
    uint8_t *inpos = (uint8_t*)input;
    uint8_t *outpos = _buf;

    while(1) {
        size_t n_sunk = 0;
        if (n) {
            heatshrink_encoder_sink(&_encoder, inpos, n, &n_sunk);
            if (n_sunk) {
                inpos += n_sunk;
                n -= n_sunk;
            }
            size_t written = 0;
            heatshrink_encoder_poll(&_encoder, outpos, (_buf + sizeof(_buf) - outpos), &written);
            outpos += written;
        }
        else {
            while (heatshrink_encoder_finish(&_encoder) == HSER_FINISH_MORE) {
                size_t written = 0;
                heatshrink_encoder_poll(&_encoder, outpos, (_buf + sizeof(_buf) - outpos), &written);
                outpos += written;
            }
            break;
        }
    }

    n = (outpos - _buf);
    if (n < output_len) {
        memcpy(output, _buf, n);
    } else {
        n = 0;
    }
    return n;
}
