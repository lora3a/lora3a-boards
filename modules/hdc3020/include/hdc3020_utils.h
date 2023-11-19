#ifndef HDC3020_UTILS_H
#define HDC3020_UTILS_H

#include "board.h"
#include "hdc3020.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HDC3020_CRC8_POLYNOMIAL     0x31
#define HDC3020_CRC8_INITIALIZATION 0xFF
#define HDC3020_CRC8_FINAL_XOR      0x00


// Function to calculate CRC-8
uint8_t calculateCRC8(const uint8_t *data, size_t size)
{
    uint8_t crc = HDC3020_CRC8_INITIALIZATION;

    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ HDC3020_CRC8_POLYNOMIAL;
            }
            else {
                crc <<= 1;
            }
        }
    }

    return crc ^ HDC3020_CRC8_FINAL_XOR;
}

uint16_t rh_to_rh_hdc3020(const double hum)
{
    return hum * (0xFFFF) / 100.;
}

uint16_t temp_to_temp_hdc3020(const double temp)
{
    return (temp + 45) * (0xFFFF) / 175.;
}

double rh_hdc3020_to_rh(const uint16_t data)
{
    return (data) * 100. / (0xFFFF);
}

double temp_hdc3020_to_temp(const uint16_t data)
{
    return (data) * 175. / (0xFFFF) - 45;
}

#ifdef __cplusplus
}
#endif

#endif /* HDC3020_UTILS_H */
