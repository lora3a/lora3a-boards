#ifndef ACME_BUS_MODE_H
#define ACME_BUS_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#define MODE_POWERED 0
#define MODE_UART 1
#define MODE_SPI 2
#define MODE_I2C 3

#define BUS_MODE_TO_STR(mode) \
    ((mode) == MODE_POWERED ? "MODE_POWERED" : \
      ((mode) == MODE_UART ? "MODE_UART" : \
      ((mode) == MODE_SPI ? "MODE_SPI" : \
       ((mode) == MODE_I2C ? "MODE_I2C" : "UNKNOWN_MODE"))))

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* ACME_BUS_MODE_H */
