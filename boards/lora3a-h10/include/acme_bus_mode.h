#ifndef ACME_BUS_MODE_H
#define ACME_BUS_MODE_H

#define MODE_UNDEF 0
#define MODE_UART 1
#define MODE_SPI 2
#define MODE_I2C 3

#define BUS_MODE_TO_STR(mode) \
    ((mode) == MODE_UNDEF ? "MODE_UNDEF" : \
     ((mode) == MODE_UART ? "MODE_UART" : \
      ((mode) == MODE_SPI ? "MODE_SPI" : \
       ((mode) == MODE_I2C ? "MODE_I2C" : "UNKNOWN_MODE"))))

#ifndef ACME0_BUS_MODE
#define ACME0_BUS_MODE MODE_UNDEF
#endif

#ifndef ACME1_BUS_MODE
#define ACME1_BUS_MODE MODE_UNDEF
#endif

#ifndef ACME2_BUS_MODE
#define ACME2_BUS_MODE MODE_UNDEF
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* ACME_BUS_MODE_H */
