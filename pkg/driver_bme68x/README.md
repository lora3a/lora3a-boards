# BME68x vendor driver

## Introduction

The [BME68x_driver](https://github.com/boschsensortec/BME68x-Sensor-API) is an
I2C/SPI API for BME68x sensor.

The library is written and maintained by Bosch Sensortec. It is platform
independent, as long as the right drivers are available for the given MCU.

In addition, this driver can use floating point if available on your MCU.
By default, this package does not use it.

## Usage

Refer to the code documentation at
[GitHub](https://github.com/boschsensortec/BME68x-Sensor-API) for more information
on the API.

## RIOT-OS interface

BME68x sensors are connected either via I2C or SPI. Which interface is used by
which BME68x sensor is defined in the `bme68x_params` parameters. The
respective implementation is enabled by the modules `bme68x_i2c` and
`bme68x_spi`. Both I2C and SPI can be used in one application.
```
USEMODULE='bme68x_spi bme68x_i2c' make BOARD=... -C tests/drivers/bme68x
```

In order to use floating point, you can enable module `bme68x_fp` variable:
```
USEMODULE='bme68x_fp bme68x_i2c' make BOARD=... -C tests/drivers/bme68x
```

The following callbacks add support for the included drivers via I2C and SPI
peripherals:

* `bme68x_i2c_read_hal`
* `bme68x_i2c_write_hal`
* `bme68x_spi_read_hal`
* `bme68x_spi_write_hal`
