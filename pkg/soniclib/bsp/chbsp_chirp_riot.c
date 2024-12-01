#include <stdio.h>
#include "ztimer.h"
#include "periph/gpio.h"
#include "periph/i2c.h"

#include "soniclib.h"
#include "soniclib_params.h"
#include "invn/soniclib/chirp_bsp.h"

ch_dev_t soniclib_devices[SONICLIB_NUMOF];
ch_group_t soniclib_group;

void chbsp_delay_us(uint32_t us) { ztimer_spin(ZTIMER_USEC, us); }
void chbsp_delay_ms(uint32_t ms) { ztimer_sleep(ZTIMER_MSEC, ms); }
uint32_t chbsp_timestamp_ms(void) { return ztimer_now(ZTIMER_MSEC); }

void chbsp_reset_assert(void) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < SONICLIB_NUMOF; dev_num++)
    {
        gpio_write(soniclib_params[dev_num].reset_pin, SONICLIB_RESET_PIN_ACTIVE);
    }
}

void chbsp_reset_release(void) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < SONICLIB_NUMOF; dev_num++)
    {
        gpio_write(soniclib_params[dev_num].reset_pin, (SONICLIB_RESET_PIN_ACTIVE ? 0 : 1));
    }
}

void chbsp_program_enable(ch_dev_t *dev_ptr) {
    uint8_t dev_num = ch_get_dev_num(dev_ptr);
    gpio_write(soniclib_params[dev_num].prog_pin, SONICLIB_PROG_PIN_ACTIVE);
}

void chbsp_program_disable(ch_dev_t *dev_ptr) {
    uint8_t dev_num = ch_get_dev_num(dev_ptr);
    gpio_write(soniclib_params[dev_num].prog_pin, (SONICLIB_PROG_PIN_ACTIVE ? 0 : 1));
}

static void _int1_callback(void *arg) {
    size_t num = (size_t)arg;
    uint8_t dev_num = (uint8_t)(num & 0xff);
    ch_interrupt(&soniclib_group, dev_num);
}

void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr))
            gpio_init(soniclib_params[dev_num].io_pin, GPIO_OUT);
    }
}

void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr) {
    uint8_t dev_num = ch_get_dev_num(dev_ptr);
    gpio_init(soniclib_params[dev_num].io_pin, GPIO_OUT);
}

void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr))
            gpio_init(soniclib_params[dev_num].io_pin, GPIO_IN);
    }
}

void chbsp_group_int1_clear(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr))
            gpio_clear(soniclib_params[dev_num].io_pin);
    }
}

void chbsp_group_int1_set(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr))
            gpio_set(soniclib_params[dev_num].io_pin);
    }
}

void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    size_t num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num = dev_num;
            gpio_init_int(soniclib_params[dev_num].io_pin, GPIO_IN, GPIO_FALLING, _int1_callback, (void *)num);
        }
    }
}

void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr) {
    uint8_t dev_num;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
    {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            gpio_irq_disable(soniclib_params[dev_num].io_pin);
        }
    }
}

void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr) {
    uint8_t dev_num = ch_get_dev_num(dev_ptr);
    gpio_irq_disable(soniclib_params[dev_num].io_pin);
}

int chbsp_i2c_init(void) {
    for (uint8_t dev_num = 0; dev_num < SONICLIB_NUMOF; dev_num++) {
        // might init same bus multiple times
        i2c_init(soniclib_params[dev_num].i2c_bus);
    }
    return 0;
}

uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) *grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr) {
    uint8_t res = 1;
    if (io_index <= SONICLIB_NUMOF) {
        info_ptr->bus_num = soniclib_params[io_index].i2c_bus;
        info_ptr->address = soniclib_params[io_index].i2c_addr;
        info_ptr->drv_flags = 0;    // no special I2C handling by SonicLib driver is needed
        res = 0;
    }
    return res;
}

int chbsp_i2c_write(ch_dev_t *dev_ptr, const uint8_t *data, uint16_t num_bytes) {
    int res;
    uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
    uint8_t bus_num  = ch_get_bus(dev_ptr);
    i2c_acquire(bus_num);
    res = i2c_write_bytes(bus_num, i2c_addr, data, num_bytes, 0);
    i2c_release(bus_num);
    return res;
}

int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
    int res;
    uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
    uint8_t bus_num  = ch_get_bus(dev_ptr);
    i2c_acquire(bus_num);
    res = i2c_write_regs(bus_num, i2c_addr, mem_addr, data, num_bytes, 0);
    i2c_release(bus_num);
    return res;
}

int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
    int res;
    uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
    uint8_t bus_num  = ch_get_bus(dev_ptr);
    i2c_acquire(bus_num);
    res = i2c_read_bytes(bus_num, i2c_addr, data, num_bytes, 0);
    i2c_release(bus_num);
    return res;
}

int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
    int res;
    uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
    uint8_t bus_num  = ch_get_bus(dev_ptr);
    i2c_acquire(bus_num);
    res = i2c_read_regs(bus_num, i2c_addr, mem_addr, data, num_bytes, 0);
    i2c_release(bus_num);
    return res;
}

void chbsp_i2c_reset(ch_dev_t * dev_ptr) {
    (void)dev_ptr;
}

void chbsp_print_str(const char *str) { printf("%s", str); }
