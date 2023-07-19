#include <stdio.h>
#include "shell.h"
#include "saml21_cpu_debug.h"

int debug_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    saml21_cpu_debug();
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "debug", "Show SAML21 peripherals config", debug_cmd },
    { NULL, NULL, NULL },
};

int main(void)
{
    if (IS_USED(MODULE_H10_ADC)) {
        extern void auto_init_h10_adc(void);
        auto_init_h10_adc();
    }

    if (IS_USED(MODULE_SENSEAIR)) {
        extern void auto_init_senseair(void);
        auto_init_senseair();
    }

    if (IS_USED(MODULE_HDC3020)) {
        extern void auto_init_hdc3020(void);
        auto_init_hdc3020();
    }

    if (IS_USED(MODULE_LIS2DW12)) {
        extern void auto_init_lis2dw12(void);
        auto_init_lis2dw12();
    }

    puts("Welcome to RIOT!\n");
    puts("Type `help` for help, type `saul` to see all SAUL devices\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
