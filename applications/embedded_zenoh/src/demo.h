#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Shared by the Linux transport test and the actual MCU firmware. */
int cp_demo_run(char *locator, uint32_t domain);
bool cp_keep_running(void);
void cp_set_led(bool enabled);
