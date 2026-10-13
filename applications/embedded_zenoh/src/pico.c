#include "demo.h"
#include "config.h"
#include "pico/cyw43_arch.h"
bool cp_keep_running(void) { return true; }
void cp_set_led(bool enabled) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, enabled); }
int app_main(void) { return cp_demo_run(ZENOH_CONFIG_CONNECT, CP_ROS_DOMAIN_ID); }
