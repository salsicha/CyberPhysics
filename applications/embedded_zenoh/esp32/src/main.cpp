#include <Arduino.h>
#include <ETH.h>
extern "C" {
#include "demo.h"
}
#ifndef CP_ZENOH_LOCATOR
#define CP_ZENOH_LOCATOR "tcp/192.168.1.193:7448"
#endif
#ifndef CP_ROS_DOMAIN_ID
#define CP_ROS_DOMAIN_ID 0
#endif
/* Same LAN8720 wiring as the previous Ethernet demo; DHCP replaces its
 * hard-coded 10.4.4.x addresses. Override the locator for your router. */
bool cp_keep_running(void) { return true; }
void cp_set_led(bool enabled) { digitalWrite(2, enabled ? HIGH : LOW); }
void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    ETH.begin(0, 5, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
    ETH.setHostname("cyberphysics-zenoh");
    while (!ETH.linkUp() || ETH.localIP() == IPAddress(0,0,0,0)) delay(100);
    Serial.println(ETH.localIP());
    static char locator[] = CP_ZENOH_LOCATOR;
    cp_demo_run(locator, CP_ROS_DOMAIN_ID);
}
void loop() { delay(100); }
