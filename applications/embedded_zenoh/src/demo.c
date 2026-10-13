#include "demo.h"
#include "picoros.h"
#include <stdio.h>
#include <string.h>

/* Jazzy RIHS01 hashes. The runtime test compares these with ROS-generated types.
 * The historical dds_ type-name component is also used by native rmw_zenoh. */
#define INT32_TYPE "std_msgs::msg::dds_::Int32"
#define INT32_HASH "b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb"
#define STRING_TYPE "std_msgs::msg::dds_::String"
#define STRING_HASH "df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
#define MAX_NAME 120u

static picoros_publisher_t status_pub = {
    .topic = {"micro_ros_arduino_status", INT32_TYPE, INT32_HASH}
};
static picoros_publisher_t response_pub = {
    .topic = {"micro_ros_response", STRING_TYPE, STRING_HASH}
};
static bool cdr_header(const uint8_t *data, size_t size) {
    return size >= 8 && data[0] == 0 && data[1] <= 1 && data[2] == 0 && data[3] == 0;
}
static uint32_t read_u32(const uint8_t *p, bool little) {
    if (little) return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
    return (uint32_t)p[3] | (uint32_t)p[2]<<8 | (uint32_t)p[1]<<16 | (uint32_t)p[0]<<24;
}
static void write_u32(uint8_t *p, uint32_t value) {
    for (unsigned i=0; i<4; ++i) p[i] = (uint8_t)(value >> (8*i));
}
static void led_callback(uint8_t *data, size_t size) {
    if (!cdr_header(data, size) || size != 8) return;
    uint32_t value = read_u32(data+4, data[1] == 1);
    cp_set_led(value != 0);
    uint8_t response[8] = {0, 1, 0, 0};
    write_u32(response+4, value);
    picoros_publish(&status_pub, response, sizeof response);
}
static void name_callback(uint8_t *data, size_t size) {
    if (!cdr_header(data, size)) return;
    uint32_t n = read_u32(data+4, data[1] == 1);
    if (n == 0 || n > MAX_NAME+1 || n > size-8 || data[8+n-1] != 0 || memchr(data+8, 0, n-1)) return;
    uint8_t response[8 + 6 + MAX_NAME + 2] = {0, 1, 0, 0};
    memcpy(response+8, "Hello ", 6);
    memcpy(response+14, data+8, n-1);
    response[14+n-1] = '!';
    response[14+n] = 0;
    write_u32(response+4, n+7);
    picoros_publish(&response_pub, response, 8+n+7);
}
static picoros_subscriber_t led_sub = {
    .topic = {"micro_ros_arduino_subscriber", INT32_TYPE, INT32_HASH}, .user_callback = led_callback
};
static picoros_subscriber_t name_sub = {
    .topic = {"micro_ros_name", STRING_TYPE, STRING_HASH}, .user_callback = name_callback
};

static int run_session(char *locator, uint32_t domain) {
    picoros_interface_t interface = {.mode = "client", .locator = locator};
    picoros_node_t node = {.name = "micro_ros_platformio_node", .domain_id = domain};
    while (cp_keep_running()) {
        int result = picoros_interface_init(&interface);
        if (result == PICOROS_OK) break;
        if (result != PICOROS_NOT_READY) return 1;
        z_sleep_ms(250);
    }
    if (!cp_keep_running()) return 0;
    bool node_ok=false, status_ok=false, response_ok=false, led_ok=false, name_ok=false;
    int result=1;
    if (picoros_node_init(&node) != PICOROS_OK) goto cleanup;
    node_ok=true;
    if (picoros_publisher_declare(&node, &status_pub) != PICOROS_OK) goto cleanup;
    status_ok=true;
    if (picoros_publisher_declare(&node, &response_pub) != PICOROS_OK) goto cleanup;
    response_ok=true;
    if (picoros_subscriber_declare(&node, &led_sub) != PICOROS_OK) goto cleanup;
    led_ok=true;
    if (picoros_subscriber_declare(&node, &name_sub) != PICOROS_OK) goto cleanup;
    name_ok=true;
    printf("READY native Zenoh domain=%lu locator=%s\n", (unsigned long)domain, locator);
    fflush(stdout);
    while (cp_keep_running()) z_sleep_ms(100);
    result=0;
cleanup:
    if (name_ok) picoros_subscriber_drop(&name_sub);
    if (led_ok) picoros_subscriber_drop(&led_sub);
    if (response_ok) picoros_publisher_drop(&response_pub);
    if (status_ok) picoros_publisher_drop(&status_pub);
    if (node_ok) picoros_node_drop(&node);
    picoros_interface_close();
    cp_set_led(false);
    return result;
}

int cp_demo_run(char *locator, uint32_t domain) {
    while (cp_keep_running()) {
        int result = run_session(locator, domain);
        if (result == 0) return 0;
        /* Retry initialization failures. Established sessions use zenoh-pico's
         * automatic reconnect and declaration replay after transport loss. */
        z_sleep_ms(250);
    }
    return 0;
}
