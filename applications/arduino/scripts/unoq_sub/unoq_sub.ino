// Arduino UNO Q counterpart of ../Sub (micro-ROS Int32 subscriber driving an
// LED), split across the board's two processors:
//   MCU (this sketch, arduino:zephyr:unoq): provides the RPC set_led(int32)
//     through Arduino_RouterBridge;
//   Linux side (unoq_sub_node.py in this folder, in a ROS 2 container):
//     subscribes to /micro_ros_arduino_subscriber (std_msgs/Int32) and calls
//     set_led through the arduino-router socket. No micro-ROS is involved.
// LED_BUILTIN is the red channel of RGB LED 3, which is active low.

#include <Arduino_RouterBridge.h>

// Same rule as ../Sub: 0 switches the LED off, anything else on.
bool setLed(int32_t data) {
  digitalWrite(LED_BUILTIN, data == 0 ? HIGH : LOW);
  return data != 0;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // off
  Bridge.begin();
  // provide_safe: digitalWrite runs with the sketch's loop, as the UNO Q
  // user manual recommends for Arduino API calls
  Bridge.provide_safe("set_led", setLed);
}

void loop() {}
