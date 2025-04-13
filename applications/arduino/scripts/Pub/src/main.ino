
#include <Arduino.h>
#include <mbed.h>
#include "uros_task.h"


#define LED_PIN 25


rtos::Thread uros_thread;
rtos::Thread led_thread;
extern volatile int32_t count_up;
extern volatile int32_t count_down;


void led_task();
void uros_task();


void setup() {
    led_thread.start(led_task);
    led_thread.set_priority(osPriorityLow);

    uros_thread.start(uros_task);
    uros_thread.set_priority(osPriorityNormal);
}


void loop() {
    delay(0xffffffff);
}


void led_task() {

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    for (;;) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);

        count_up += 1;
        count_down -= 1;
    }
}


void uros_task() {

    uros_init();

    for (;;) {
        uros_spin();
    }
}