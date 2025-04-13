/**
 * @file uros_task.cpp
 * @brief Wrapping all wanted microROS routines in a single file
 * 
 * Information related to objects: allocator, support, node, executor, etc., can
 * be found here
 * https://micro.ros.org/docs/tutorials/programming_rcl_rclc/micro-ROS/
 * https://micro.ros.org/docs/concepts/client_library/execution_management/
 * 
 * Information related to microros reconnection to agent:
 * https://discourse.ros.org/t/hard-liveliness-check-in-micro-ros/24891
 * https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
 * 
 * This file is intended to serve as both an example and a foundation for future 
 * projects that require more complex routines. 
 */


 #include <Arduino.h>
 #include <rcl/rcl.h>
 #include <rclc/rclc.h>
 #include <rclc/executor.h>
 #include <micro_ros_platformio.h>
 #include <std_msgs/msg/int32.h>
 #include "uros_task.h"
 
 
 #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
 #error This example is only avaliable for Arduino framework with serial transport.
 #endif
 
 
 #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uros_error_loop();}}
 #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
 #define REPETITIVE_TASK_MS(MS, X)  do { \
     static volatile int64_t init = -1; \
     if (init == -1) { init = uxr_millis();} \
     if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
 } while (0);\
 
 #define SUPPORT_INT32 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32)
 
 #define publisher_init(...) RCCHECK(rclc_publisher_init_default(__VA_ARGS__))
 #define subscriber_init(...) RCCHECK(rclc_subscription_init_default(__VA_ARGS__))
 #define timer_init(timer, support, period, callback) \
         RCCHECK(rclc_timer_init_default(timer, support, RCL_MS_TO_NS(period), callback))
 
 
 typedef rcl_allocator_t uros_allocator;
 typedef rclc_support_t uros_support;
 typedef rcl_node_t uros_node;
 typedef rclc_executor_t uros_executor;
 typedef rcl_publisher_t uros_publisher;
 typedef rcl_subscription_t uros_subscriber;
 typedef rcl_timer_t uros_timer;
 
 
 // Parameters
 const static uint8_t num_handles = 2;
 const static uint32_t timer_period = 500;  // ms
 const static uint32_t spin_period = 100;  // ms
 
 const char * node_id = "pio_uros_base_node";
 const char * topic1 = "/test1";
 const char * topic2 = "/test2";
 const char * topic_sub1 = "/cmd_topic";
 
 
 // File scope variables
 static uros_allocator allocator;
 static uros_support support;
 static uros_node node;
 static uros_executor executor;
 static uros_publisher publisher1;
 static uros_publisher publisher2;
 static uros_subscriber sub1;
 static uros_timer timer1;
 static std_msgs__msg__Int32 sub1_invalue;
 
 
 // Exported variables
 volatile int32_t count_up;
 volatile int32_t count_down;
 
 
 static void uros_create_entities();
 static void uros_destroy_entities();
 static void uros_reconnect();
 static void uros_error_loop();
 static void uros_publish_data(uros_timer * timer, int64_t last_call);
 static void uros_sub1_callback(const void * msgin);
 
 
 void uros_init() {
     // Config serial transport
     Serial.begin(115200);
     set_microros_serial_transports(Serial);
 
     // This line blocks until a microros agent answer with a ping
     while (RMW_RET_OK != rmw_uros_ping_agent(10, 1)) {
         delay(100);
     }
 
     uros_create_entities();
 }
 
 
 void uros_spin() {
     // This is important
     // We want executor to check if something is ready to run and return 
     // inmediately if nothing is ready. 
     // Since this is underlying mbed we are freeing resources
     // so another tasks can run when using arduino-delay function.
     RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
     delay(spin_period);
 
     // reconnection task (check every 3s)
     REPETITIVE_TASK_MS(
         3000,
         if (RMW_RET_OK != rmw_uros_ping_agent(10, 1)) uros_reconnect()
     )
 }
 
 
 // ===== Private methods =====
 
 static void uros_create_entities() {
     // Memory allocator object:
     allocator = rcl_get_default_allocator();
 
     // Starting uros node
     RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
     RCCHECK(rclc_node_init_default(&node, node_id, "", &support));
 
     // Creating publishers
     publisher_init(&publisher1, &node, SUPPORT_INT32, topic1)
     publisher_init(&publisher2, &node, SUPPORT_INT32, topic2)
 
     // Configuring subscribers
     subscriber_init(&sub1, &node, SUPPORT_INT32, topic_sub1)
 
     // Configuring timers
     timer_init(&timer1, &support, timer_period, uros_publish_data)
 
     // Configuring executors
     RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
     RCCHECK(rclc_executor_add_subscription( &executor, &sub1, &sub1_invalue, &uros_sub1_callback, ON_NEW_DATA));
     RCCHECK(rclc_executor_add_timer( &executor, &timer1));
 
     // Init any other thing related to your application
     count_up = 0;
     count_down = 0;
 }
 
 
 static void uros_destroy_entities() {
     rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
     (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
 
     rcl_publisher_fini(&publisher1, &node);
     rcl_publisher_fini(&publisher2, &node);
     rcl_subscription_fini(&sub1, &node);
     rcl_timer_fini(&timer1);
     rclc_executor_fini(&executor);
     rcl_node_fini(&node);
     rclc_support_fini(&support);
 }
 
 
 static void uros_reconnect() {
     uros_destroy_entities();
     uros_init();
 }
 
 
 // Error handling. Indicating uros errors through rpico led.
 static void uros_error_loop() {
     while(1) {
         digitalWrite(25, HIGH);
         delay(50);
         digitalWrite(25, LOW);
         delay(50);
     }
 }
 
 // Periodically exposing data via topics
 static void uros_publish_data(uros_timer * timer, int64_t last_call) {
     std_msgs__msg__Int32 msg;
     msg.data = count_up;
  
     RCSOFTCHECK(rcl_publish(&publisher1, &msg, NULL));
 
     msg.data = count_down;
     RCSOFTCHECK(rcl_publish(&publisher2, &msg, NULL));
 }
 
 
 static void uros_sub1_callback(const void * msgin) {
     // Storing incoming value altough not used for this example
     int32_t data = ((std_msgs__msg__Int32 *) msgin) -> data;
 
     count_up = data;
     count_down = data;
 }