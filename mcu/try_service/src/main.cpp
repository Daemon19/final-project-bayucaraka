#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <gantry_interfaces/msg/object_positions.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_subscription_t object_positions_subscriber;
gantry_interfaces__msg__ObjectPositions object_positions_msg;
bool positions_received;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }

// Error handle loop
void error_loop() {
  while (1) {
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void mission(float x_payload, float y_payload, float x_dropping_zone,
             float y_dropping_zone) {
  while (1) {
    delay(1000);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void object_positions_callback(const void* msgin) {
  RCL_UNUSED(msgin);
  positions_received = true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &object_positions_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(gantry_interfaces, msg, ObjectPositions),
      "/object_positions"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &object_positions_subscriber, &object_positions_msg,
      &object_positions_callback, ON_NEW_DATA));

  msg.data = 0;
  positions_received = false;
}

void loop() {
  while (!positions_received) {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  positions_received = false;
  mission(object_positions_msg.payload.x, object_positions_msg.payload.y,
          object_positions_msg.dropping_zone.x,
          object_positions_msg.dropping_zone.y);
}