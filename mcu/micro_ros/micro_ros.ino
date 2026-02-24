#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point_stamped.h>

#define LED_PIN 2

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

typedef enum {
  IDLE,
  MOVING,
  ERROR
} gantry_state_t;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

geometry_msgs__msg__PointStamped target_msg;
rcl_subscription_t target_subscriber;

geometry_msgs__msg__PointStamped status_msg;
rcl_publisher_t status_publisher;
rcl_timer_t status_timer;

gantry_state_t gantry_state;


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void set_target(const void *msgin) {
  RCLC_UNUSED(msgin);
  gantry_state = MOVING;
}

void publish_status(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
  RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
}

void move_to_target() {
  const float threshold = 10.0f;
  const float speed = 2.0f;

  float dx = target_msg.point.x - status_msg.point.x;
  if (fabs(dx) > threshold) {
    status_msg.point.x += (dx > 0) ? speed : -speed;
    return;
  }

  float dy = target_msg.point.y - status_msg.point.y;
  if (fabs(dy) > threshold) {
    status_msg.point.y += (dy > 0) ? speed : -speed;
    return;
  }

  gantry_state = IDLE;
}

void setup() {
  gantry_state = IDLE;

  // initialize status message frame_id
  status_msg.header.frame_id.data = (char *)"base_link";
  status_msg.header.frame_id.size = strlen(status_msg.header.frame_id.data);
  status_msg.header.frame_id.capacity = status_msg.header.frame_id.size + 1;

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &target_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "gantry/target"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "gantry/status"));

  // create status timer
  const unsigned int publish_frequency = 10;
  const unsigned int timer_timeout = 1000 / publish_frequency;
  RCCHECK(rclc_timer_init_default(
    &status_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publish_status));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &target_subscriber, &target_msg, &set_target, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  move_to_target();
}
