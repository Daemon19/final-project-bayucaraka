#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <gantry_interfaces/srv/object_positions.h>

rcl_service_t object_positions_service;
gantry_interfaces__srv__ObjectPositions_Request object_positions_request;
gantry_interfaces__srv__ObjectPositions_Response object_positions_response;
bool positions_received;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

void error_loop() {
  while (1) {
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void mission(float x_payload, float y_payload, float x_dropping_zone,
             float y_dropping_zone) {
  // TODO: Implement the mission logic here. For demonstration purposes, we will just blink the LED
  while (1) {
    delay(1000);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void object_positions_callback(const void* request_msg, void* response_msg) {
  RCL_UNUSED(request_msg);
  RCL_UNUSED(response_msg);
  positions_received = true;
}

void setup() {
  positions_received = false;

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_service_init_default(
      &object_positions_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(gantry_interfaces, srv, ObjectPositions),
      "/object_positions"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &object_positions_service, &object_positions_request, &object_positions_response,
                                    object_positions_callback));
}

void loop() {
  while (!positions_received) {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  mission(object_positions_request.payload.x, object_positions_request.payload.y, object_positions_request.dropping_zone.x,
          object_positions_request.dropping_zone.y);
}