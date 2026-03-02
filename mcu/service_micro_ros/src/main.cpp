#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <gantry_interfaces/srv/object_positions.h>

#define SERVO_NT 18
#define SERVO_C 19
#define STEPX 33
#define DIRX 32
#define STEPY 27
#define DIRY 26
#define LIMX 16
#define LIMY 17
#define ENABLE 14

Servo servoc;
Servo servont;

const float stepsPerRevolution = 800;
const float max_x = 41.4;
const float max_y = 45.4;
const float min_arm_x = 8;
const float min_arm_y = max_y - 8;
const float max_arm_y = max_y - 8;
const float max_revolution_x = 7;
const float max_revolution_y = 7.275;
const float stepsPercm_x = (stepsPerRevolution * max_revolution_x) / max_x;
const float stepsPercm_y = (stepsPerRevolution * max_revolution_y) / max_y;
float position_x = 0;
float position_y = 0;

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

void home();
void capit(bool arah);
void naikturun(bool arah);
void mission(float x_load, float y_load, float x_drop, float y_drop);
void moveToTarget(float x, float y);

void error_loop() {
  while (1) {
    delay(100);
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
  RCCHECK(rclc_executor_add_service(
      &executor, &object_positions_service, &object_positions_request,
      &object_positions_response, object_positions_callback));

  pinMode(ENABLE, OUTPUT);
  pinMode(STEPX, OUTPUT);
  pinMode(DIRX, OUTPUT);
  pinMode(STEPY, OUTPUT);
  pinMode(DIRY, OUTPUT);
  pinMode(LIMX, INPUT_PULLUP);
  pinMode(LIMY, INPUT_PULLUP);
  pinMode(SERVO_C, OUTPUT);
  pinMode(SERVO_NT, OUTPUT);

  digitalWrite(ENABLE, LOW);

  servoc.attach(SERVO_C);
  servont.attach(SERVO_NT);
  servoc.write(80);
  servont.write(40);
  moveToTarget(min_arm_x, min_arm_y);
}

void loop() {
  while (!positions_received) {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }

  mission(object_positions_request.payload.x,
          object_positions_request.payload.y,
          object_positions_request.dropping_zone.x,
          object_positions_request.dropping_zone.y);
}

// ==== The code below is written with 💕 by Nofer ====

void mission(float x_load, float y_load, float x_drop, float y_drop) {
  moveToTarget(x_load, y_load);
  delayMicroseconds(500);
  naikturun(true);
  delayMicroseconds(500);
  capit(true);
  delayMicroseconds(500);
  naikturun(false);
  delayMicroseconds(500);
  moveToTarget(x_drop, y_drop);
  delayMicroseconds(500);
  naikturun(true);
  delayMicroseconds(500);
  capit(false);
  delayMicroseconds(500);
  naikturun(false);
  delayMicroseconds(500);
  moveToTarget(0, 0);
  delayMicroseconds(500);
}

void moveToTarget(float x, float y) {
  if (x > max_x) x = max_x;
  if (x < min_arm_x) x = min_arm_x;
  float selisih_X = x - position_x;
  float stepsX = abs(selisih_X) * stepsPercm_x;

  digitalWrite(DIRX, selisih_X > 0 ? HIGH : LOW);
  Serial.printf("stepsX: %f\n", stepsX);

  for (int i = 0; i < stepsX; i++) {
    digitalWrite(STEPX, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPX, LOW);
    delayMicroseconds(500);
  }

  position_x = x;

  if (y > max_arm_y) y = max_arm_y;
  if (y < min_arm_y) y = min_arm_y;
  float selisih_Y = y - position_y;
  float stepsY = abs(selisih_Y) * stepsPercm_y;

  digitalWrite(DIRY, selisih_Y > 0 ? LOW : HIGH);
  Serial.printf("stepsY: %f\n", stepsY);

  for (int i = 0; i < stepsY; i++) {
    digitalWrite(STEPY, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPY, LOW);
    delayMicroseconds(500);
  }

  position_y = y;
}

void home() {
  digitalWrite(DIRX, LOW);

  while (digitalRead(LIMX) != LOW) {
    digitalWrite(STEPX, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPX, LOW);
    delayMicroseconds(500);
  }

  digitalWrite(DIRY, HIGH);

  while (digitalRead(LIMY) != LOW) {
    digitalWrite(STEPY, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPY, LOW);
    delayMicroseconds(500);
  }

  position_x = 0;
  position_y = 0;
}

void capit(bool arah) {
  int buka = 80;
  int tutup = 140;

  if (arah) {
    for (int i = buka; i <= tutup; i++) {
      servoc.write(i);
      delay(5);
    }
    delay(10);
  } else {
    for (int i = tutup; i >= buka; i--) {
      servoc.write(i);
      delay(5);
    }
    delay(10);
  }
}

void naikturun(bool arah) {
  int naik = 140;
  int turun = 40;

  if (arah) {
    for (int i = turun; i < naik; i++) {
      servont.write(i);
      delay(5);
    }
    delay(10);
  } else {
    for (int b = naik; b > turun; b--) {
      servont.write(b);
      delay(5);
    }
    delay(10);
  }
}
