#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <gantry_interfaces/msg/object_positions.h>

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

const float stepsPerRevolution = 1600;
const float max_x = 41.4;
const float max_y = 45.4;
const float min_arm_x = 8;
const float min_arm_y = 9;
const float max_arm_y = max_y - 9;
const float max_revolution_x = 7;
const float max_revolution_y = 7.275;
const float stepsPercm_x = (stepsPerRevolution * max_revolution_x) / max_x;
const float stepsPercm_y = (stepsPerRevolution * max_revolution_y) / max_y;
float position_x = 0;
float position_y = 9;

rcl_subscription_t object_positions_subscriber;
gantry_interfaces__msg__ObjectPositions object_positions_msg;
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

void object_positions_callback(const void* msgin) {
  RCL_UNUSED(msgin);
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

  RCCHECK(rclc_subscription_init_default(
      &object_positions_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(gantry_interfaces, msg, ObjectPositions),
      "/object_positions"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &object_positions_subscriber, &object_positions_msg,
      &object_positions_callback, ON_NEW_DATA));

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
  servoc.write(85);
  servont.write(30);
  // moveToTarget(min_arm_x, min_arm_y);
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

// ==== The code below is written with 💕 by Nofer ====

void mission(float x_load, float y_load, float x_drop, float y_drop) {
  moveToTarget(x_load, y_load);
  delay(3000);
  naikturun(true);
  delay(3000);
  capit(true);
  delay(3000);
  naikturun(false);
  delay(3000);
  moveToTarget(x_drop, y_drop);
  delay(3000);
  naikturun(true);
  delay(3000);
  capit(false);
  delay(3000);
  naikturun(false);
  delay(3000);
  moveToTarget(0, 9);
  delay(3000);
}

void moveToTarget(float x, float y) {
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
  int buka = 85;
  int tutup = 100;

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
