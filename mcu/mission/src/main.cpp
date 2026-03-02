#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_NT 18
#define SERVO_C 19
#define STEPX 33
#define DIRX 32
#define STEPY 27
#define DIRY 26
#define LIMX 16
#define LIMY 17
#define ENABLE 14

void mission(float x_load, float y_load, float x_drop, float y_drop);
void moveToTarget(float x, float y);
void test();
void home();
void capit(bool arah);
void naikturun(bool arah);

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

void setup() {
  Serial.begin(115200);
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
  naikturun(true);
  delayMicroseconds(500);
  capit(true);
  delayMicroseconds(500);
  naikturun(false);
  delayMicroseconds(500);
  capit(false);
  delayMicroseconds(500);
}

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

// void test(){
//   digitalWrite(DIRX, HIGH);
//   int stepsX = 0;

//   while(digitalRead(LIMX) != LOW){
//     digitalWrite(STEPX, HIGH);
//     delayMicroseconds(500);
//     digitalWrite(STEPX, LOW);
//     delayMicroseconds(500);
//     stepsX++;
//     Serial.printf("stepsX: %d\n", stepsX);
//   }

//   digitalWrite(DIRY, LOW);
//   int stepsY = 0;

//   while(digitalRead(LIMY) != LOW){
//     digitalWrite(STEPY, HIGH);
//     delayMicroseconds(500);
//     digitalWrite(STEPY, LOW);
//     delayMicroseconds(500);
//     stepsY++;
//     Serial.printf("stepsY: %d\n", stepsY);
//   }
// }

// void home(){
//   digitalWrite(DIRX, LOW);

//   while(digitalRead(LIMX) != LOW){
//     digitalWrite(STEPX, HIGH);
//     delayMicroseconds(500);
//     digitalWrite(STEPX, LOW);
//     delayMicroseconds(500);
//   }

//   digitalWrite(DIRY, HIGH);

//   while(digitalRead(LIMY) != LOW){
//     digitalWrite(STEPY, HIGH);
//     delayMicroseconds(500);
//     digitalWrite(STEPY, LOW);
//     delayMicroseconds(500);
//   }

//   position_x = 0;
//   position_y = 0;
// }

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
