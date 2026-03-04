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

Servo servoc;
Servo servont;

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

void servo_rotate(Servo& servo, int start, int end) {
  if (start < end) {
    for (int i = start; i <= end; i++) {
      servo.write(i);
      delay(5);
    }
  } else {
    for (int i = start; i >= end; i--) {
      servo.write(i);
      delay(5);
    }
  }
}

void test_stepper() {
  digitalWrite(DIRX, HIGH);
  int stepsX = 0;

  while (digitalRead(LIMX) != LOW) {
    digitalWrite(STEPX, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPX, LOW);
    delayMicroseconds(500);
    stepsX++;
    Serial.printf("stepsX: %d\n", stepsX);
  }

  digitalWrite(DIRY, LOW);
  int stepsY = 0;

  while (digitalRead(LIMY) != LOW) {
    digitalWrite(STEPY, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPY, LOW);
    delayMicroseconds(500);
    stepsY++;
    Serial.printf("stepsY: %d\n", stepsY);
  }
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
void setup() {
  Serial.begin(115200);

  servoc.attach(SERVO_C);
  servont.attach(SERVO_NT);

  pinMode(ENABLE, OUTPUT);
  pinMode(STEPX, OUTPUT);
  pinMode(DIRX, OUTPUT);
  pinMode(STEPY, OUTPUT);
  pinMode(DIRY, OUTPUT);
  pinMode(LIMX, INPUT_PULLUP);
  pinMode(LIMY, INPUT_PULLUP);
  pinMode(SERVO_C, OUTPUT);
  pinMode(SERVO_NT, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.printf("Received command: %s\n", command);

    float start, end;
    if (command == "count") {
      Serial.printf("Testing stepper\n");
      test_stepper();
    } else if (sscanf(command.c_str(), "stepper:%f,%f", &start, &end) == 2) {
      Serial.printf("Moving stepper to %f,%f\n", start, end);
      moveToTarget(start, end);
    } else if (command == "buka") {
      Serial.printf("Opening cap\n");
      capit(true);
    } else if (command == "tutup") {
      Serial.printf("Closing cap\n");
      capit(false);
    } else if (sscanf(command.c_str(), "capit:%f,%f", &start, &end) == 2) {
      Serial.printf("Closing cap from %f to %f\n", start, end);
      servo_rotate(servoc, start, end);
    } else if (command == "naik") {
      Serial.printf("Moving arm up\n");
      naikturun(true);
    } else if (command == "turun") {
      Serial.printf("Moving arm down\n");
      naikturun(false);
    } else if (sscanf(command.c_str(), "z:%f,%f", &start, &end) == 2) {
      Serial.printf("Moving arm down from %f to %f\n", start, end);
      servo_rotate(servont, start, end);
    } else {
      Serial.printf("Unknown command: %s\n", command);
    }
  }
}
