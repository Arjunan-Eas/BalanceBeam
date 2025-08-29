#include <Wire.h>

// Motor
#define in3   5
#define in4   4
#define speed 6

// Limits
#define left_limit  3
#define right_limit 2

// Potentiometers
#define axle_trimmer  A1
#define slide_trimmer A2

// IMU
#define mpu_address 0x68
const uint8_t ACCEL_ADR = 0x3B;
const uint8_t GYRO_ADR = 0x43;

void setup() {
  // Motor
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(speed, OUTPUT);

  // Limits
  pinMode(left_limit, INPUT_PULLUP);
  pinMode(right_limit, INPUT_PULLUP);

  // Potentiometers
  pinMode(axle_trimmer, INPUT);
  pinMode(slide_trimmer, INPUT);

  // Serial output
  Serial.begin(115200);

  // I2C
  Wire.begin(8);

  // Motor initialization
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(speed, 0);
  motor_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  float theta_goal = map(analogRead(slide_trimmer), 0, 1023, -3000, 3000) / 100.0;
  float theta_trimmer = map(analogRead(slide_trimmer), 384, 615, -300, 300) / 10.0;
  Serial.println(theta_trimmer);
}

void move_right(int sp) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(speed, sp);
}

void move_left(int sp) {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(speed, sp);
}

void stop() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(speed, 0);
}

void motor_setup() {
  move_left(125);
  while(digitalRead(left_limit));
  stop();
  move_right(125);
  while(digitalRead(right_limit));
  stop();
  float theta_trimmer = map(analogRead(slide_trimmer), 384, 615, -300, 300) / 10.0;
  move_left(125);
  while(theta_trimmer > 0);
  stop();

}
