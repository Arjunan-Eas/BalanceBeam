#include <Wire.h>
#include <math.h>

#define MPU6500_ADDR 0x68

// Register addresses
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B

// Sensitivity scale factors
#define ACCEL_SCALE 16384.0  // For ±2g
#define GYRO_SCALE  131.0    // For ±250°/s

// Calibration offsets
float accelOffsets[3] = {0};
float gyroOffsets[3]  = {0};

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6500
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);

  Wire.endTransmission();
  delay(100);

  Serial.println("Calibrating... Keep device still");
  calibrateMPU6500();
  Serial.println("Calibration complete.");
}

void loop() {
  int16_t rawAccel[3], rawGyro[3];
  readRawAccelGyro(rawAccel, rawGyro);

  // Apply calibration and scale
  float ax = (rawAccel[0] - accelOffsets[0]) / ACCEL_SCALE;
  float ay = (rawAccel[1] - accelOffsets[1]) / ACCEL_SCALE;
  float az = (rawAccel[2] - accelOffsets[2]) / ACCEL_SCALE;

  float gx = (rawGyro[0] - gyroOffsets[0]) / GYRO_SCALE;
  float gy = (rawGyro[1] - gyroOffsets[1]) / GYRO_SCALE;
  float gz = (rawGyro[2] - gyroOffsets[2]) / GYRO_SCALE;
  
  // Calculates pitch angle
  float theta = atan(ay/az)*180/M_PI;
  Serial.print("Pitch Angle: ");
  Serial.print(theta);

  // Acceleration magnitude
  float acceleration = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  Serial.print("°   Acc mag: ");
  Serial.print(acceleration);

  // On-axis acceleration %
  float acc_on_axis = sqrt(pow(ay, 2) + pow(az, 2)) / acceleration * 100;
  Serial.print("g   On-axis: ");
  Serial.print(acc_on_axis);
  Serial.println("%");
  

  delay(100);
}

void readRawAccelGyro(int16_t* accel, int16_t* gyro) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14);

  if (Wire.available() == 14) {
    accel[0] = (Wire.read() << 8) | Wire.read();  // Accel X
    accel[1] = (Wire.read() << 8) | Wire.read();  // Accel Y
    accel[2] = (Wire.read() << 8) | Wire.read();  // Accel Z
    Wire.read(); Wire.read();  // Skip temperature
    gyro[0]  = (Wire.read() << 8) | Wire.read();  // Gyro X
    gyro[1]  = (Wire.read() << 8) | Wire.read();  // Gyro Y
    gyro[2]  = (Wire.read() << 8) | Wire.read();  // Gyro Z
  }
}

void calibrateMPU6500() {
  const int samples = 200;
  long accelSum[3] = {0};
  long gyroSum[3]  = {0};

  for (int i = 0; i < samples; i++) {
    int16_t a[3], g[3];
    readRawAccelGyro(a, g);
    for (int j = 0; j < 3; j++) {
      accelSum[j] += a[j];
      gyroSum[j]  += g[j];
    }
    delay(5);
  }

  for (int i = 0; i < 3; i++) {
    accelOffsets[i] = accelSum[i] / (float)samples;
    gyroOffsets[i]  = gyroSum[i] / (float)samples;
  }

  // Optional: For Z axis, subtract 1g from accel offset (assuming device is upright)
  accelOffsets[2] -= ACCEL_SCALE;
}
