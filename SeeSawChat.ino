#include <Wire.h>
#include <math.h>

/* ===========================
   Hardware Pins (Motor & Limits)
   =========================== */
#define IN3_PIN        5
#define IN4_PIN        4
#define SPEED_PIN      6
#define LEFT_LIMIT     3     // Active LOW (using INPUT_PULLUP)
#define RIGHT_LIMIT    2     // Active LOW (using INPUT_PULLUP)

/* ===========================
   Potentiometers
   =========================== */
#define AXLE_TRIMMER   A1    // Angle potentiometer (on platform axis)
#define SLIDE_TRIMMER  A2    // User target angle command

/* ===========================
   IMU (MPU6500 / 6050)
   =========================== */
#define MPU6500_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B

// Sensitivity scale factors (raw -> physical units)
#define ACCEL_SCALE 16384.0f  // LSB/g  (±2g)
#define GYRO_SCALE   131.0f   // LSB/(°/s) (±250 dps)

/* ===========================
   IMU calibration (means & variances in RAW units)
   accelOffsets[0..2] = mean (raw)
   accelOffsets[3..5] = variance (raw^2)
   gyroOffsets [0..2] = mean (raw)
   gyroOffsets [3..5] = variance (raw^2)
   =========================== */
float accelOffsets[6] = {0,0,0,0,0,0};
float gyroOffsets[6]  = {0,0,0,0,0,0};

/* ===========================
   Control loop timing
   =========================== */
static const uint16_t LOOP_HZ    = 100;
static const uint32_t LOOP_DT_US = 10000UL;

/* ===========================
   PID gains and limits
   =========================== */
float Kp = 6.0f;
float Ki = 20.0f;
float Kd = 0.45f;
int   MAX_PWM = 255;           // hard PWM clamp (0..255)
int   MIN_PWM = 85;            // deadband compensation (minimum to move)

/* ===== Small-adjustment stick–slip nudge ===== */
float SMALL_ERR_MIN_DEG   = 0.05f;  // ignore noise below this
float SMALL_ERR_MAX_DEG   = 2.0f;   // “small change” ceiling
float VEL_STUCK_DPS       = 0.30f;  // consider stuck if |dθ/dt| below this

// Micro-pulse characteristics (short “nudge” to break static friction)
int      NUDGE_PWM        = 25;     // must be >= MIN_PWM or we’ll max() it
uint16_t NUDGE_ON_MS      = 35;     // pulse ON duration
uint16_t NUDGE_OFF_MS     = 65;     // pause OFF (limits overshoot)

bool          nudge_active        = false;
bool          nudge_phase_on      = false;
unsigned long nudge_phase_until_ms= 0;

/* ===========================
   Kalman filter (theta, bias)
   =========================== */
float kf_theta = 0.0f;   // estimated angle (deg)
float kf_bias  = 0.0f;   // estimated gyro bias (deg/s)
float P00=1, P01=0, P10=0, P11=1;  // covariance
// Process noise (tune):
float Q_angle = 0.02f;   // angle random walk (deg^2 / s)
float Q_bias  = 0.005f;  // bias random walk  (deg^2 / s)

// Measurement noise (deg^2) — accelerometer and potentiometer
float R_accel_base = 4.0f;    // ~ (2 deg)^2 base; dynamic scaling with | |a| - 1g |
float R_pot        = 1.0f;    // set after calibration from endpoint jitter

/* ===========================
   Calibration results
   =========================== */
float adc_left_mean  = 0.0f, adc_right_mean = 1023.0f; // AXLE_TRIMMER ADC means at limits
float adc_left_var   = 0.0f, adc_right_var  = 0.0f;    // variance of ADC counts at limits
float theta_left_deg = 0.0f, theta_right_deg = 0.0f;   // IMU pitch (deg) at limits

// Linear map from AXLE_TRIMMER ADC -> degrees: theta_deg = m*adc + b
float pot_m_deg_per_adc = 0.0f;
float pot_b_deg         = 0.0f;

// Boundaries for commanded angle (deg)
float theta_min_deg = 0.0f;
float theta_max_deg = 0.0f;

/* ===========================
   PID state
   =========================== */
float pid_integral = 0.0f;
float last_theta   = 0.0f;

/* ===========================
   Forward Declarations
   =========================== */
// Motor helpers
void move_right(int sp);
void move_left(int sp);
void motor_stop();
void motor_home_and_calibrate();

// IMU helpers
void imuWake();
void imuBurstRead(uint8_t startReg, uint8_t* buf14);
void readRawAccelGyro(int16_t accel[3], int16_t gyro[3]);
void calibrateMPU6500(uint16_t samples = 300, bool populationVariance = true);

// Angle helpers
static inline float toAccelG(int16_t raw, float meanRaw)   { return (raw - meanRaw) / ACCEL_SCALE; }
static inline float toGyroDps(int16_t raw, float meanRaw)  { return (raw - meanRaw) / GYRO_SCALE; }
static inline float pitchDegFromAccel(float ay_g, float az_g) { return atan2f(ay_g, az_g) * 180.0f / M_PI; }
float thetaFromPotADC(int adc);

// Kalman helpers
void kalmanPredict(float gyro_dps, float dt);
void kalmanUpdate(float z_deg, float R);

// Calibration steps
void collectAtLimit(bool left, uint16_t N, float* adc_mean, float* adc_var, float* theta_mean_deg);
void computePotMapAndNoise();
bool atLeftLimit();
bool atRightLimit();

// Control
int  computeMotorPWM(float target_deg, float theta_deg, float dt);
int  smallErrorNudgePWM(float err_deg, float dtheta_dps);
void applyMotorCommand(int pwm);
float mapSlideToTargetDeg(int slide_adc);

// Utility
static inline int clampInt(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float clampF(float v, float lo, float hi){ return (v < lo) ? lo : (v > hi) ? hi : v; }

/* ===========================
   Setup
   =========================== */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(SPEED_PIN, OUTPUT);
  pinMode(LEFT_LIMIT,  INPUT_PULLUP);
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);
  pinMode(AXLE_TRIMMER,  INPUT);
  pinMode(SLIDE_TRIMMER, INPUT);

  motor_stop();

  imuWake();
  delay(150);
  Serial.println(F("IMU calibration (Welford): keep device still..."));
  calibrateMPU6500();
  Serial.println(F("IMU calibration complete."));

  motor_home_and_calibrate();

  // Initialize KF near current measured angle
  int16_t a[3], g[3];
  readRawAccelGyro(a, g);
  float ay = toAccelG(a[1], accelOffsets[1]);
  float az = toAccelG(a[2], accelOffsets[2]);
  float theta_acc = pitchDegFromAccel(ay, az);
  float theta_pot = thetaFromPotADC(analogRead(AXLE_TRIMMER));
  kf_theta = 0.5f * (theta_acc + theta_pot);
  kf_bias  = 0.0f;
  P00 = 10; P01 = 0; P10 = 0; P11 = 10;
}

/* ===========================
   Loop: 100 Hz control
   =========================== */
void loop() {
  static unsigned long last_us = micros();
  unsigned long now_us = micros();
  if (now_us - last_us < LOOP_DT_US) return;
  float dt = (now_us - last_us) / 1e6f;
  last_us = now_us;

  // ===== Read sensors =====
  int16_t rawA[3], rawG[3];
  readRawAccelGyro(rawA, rawG);

  float gx_dps = toGyroDps(rawG[0], gyroOffsets[0]);
  float gy_dps = toGyroDps(rawG[1], gyroOffsets[1]);
  float gz_dps = toGyroDps(rawG[2], gyroOffsets[2]);
  (void)gy_dps; (void)gz_dps; // pitch control uses X gyro

  float ax_g = toAccelG(rawA[0], accelOffsets[0]);
  float ay_g = toAccelG(rawA[1], accelOffsets[1]);
  float az_g = toAccelG(rawA[2], accelOffsets[2]);
  float theta_acc = pitchDegFromAccel(ay_g, az_g);

  int   adc_axle = analogRead(AXLE_TRIMMER);
  float theta_pot = thetaFromPotADC(adc_axle);

  // Adaptive accel measurement noise
  float acc_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g); // in g
  float motion_factor = 1.0f + 15.0f * fabsf(acc_mag - 1.0f);
  float R_accel = R_accel_base * motion_factor;

  // ===== Kalman filter =====
  kalmanPredict(gx_dps, dt);
  kalmanUpdate(theta_acc, R_accel);
  kalmanUpdate(theta_pot, R_pot);

  // ===== Target angle from slide pot =====
  int   adc_slide = analogRead(SLIDE_TRIMMER);
  float theta_cmd = mapSlideToTargetDeg(adc_slide);
  theta_cmd = clampF(theta_cmd, theta_min_deg, theta_max_deg);

  // ---- Telemetry (including desired angle, per your request) ----
  Serial.print(F(" target: ")); Serial.print(theta_cmd);
  Serial.print(F(" | theta: ")); Serial.print(kf_theta);
  Serial.print(F(" | acc_mag(g): ")); Serial.print(acc_mag);
  Serial.print(F(" | Racc: ")); Serial.print(R_accel);
  Serial.print(F(" | Rpot: ")); Serial.println(R_pot);

  // ===== Motor control =====
  int pwm = computeMotorPWM(theta_cmd, kf_theta, dt);
  applyMotorCommand(pwm);
}

/* ===========================
   Motor helpers
   =========================== */
void move_right(int sp) {
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(SPEED_PIN, sp);
}

void move_left(int sp) {
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(SPEED_PIN, sp);
}

void motor_stop() {
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(SPEED_PIN, 0);
}

bool atLeftLimit()  { return digitalRead(LEFT_LIMIT)  == LOW; }
bool atRightLimit() { return digitalRead(RIGHT_LIMIT) == LOW; }

/* ===========================
   IMU helpers
   =========================== */
void imuWake() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
}

// Burst-read 14 bytes (Accel XYZ, Temp, Gyro XYZ) starting at ACCEL_XOUT_H
void imuBurstRead(uint8_t startReg, uint8_t* buf14) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);     // repeated start
  Wire.requestFrom(MPU6500_ADDR, (uint8_t)14);
  for (uint8_t i = 0; i < 14 && Wire.available(); ++i)
    buf14[i] = Wire.read();
}

void readRawAccelGyro(int16_t accel[3], int16_t gyro[3]) {
  uint8_t b[14];
  imuBurstRead(ACCEL_XOUT_H, b);
  // [AxH AxL AyH AyL AzH AzL TempH TempL GxH GxL GyH GyL GzH GzL]
  accel[0] = (int16_t)((b[0] << 8) | b[1]);
  accel[1] = (int16_t)((b[2] << 8) | b[3]);
  accel[2] = (int16_t)((b[4] << 8) | b[5]);
  gyro[0]  = (int16_t)((b[8] << 8) | b[9]);
  gyro[1]  = (int16_t)((b[10] << 8) | b[11]);
  gyro[2]  = (int16_t)((b[12] << 8) | b[13]);
}

/*
  IMU mean/variance via Welford’s algorithm (single pass, numerically stable)
  Means/variances are in RAW sensor units.
*/
void calibrateMPU6500(uint16_t samples, bool populationVariance) {
  float meanA[3] = {0,0,0}, m2A[3] = {0,0,0};
  float meanG[3] = {0,0,0}, m2G[3] = {0,0,0};

  for (uint16_t n = 1; n <= samples; ++n) {
    int16_t a[3], g[3];
    readRawAccelGyro(a, g);

    for (int j = 0; j < 3; ++j) {
      // Accel
      float xA = (float)a[j];
      float dA = xA - meanA[j];
      meanA[j] += dA / n;
      float d2A = xA - meanA[j];
      m2A[j] += dA * d2A;

      // Gyro
      float xG = (float)g[j];
      float dG = xG - meanG[j];
      meanG[j] += dG / n;
      float d2G = xG - meanG[j];
      m2G[j] += dG * d2G;
    }
    delay(4);
  }

  const float denom = populationVariance ? (float)samples : (float)(samples - 1);
  for (int j = 0; j < 3; ++j) {
    accelOffsets[j]     = meanA[j];
    accelOffsets[j + 3] = (samples > 1) ? (m2A[j] / denom) : 0.0f;
    gyroOffsets[j]      = meanG[j];
    gyroOffsets[j + 3]  = (samples > 1) ? (m2G[j] / denom) : 0.0f;
  }

  // Optional: upright zero-g on Z
  accelOffsets[2] -= ACCEL_SCALE;
}

/* ===========================
   Calibration sweep
   =========================== */
void motor_home_and_calibrate() {
  Serial.println(F("Homing to left limit..."));
  move_left(140);
  while (!atLeftLimit()) { /* wait */ }
  motor_stop();
  delay(150);

  // Collect stats at left limit
  collectAtLimit(true,  150, &adc_left_mean, &adc_left_var, &theta_left_deg);
  Serial.print(F("Left limit:  ADC="));   Serial.print(adc_left_mean);
  Serial.print(F("  theta="));            Serial.print(theta_left_deg);
  Serial.print(F(" deg  varADC="));       Serial.println(adc_left_var);

  Serial.println(F("Sweeping to right limit..."));
  move_right(140);
  while (!atRightLimit()) { /* wait */ }
  motor_stop();
  delay(150);

  // Collect stats at right limit
  collectAtLimit(false, 150, &adc_right_mean, &adc_right_var, &theta_right_deg);
  Serial.print(F("Right limit: ADC="));   Serial.print(adc_right_mean);
  Serial.print(F("  theta="));            Serial.print(theta_right_deg);
  Serial.print(F(" deg  varADC="));       Serial.println(adc_right_var);

  // Build linear map AXLE_TRIMMER ADC -> degrees and compute R_pot
  computePotMapAndNoise();

  // Save convenience min/max
  theta_min_deg = fminf(theta_left_deg,  theta_right_deg);
  theta_max_deg = fmaxf(theta_left_deg,  theta_right_deg);

  Serial.print(F("Pot map: theta = ")); Serial.print(pot_m_deg_per_adc, 6);
  Serial.print(F(" * ADC + "));         Serial.println(pot_b_deg, 3);
  Serial.print(F("Theta range (deg): [")); Serial.print(theta_min_deg);
  Serial.print(F(", "));                   Serial.print(theta_max_deg);
  Serial.println(F("]"));
  Serial.print(F("R_pot (deg^2): "));      Serial.println(R_pot, 4);

  // Back off a little from hard stops for safety
  move_left(120);
  delay(250);
  motor_stop();
}

void collectAtLimit(bool left, uint16_t N, float* adc_mean, float* adc_var, float* theta_mean_deg) {
  (void)left; // not used; kept for clarity/extension
  float mean_adc = 0.0f, m2_adc = 0.0f;
  float mean_theta = 0.0f, m2_theta = 0.0f;

  for (uint16_t i = 1; i <= N; ++i) {
    int adc = analogRead(AXLE_TRIMMER);
    float delta = adc - mean_adc;
    mean_adc += delta / i;
    m2_adc   += delta * (adc - mean_adc);

    int16_t a[3], g[3];
    readRawAccelGyro(a, g);

    float ay = toAccelG(a[1], accelOffsets[1]);
    float az = toAccelG(a[2], accelOffsets[2]);
    float theta = pitchDegFromAccel(ay, az);

    float d2 = theta - mean_theta;
    mean_theta += d2 / i;
    m2_theta   += d2 * (theta - mean_theta);

    delay(4);
  }

  *adc_mean       = mean_adc;
  *adc_var        = (N > 1) ? (m2_adc / (float)(N - 1)) : 0.0f;
  *theta_mean_deg = mean_theta;
  (void)m2_theta; // hook for future adaptive R based on endpoint stability
}

void computePotMapAndNoise() {
  // Linear mapping using IMU-measured angles at the two ends
  float d_adc   = adc_right_mean - adc_left_mean;
  float d_theta = theta_right_deg - theta_left_deg;
  if (fabsf(d_adc) < 1e-3f) {
    pot_m_deg_per_adc = 0.0f;
    pot_b_deg         = theta_left_deg; // fallback (deg)
  } else {
    pot_m_deg_per_adc = d_theta / d_adc;
    pot_b_deg         = theta_left_deg - pot_m_deg_per_adc * adc_left_mean;
  }

  // Pot measurement noise (deg^2) from endpoint jitter (average of both ends)
  float mean_var_adc = 0.5f * (adc_left_var + adc_right_var);
  R_pot = mean_var_adc * pot_m_deg_per_adc * pot_m_deg_per_adc;
}

/* ===========================
   Angle helpers
   =========================== */
float thetaFromPotADC(int adc) {
  return pot_m_deg_per_adc * adc + pot_b_deg;
}

/* ===========================
   Kalman filter (1D angle + gyro bias)
   =========================== */
void kalmanPredict(float gyro_dps, float dt) {
  // x_k|k-1 = [theta + dt*(gyro - bias); bias]
  kf_theta += dt * (gyro_dps - kf_bias);
  // F = [[1, -dt],[0,1]]; add process noise scaled by dt
  P00 += dt * (dt*P11 - P01 - P10) + Q_angle * dt;
  P01 += -dt * P11;
  P10 += -dt * P11;
  P11 += Q_bias * dt;
}

void kalmanUpdate(float z_deg, float R) {
  // H = [1, 0]
  float y  = z_deg - kf_theta;      // innovation
  float S  = P00 + R;               // innovation covariance
  float K0 = P00 / S;               // Kalman gain for theta
  float K1 = P10 / S;               // Kalman gain for bias

  kf_theta += K0 * y;
  kf_bias  += K1 * y;

  // (I - K H) P
  float P00_old = P00;
  float P01_old = P01;
  P00 -= K0 * P00_old;
  P01 -= K0 * P01_old;
  P10 -= K1 * P00_old;
  P11 -= K1 * P01_old;
}

/* ===========================
   Control (PID on angle) with small-error nudge
   =========================== */
int smallErrorNudgePWM(float err_deg, float dtheta_dps) {
  unsigned long now = millis();

  if (fabsf(err_deg) >= SMALL_ERR_MIN_DEG &&
      fabsf(err_deg) <= SMALL_ERR_MAX_DEG &&
      fabsf(dtheta_dps) <= VEL_STUCK_DPS) {

    if (!nudge_active) {
      nudge_active = true;
      nudge_phase_on = true;
      nudge_phase_until_ms = now + NUDGE_ON_MS;
    }

    if (nudge_phase_on) {
      if (now >= nudge_phase_until_ms) {
        nudge_phase_on = false;
        nudge_phase_until_ms = now + NUDGE_OFF_MS;
        return 0;
      }
      int pwm = max(NUDGE_PWM, MIN_PWM);
      return (err_deg > 0 ? +pwm : -pwm);
    } else {
      if (now >= nudge_phase_until_ms) {
        nudge_phase_on = true;
        nudge_phase_until_ms = now + NUDGE_ON_MS;
      }
      return 0;
    }
  }

  // Exit nudge if conditions not met
  nudge_active = false;
  return 0;
}

int computeMotorPWM(float target_deg, float theta_deg, float dt) {
  // --- PID core ---
  float err = target_deg - theta_deg;

  // Integral with modest clamp to prevent slow drift/creep.
  static const float INTEGRAL_CLAMP = 200.0f;
  pid_integral += err * dt;
  pid_integral = clampF(pid_integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

  float dtheta = (theta_deg - last_theta) / dt; // derivative of MEASUREMENT
  last_theta = theta_deg;

  float u = Kp*err + Ki*pid_integral - Kd*dtheta;

  // --- Convert to PWM and deadband handling ---
  int pwm = (int)roundf(u);
  pwm = clampInt(pwm, -MAX_PWM, MAX_PWM);

  // If PWM is below deadband, try small-error nudge before giving up:
  if (abs(pwm) < MIN_PWM) {
    int nudge = smallErrorNudgePWM(err, dtheta);
    if (nudge != 0) {
      if ((nudge < 0 && atLeftLimit()) || (nudge > 0 && atRightLimit())) return 0;
      return nudge;
    }
  }

  // Normal deadband compensation outside of nudge path
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  if (pwm < 0 && -pwm < MIN_PWM) pwm = -MIN_PWM;

  // Respect hard limits
  if (atLeftLimit()  && pwm < 0) pwm = 0;
  if (atRightLimit() && pwm > 0) pwm = 0;

  return pwm;
}

void applyMotorCommand(int pwm) {
  if (pwm == 0) { motor_stop(); return; }
  if (pwm > 0)   move_right(pwm);
  else           move_left(-pwm);
}

/* ===========================
   Slide pot → target angle
   =========================== */
float mapSlideToTargetDeg(int slide_adc) {
  // Map 0..1023 to [theta_min_deg, theta_max_deg]
  float t = (float)slide_adc / 1023.0f;
  return theta_min_deg + t * (theta_max_deg - theta_min_deg);
}
