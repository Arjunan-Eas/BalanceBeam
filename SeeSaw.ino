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
double accelOffsets[6] = {0,0,0,0,0,0};
double gyroOffsets[6]  = {0,0,0,0,0,0};

/* Min/Max constants */
const int MAX_PWM = 255;
const int MIN_PWM = 80;
double MAX_ANGLE = 0.0;
double MIN_ANGLE = 0.0;
int MAX_POT = 0;
int MIN_POT = 0;

/* Kalman */
double cov;    // Accelerometer and gyro covariance
double pot_var;
double theta_kalman;
unsigned long loop_start;
int16_t acc[3] = {0, 0, 0};
int16_t gyro[3] = {0, 0, 0};

/* PID */
// Gains (units): Kp [PWM/deg], Kd [PWM/(deg/s)], Ki [PWM/(deg*s)]
float Kp = 7.0f;
float Kd = 0.7f;      // positive, since we subtract the rate term below
float Ki = 10.0f;      // start small; see tuning notes

// Derivative filter (seconds)
double D_tau = 0.05;   // 50 ms low-pass on gyro rate

// Integral state and limits
double I_state = 0.0;          // integrates error
double I_leak_tau = 0;       // seconds; set 0 to disable leak
double I_max_pwm = 255.0;      // clamp Ki*I_state within ±I_max_pwm

double theta_goal;
int motor_speed;
long prev_angle;
double dt;
const long MAX_DERIVATIVE = 6000;
long max_der = 0;
long min_der = 0;

String inputString = "";

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

    stop_motor();

    // Initialize IMU
    imuWake();
    delay(150);
    calibrateMPU6500(15);

    // Calibrate motor offsets
    motor_calibrate(100);
}

void loop() {
    // Perform angle update
    loop_start = micros();
    dt = kalman_update(loop_start, acc, gyro);
    theta_goal = map(analogRead(SLIDE_TRIMMER), 0, 1023, MIN_ANGLE*100, MAX_ANGLE*100) / 100.0;
    pid(dt, gyro[0]);
    driveMotor(motor_speed);
    Serial.print("Goal angle: ");
    Serial.print(theta_goal);
    Serial.print(" deg | Current angle: ");
    Serial.print(theta_kalman);
    Serial.print(" deg | Motor speed: ");
    Serial.println(motor_speed);

    if (Serial.available()) {
        char inChar = (char)Serial.read();
        
        // End of line marks full command
        if (inChar == '\n' || inChar == '\r') {
        if (inputString.length() > 0) {
            parseCommand(inputString);
            inputString = ""; // reset
        }
        } else {
        inputString += inChar;
        }
    }

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

void stop_motor() {
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
  Means/variances are in RAW sensor units.
*/
void calibrateMPU6500(uint16_t samples) {
    long acc_data[samples][3];
    long acc_sum[3];
    int16_t a[3] = {0, 0, 0};
    
    long gyro_data[samples][3];
    long gyro_sum[3];
    int16_t g[3] = {0, 0, 0};
    int next = 100;
    // Take samples and record data
    for(int i = 0; i < samples; i++) {
        driveMotor(next);
        readRawAccelGyro(a, g);
        for(int j = 0; j < 3; j++) {
            acc_data[i][j] = a[j];
            acc_sum[j] += a[j];

            gyro_data[i][j] = g[j];
            gyro_sum[j] += g[j];
        }
        next = -1 * next;
        delay(100);
    }

    // Compute means
    for(int i = 0; i < 3; i++) {
        accelOffsets[i] = acc_sum[i] / samples;
        gyroOffsets[i] = gyro_sum[i] / samples;
    }

    long acc_var_sum[3];
    long gyro_var_sum[3];

    // Compute variances (raw^2)
    for(int i = 0; i < samples; i++) {
        for(int j = 0; j < 3; j++) {
            acc_var_sum[j] = pow((acc_data[i][j] - accelOffsets[j]), 2);
            gyro_var_sum[j] = pow((gyro_data[i][j] - gyroOffsets[j]), 2);
        }
    }

    for(int i = 0; i < 3; i++) {
        accelOffsets[i+3] = acc_var_sum[i] / samples;
        gyroOffsets[i+3] = gyro_var_sum[i] / samples;
    }
}

int pwm_valid(int pwm) {
    if(abs(pwm) > MIN_PWM && abs(pwm) < MAX_PWM) { return 0; }
    else {
        if(pwm > MAX_PWM) { return MAX_PWM; }
        else if(pwm < (-1*MAX_PWM)) { return -1*MAX_PWM; }
        else if(pwm > 0 && pwm < MIN_PWM) { return MIN_PWM; }
        else if(pwm < 0 && pwm > (-1*MIN_PWM)) { return -1*MIN_PWM; }
        else if(pwm == 0) { return 0; }
    }
}

void driveMotor(int pwm) {
    if (pwm == 0) { stop_motor(); return; }
    if (pwm > 0 && (pwm_valid(pwm) == 0) && !atRightLimit()) { move_right(pwm); }
    else if((pwm_valid(pwm) == 0) && !atLeftLimit()) { move_left(-pwm); } 
}

double angleAccel(int16_t ay, int16_t az) {
    return atan2f(ay, az) * 180.0f / M_PI;
}

double angleGyro(int16_t gx, double theta_init, double dt) {
    return theta_init + (gx / GYRO_SCALE) * dt;
}

double measureAnglePot() {
    int raw = analogRead(AXLE_TRIMMER);
    return map(raw, MIN_POT, MAX_POT, (long)(MIN_ANGLE*100), (long)(MAX_ANGLE*100)) / 100.0f;
}

void motor_calibrate(uint16_t samples) {
    Serial.println("Going left");
    driveMotor(-150);
    while(!atLeftLimit());
    stop_motor();
    delay(500);
    int16_t a[3] = {0, 0, 0};
    int16_t g[3] = {0, 0, 0};

    double angle = 0;
    long pot = 0;
    for(int i = 0; i < samples; i++) {
        readRawAccelGyro(a,g);
        angle += angleAccel(a[1], a[2]);
        pot += analogRead(AXLE_TRIMMER);
        delay(10);
    }
    MIN_ANGLE = angle / samples;
    MIN_POT = int((double)pot / (double)samples);
    Serial.print("Minimum angle: ");
    Serial.print(MIN_ANGLE);
    Serial.println(" deg");
    delay(500);

    Serial.println("Going right");
    driveMotor(150);
    while(!atRightLimit());
    stop_motor();
    delay(500);

    angle = 0;
    pot = 0;
    for(int i = 0; i < samples; i++) {
        readRawAccelGyro(a,g);
        angle += angleAccel(a[1], a[2]);
        pot += analogRead(AXLE_TRIMMER);
        delay(10);
    }
    MAX_ANGLE = angle / samples;
    MAX_POT = int((double)pot / (double)samples);
    Serial.print("Maximum angle: ");
    Serial.print(MAX_ANGLE);
    Serial.println(" deg");
    delay(500);

    // Calculate covariance of acc and gyro
    theta_kalman = MAX_ANGLE; // Initialize to max angle since known
    covariance_a_g();
}

void covariance_a_g() {
    int16_t a[3] = {0, 0, 0};
    int16_t g[3] = {0, 0, 0};
    double a_angle[10];
    double g_angle[11];
    g_angle[0] = MAX_ANGLE;
    unsigned long t0;
    unsigned long t1;

    // Take 10 measurements to determine covariance
    for(int i = 0; i < 10; i++) {
        t0 = micros();
        readRawAccelGyro(a, g);
        t1 = micros();
        a_angle[i] = angleAccel(a[1], a[2]);
        g_angle[i+1] = angleGyro(g[0], g_angle[i], (t1-t0)/1.0e6);
    }

    double a_mean = 0.0;
    double g_mean = 0.0;

    for(int i = 0; i < 10; i++) {
        a_mean += a_angle[i];
        g_mean += g_angle[i+1];
    }

    a_mean = a_mean / 10;
    g_mean = g_mean / 10;

    for(int i = 0; i < 10; i++) {
        cov += (a_angle[i] - a_mean) * (g_angle[i+1] - g_mean);
    }
    
    cov = cov / 10;
    delay(1000);
    pot_variance();
}

void pot_variance() {
    int16_t a[3] = {0, 0, 0};
    int16_t g[3] = {0, 0, 0};
    double pot_angle[10];

    // Take 10 measurements to determine covariance
    for(int i = 0; i < 10; i++) {
        pot_angle[i] = measureAnglePot();
        driveMotor(-150);
        delay(100);
        driveMotor(150);
        while(!atRightLimit());
        stop_motor();
    }

    double pot_mean = 0.0;

    for(int i = 0; i < 10; i++) {
        pot_mean += pot_angle[i];
    }

    pot_mean = pot_mean / 10;

    for(int i = 0; i < 10; i++) {
        pot_var += pow((pot_angle[i] - pot_mean), 2);
    }
    
    pot_var = pot_var / 10;
    prev_angle = MAX_ANGLE;
}

double kalman_update(unsigned long t0, int16_t* a, int16_t* g) {
    readRawAccelGyro(a, g);
    double dt = (micros() - t0)/1.0e6;
    // Predict
    theta_kalman = angleGyro(g[0], theta_kalman, dt);
    // Update with prediction uncertainty
    cov = cov + pow(dt, 2) * pow(gyroOffsets[3] / (GYRO_SCALE*GYRO_SCALE), 2);
    // Measure
    double theta_meas = angleAccel(a[1], a[2]);
    // Kalman gain of measurement
    double kalman_gain = cov / (cov + (accelOffsets[4] + accelOffsets[5])/(2*pow(ACCEL_SCALE, 2)));
    // Update with measurement uncertainty
    cov = (1 - kalman_gain)*cov;
    // Intermediate output
    theta_kalman = theta_kalman + kalman_gain * (theta_meas - theta_kalman);
    // Measure with other sensor
    theta_meas = measureAnglePot();
    // Kalman gain of second measurement
    kalman_gain = cov / (cov + pot_var);
    // Update with second measurement uncertainty
    cov = (1 - kalman_gain)*cov;
    // Final output
    theta_kalman = theta_kalman + kalman_gain * (theta_meas - theta_kalman);
    
    return dt;
}

void pid(double dt, int16_t gx) {
  // 1) Error: setpoint - measurement
  double e = theta_goal - theta_kalman;

  // 2) Derivative from gyro (deg/s), low-pass filtered
  static double theta_dot_f = 0.0;
  double theta_dot = (double)gx / GYRO_SCALE;      // deg/s
  double alpha = dt / (D_tau + dt);
  theta_dot_f += alpha * (theta_dot - theta_dot_f);

  // 3) Leaky integrator (bleed toward 0 to avoid long-term windup)
  if (I_leak_tau > 0.0) {
    double leak = dt / I_leak_tau;
    if (leak > 1.0) leak = 1.0;
    I_state *= (1.0 - leak);
  }

  // ---------- Exponential boost for persistent small error ----------
  // Counts time only while |e| is small (stuck near target) and the sign is stable.
  static double persist_t = 0.0;
  static double e_prev = 0.0;
  const double E_PERSIST_MIN = 0.15;  // deg: ignore pure noise below this
  const double E_PERSIST_MAX = 6.0;   // deg: don't "boost" when far away
  const double I_BOOST_RATE = 7;    // 1/s: growth rate for exp() (tune 0.3–1.0)
  const double I_BOOST_MAX  = 50;   // cap on the boost factor (tune 5–20)

  bool small_and_stable =
      (fabs(e) >= E_PERSIST_MIN) &&
      (fabs(e) <= E_PERSIST_MAX) &&
      (e * e_prev > 0.0);     // same sign as last loop

  if (small_and_stable) {
    persist_t += dt;          // accumulate persistence time
  } else {
    persist_t = 0.0;          // reset if large jump, sign flip, or too small/large
  }

  double I_boost = exp(I_BOOST_RATE * persist_t);
  if (I_boost > I_BOOST_MAX) I_boost = I_BOOST_MAX;
  // -----------------------------------------------------------------

  // 4) Conditional integration (anti-windup)
  double u_pd  = Kp * e - Kd * theta_dot_f;
  double u_pred = u_pd + Ki * I_state;

  bool will_saturate_hi = (u_pred >  (MAX_PWM - 1));
  bool will_saturate_lo = (u_pred < -(MAX_PWM - 1));

  bool integrate = true;
  if (will_saturate_hi && e > 0) integrate = false;
  if (will_saturate_lo && e < 0) integrate = false;

  if (integrate) {
    // Exponentially accelerated integration when small error persists
    I_state += (I_boost * e * dt);   // deg*s scaled by boost
  }

  // Clamp integral so its contribution stays bounded
  double I_term = Ki * I_state;
//   if (I_term >  I_max_pwm) { I_term =  I_max_pwm; I_state = I_max_pwm / Ki; }
//   if (I_term < -I_max_pwm) { I_term = -I_max_pwm; I_state = -I_max_pwm / Ki; }

  // 5) Raw command
  double u = u_pd + I_term;
  Serial.println(u);

  // 6) Small deadband → zero (don’t snap to ±MIN)
  if (fabs(u) < MIN_PWM) u = 0.0;

  // 7) Saturate to driver limits (inside valid range)
  if (u >  (MAX_PWM - 1)) u =  (MAX_PWM - 1);
  if (u < -(MAX_PWM - 1)) u = -(MAX_PWM - 1);

  // Optional: reset integral if we slam a limit switch
  if (atLeftLimit() || atRightLimit()) {
    I_state = 0.0;
    persist_t = 0.0;  // also reset persistence timer
  }

  motor_speed = (int)u;
  e_prev = e;  // update after using it
}




void parseCommand(String command) {
  command.trim();  // remove whitespace
  
  int eqIndex = command.indexOf('=');
  if (eqIndex == -1) return; // no '=' found
  
  String key = command.substring(0, eqIndex);
  String valueStr = command.substring(eqIndex + 1);
  
  float value = valueStr.toFloat();
  
  if (key == "Kp") {
    Kp = value;
    Serial.print("Set Kp = ");
    Serial.println(Kp);
  } else if (key == "Kd") {
    Kd = value;
    Serial.print("Set Kd = ");
    Serial.println(Kd);
  } else if (key == "Ki" || key == "ki") {
    Ki = value;
    Serial.print("Set Ki = ");
    Serial.println(Ki);
  } else {
    Serial.print("Unknown key: ");
    Serial.println(key);
  }
}