/*
 * ============================================================
 *  Thrust Vector Control (TVC) — ESP32
 *  Sensors  : MPU6050 (I2C, 0x68)
 *  Actuators: 2× Servo  (Pitch → X_SERVO_PIN, Yaw → Y_SERVO_PIN)
 *  Control  : Complementary Filter + Dual PID
 
 * Author - Daksh Fofindi (Daksh Aerospace)

# If you liked my work please follow me on below social media profiles

youtube - https://m.youtube.com/@dakshfofindi907/videos?view=0&sort=dd&shelf_id=2
<br>
Website - http://www.dakshaerospace.in/
<br>
instagram - https://www.instagram.com/dakshfofindi/
<br>
X - https://x.com/dakshfofindi
<br>
LinkedIn - https://in.linkedin.com/in/dakshfofindi

 * ============================================================
 */

#include <Wire.h>
#include <ESP32Servo.h>

// ─────────────────────── Pin Definitions ────────────────────
#define LED_PIN       13
#define BUZZER_PIN    27
#define X_SERVO_PIN   32   // Pitch axis
#define Y_SERVO_PIN   33   // Yaw axis

// ─────────────────────── MPU6050 Registers ──────────────────
#define MPU6050_ADDR  0x68
#define ACCEL_XOUT_H  0x3B
#define PWR_MGMT_1    0x6B

// ─────────────────────── Filter Constant ────────────────────
#define ALPHA         0.98f   // Complementary filter weight (gyro side)

// ─────────────────────── Servo Limits ───────────────────────
#define SERVO_CENTER  90
#define SERVO_MIN     60
#define SERVO_MAX     120

// ─────────────────────── PID Gains ──────────────────────────
// Tune these values for your rocket's mass / moment of inertia

// --- Pitch PID ---
float Kp_pitch = 1.2f;
float Ki_pitch = 0.01f;
float Kd_pitch = 0.4f;

// --- Yaw PID ---
float Kp_yaw   = 1.2f;
float Ki_yaw   = 0.01f;
float Kd_yaw   = 0.4f;

// Integral wind-up clamp (degrees)
#define INTEGRAL_LIMIT 30.0f

// ─────────────────────── Objects ────────────────────────────
Servo xServo;   // controls pitch
Servo yServo;   // controls yaw

// ─────────────────────── State Variables ────────────────────
float compPitch = 0.0f, compYaw = 0.0f;

// PID state
float pitchIntegral = 0.0f, pitchPrevError = 0.0f;
float yawIntegral   = 0.0f, yawPrevError   = 0.0f;

unsigned long lastTime = 0;
unsigned long lastBlink = 0;
bool blinkState = false;

// Setpoints (target orientation, degrees)
const float pitchSetpoint = 0.0f;
const float yawSetpoint   = 0.0f;

// ─────────────────────── MPU6050 Init ───────────────────────
void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);   // wake up
  Wire.endTransmission(true);
  delay(100);
}

// ─────────────────────── Read MPU6050 ───────────────────────
struct IMUData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

IMUData readMPU6050() {
  IMUData d;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  d.ax = (int16_t)(Wire.read() << 8 | Wire.read());
  d.ay = (int16_t)(Wire.read() << 8 | Wire.read());
  d.az = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // skip internal temperature
  d.gx = (int16_t)(Wire.read() << 8 | Wire.read());
  d.gy = (int16_t)(Wire.read() << 8 | Wire.read());
  d.gz = (int16_t)(Wire.read() << 8 | Wire.read());
  return d;
}

// ─────────────────────── PID Compute ────────────────────────
/*
 * Returns a correction value (degrees) to add to SERVO_CENTER.
 * Positive output → deflect servo in one direction.
 */
float computePID(float setpoint, float measured,
                 float &integral, float &prevError,
                 float Kp, float Ki, float Kd,
                 float dt) {

  float error = setpoint - measured;

  // Integral with anti-windup clamp
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (dt > 0) ? (error - prevError) / dt : 0.0f;
  prevError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// ─────────────────────── Setup ──────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA=21, SCL=22 (standard ESP32 pins)

  pinMode(LED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN,    LOW);
  digitalWrite(BUZZER_PIN, LOW);

  initMPU6050();

  xServo.attach(X_SERVO_PIN);
  yServo.attach(Y_SERVO_PIN);
  xServo.write(SERVO_CENTER);
  yServo.write(SERVO_CENTER);

  delay(500);
  lastTime = millis();
  Serial.println("TVC System Ready.");
}

// ─────────────────────── Main Loop ──────────────────────────
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // Safety: skip iteration if dt is zero or unreasonably large
  if (dt <= 0.0f || dt > 0.5f) return;

  // ── 1. Read IMU ──────────────────────────────────────────
  IMUData imu = readMPU6050();

  // ── 2. Accel angles (MPU6050 mounted Y-axis upward) ──────
  // accelPitch: forward / back tilt
  // accelYaw  : left  / right tilt
  float accelPitch = atan2f((float)(-imu.az), (float)(imu.ay)) * (180.0f / PI);
  float accelYaw   = atan2f((float)( imu.ax), (float)(imu.ay)) * (180.0f / PI);

  // ── 3. Gyro rates (°/s) ─────────────────────────────────
  float gyroPitchRate = imu.gx / 131.0f;
  float gyroYawRate   = imu.gz / 131.0f;

  // ── 4. Complementary Filter ──────────────────────────────
  compPitch = ALPHA * (compPitch + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
  compYaw   = ALPHA * (compYaw   + gyroYawRate   * dt) + (1.0f - ALPHA) * accelYaw;

  // ── 5. PID Control ───────────────────────────────────────
  float pitchCorrection = computePID(pitchSetpoint, compPitch,
                                     pitchIntegral, pitchPrevError,
                                     Kp_pitch, Ki_pitch, Kd_pitch, dt);

  float yawCorrection   = computePID(yawSetpoint, compYaw,
                                     yawIntegral, yawPrevError,
                                     Kp_yaw, Ki_yaw, Kd_yaw, dt);

  // ── 6. Write Servos ──────────────────────────────────────
  // Correction added to center position; clamp to safe range
  int servoX = constrain((int)(SERVO_CENTER + pitchCorrection), SERVO_MIN, SERVO_MAX);
  int servoY = constrain((int)(SERVO_CENTER + yawCorrection),   SERVO_MIN, SERVO_MAX);
  xServo.write(servoX);
  yServo.write(servoY);

  // ── 7. Heartbeat LED / Buzzer ────────────────────────────
  if (now - lastBlink > 500) {
    blinkState = !blinkState;
    digitalWrite(LED_PIN,    blinkState);
    digitalWrite(BUZZER_PIN, blinkState);
    lastBlink = now;
  }

  // ── 8. Serial Telemetry ──────────────────────────────────
  Serial.print("Pitch: ");   Serial.print(compPitch,   2);
  Serial.print(" | Yaw: ");  Serial.print(compYaw,     2);
  Serial.print(" | SrvX: "); Serial.print(servoX);
  Serial.print(" | SrvY: "); Serial.println(servoY);
}
