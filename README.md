Author - Daksh Fofindi (Daksh Aerospace)

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

---


# 🚀 ESP32 Thrust Vector Control (TVC)

A bare-metal Thrust Vector Control firmware for ESP32 using an MPU6050 IMU and two servos, with a complementary filter for attitude estimation and dual PID controllers for pitch and yaw correction.

---

## 📦 Hardware Requirements

| Component | Details |
|---|---|
| Microcontroller | ESP32 (any variant with GPIO 21/22 for I2C) |
| IMU | MPU6050 (I2C, address `0x68`) |
| Servos | 2× standard hobby servo (5V, PWM) |
| LED | Any LED on GPIO 13 (heartbeat indicator) |
| Buzzer | Active buzzer on GPIO 27 |
| Power | 3.3V for MPU6050 logic; 5V for servos (use separate rail) |

### Wiring

```
ESP32 GPIO 21  ──►  MPU6050 SDA
ESP32 GPIO 22  ──►  MPU6050 SCL
ESP32 3.3V     ──►  MPU6050 VCC
ESP32 GND      ──►  MPU6050 GND

ESP32 GPIO 32  ──►  X-Servo (Pitch) Signal
ESP32 GPIO 33  ──►  Y-Servo (Yaw)   Signal
5V Rail        ──►  Both Servo VCC
GND            ──►  Both Servo GND

ESP32 GPIO 13  ──►  LED (+ 220Ω resistor to GND)
ESP32 GPIO 27  ──►  Active Buzzer (+)
GND            ──►  Active Buzzer (-)
```

> ⚠️ **Never power servos from the ESP32's 3.3V or 5V pin.** Use a dedicated 5V BEC or servo rail — servo current spikes will reset the ESP32.

---

## 🛠️ Software Requirements

### Arduino IDE Setup

1. Install **Arduino IDE 2.x** from [arduino.cc](https://www.arduino.cc/en/software)
2. Add ESP32 board support:
   - Go to **File → Preferences**
   - Add to "Additional Boards Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to **Tools → Board → Boards Manager**, search `esp32`, install by Espressif

### Required Libraries

Install all via **Tools → Manage Libraries**:

| Library | Author | Version |
|---|---|---|
| `ESP32Servo` | Kevin Harrington | ≥ 0.13.0 |

> The `Wire.h` library is built into the ESP32 Arduino core — no separate install needed.

---

## 📁 File Structure

```
tvc_pid/
├── tvc-pid.ino      # Main firmware
└── README.md        # This file
```

---

## ⚙️ Configuration

All tunable parameters are at the top of `tvc_pid.ino`:

### IMU Orientation

The MPU6050 is assumed to be mounted with the **Y-axis pointing upward** (along the rocket's thrust axis). If your orientation differs, adjust the `accelPitch` / `accelYaw` `atan2f()` axis mapping accordingly.

```cpp
// Default (Y-axis up)
float accelPitch = atan2f((float)(-imu.az), (float)(imu.ay)) * (180.0f / PI);
float accelYaw   = atan2f((float)( imu.ax), (float)(imu.ay)) * (180.0f / PI);
```

### Complementary Filter

```cpp
#define ALPHA  0.98f   // 0.0 = full accelerometer, 1.0 = full gyro
```

Higher `ALPHA` → smoother but slower to correct drift. Recommended range: `0.95 – 0.99`.

### Servo Limits

```cpp
#define SERVO_CENTER  90    // Neutral position (degrees)
#define SERVO_MIN     60    // Maximum deflection limit
#define SERVO_MAX     120   // Maximum deflection limit
```

Adjust `SERVO_MIN` / `SERVO_MAX` to match the physical travel of your TVC mount.

### PID Gains

```cpp
// Pitch axis
float Kp_pitch = 1.2f;
float Ki_pitch = 0.01f;
float Kd_pitch = 0.4f;

// Yaw axis
float Kp_yaw   = 1.2f;
float Ki_yaw   = 0.01f;
float Kd_yaw   = 0.4f;
```

---

## 🎛️ PID Tuning Guide

Follow this sequence on the bench with the rocket in a gimbal or hand-held fixture:

### Step 1 — Proportional only
Set `Ki = 0`, `Kd = 0`. Increase `Kp` until the mount actively corrects tilts but starts to oscillate. Back off `Kp` by ~20%.

### Step 2 — Add Derivative
Increase `Kd` gradually. It should damp the oscillation from Step 1 without making the response sluggish.

### Step 3 — Add Integral (optional)
A small `Ki` (0.005 – 0.02) removes any steady-state offset. Keep it tiny — too high causes slow hunting/wind-up.

### Anti-Windup Clamp
The integral is clamped to ±30° by `INTEGRAL_LIMIT`. Reduce this if you observe slow oscillation after large disturbances:
```cpp
#define INTEGRAL_LIMIT  30.0f   // degrees
```

---

## 📡 Serial Telemetry

Open Serial Monitor at **115200 baud**. Output format:

```
Pitch: -1.24 | Yaw: 0.87 | SrvX: 89 | SrvY: 91
```

| Field | Description |
|---|---|
| `Pitch` | Filtered pitch angle (°) |
| `Yaw` | Filtered yaw angle (°) |
| `SrvX` | X-servo position sent (°) |
| `SrvY` | Y-servo position sent (°) |

---

## 🔄 Control Flow

```
MPU6050 Raw Data (Accel + Gyro)
        │
        ▼
Complementary Filter  ◄──── ALPHA = 0.98
        │
        ▼
  compPitch, compYaw
        │
        ▼
  Dual PID Controller  ◄──── Setpoint = 0° (upright)
        │
        ▼
  Servo Correction
  (SERVO_CENTER ± output)
        │
        ▼
  xServo (Pitch)   yServo (Yaw)
```

---

## ⚡ Flash Instructions

1. Connect ESP32 via USB
2. Select board: **Tools → Board → ESP32 Dev Module** (or your specific variant)
3. Select correct **Port** under Tools
4. Click **Upload** (Ctrl+U)
5. Open Serial Monitor (Ctrl+Shift+M) at 115200 baud

> If upload fails, hold the **BOOT** button on the ESP32 while clicking Upload, then release after "Connecting…" appears.

---

## ⚠️ Safety Notes

- Always test servo deflection direction on the bench before flight. Flip the sign of `pitchCorrection` or `yawCorrection` if a servo moves the wrong way.
- Never arm/power servos while the rocket is in a confined space.
- The heartbeat LED and buzzer blink every 500 ms to confirm the control loop is running.
- The `dt > 0.5f` guard in `loop()` prevents runaway corrections after watchdog resets or I2C stalls.

---

## 📄 License

MIT License — free to use, modify, and distribute for personal and educational rocketry projects.