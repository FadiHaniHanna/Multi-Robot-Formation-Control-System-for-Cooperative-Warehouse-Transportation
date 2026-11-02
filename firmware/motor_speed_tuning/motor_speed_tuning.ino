#include <Arduino.h>
#include <math.h>

// ============================================================
//                      USER PIN SECTION
// ============================================================

#define M8_RPWM_PIN   25
#define M8_LPWM_PIN   26
#define M8_ENC_A_PIN  35
#define M8_ENC_B_PIN  34

#define M6_RPWM_PIN   27
#define M6_LPWM_PIN   14
#define M6_ENC_A_PIN  22
#define M6_ENC_B_PIN  21

#define M5_RPWM_PIN   32
#define M5_LPWM_PIN   33
#define M5_ENC_A_PIN  4
#define M5_ENC_B_PIN  15

#define M7_RPWM_PIN   18
#define M7_LPWM_PIN   19
#define M7_ENC_A_PIN  23
#define M7_ENC_B_PIN  13

// ============================================================
//                    GENERAL CONTROLLER SETTINGS
// ============================================================
const float GEAR_RATIO = 1.0f;
const float RPM_ALPHA = 0.40f;
const float MAX_TARGET_STEP_PER_UPDATE = 4.0f;   // RPM every 100 ms

const float PWM_STEP_UP   = 24.0f;
const float PWM_STEP_DOWN = 10.0f;

const uint32_t CONTROL_PERIOD_MS = 100;
const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_RES_BITS = 8;

// Logging
bool logEnabled = true;
unsigned long lastLogTime = 0;
const unsigned long LOG_PERIOD_MS = 500;
int logMotorId = -1;   // -1 = all running motors

// ============================================================
//               ROBOT-BODY COMMAND / KINEMATICS
// ============================================================
// Physical matrix-based tuning mode
const float WHEEL_RADIUS_M = 0.05f;
const float LX_M = 0.10f;
const float LY_M = 0.10f;

// Matrix row order:
// w1 = Front Left  = M8
// w2 = Front Right = M5
// w3 = Back Left   = M6
// w4 = Back Right  = M7
int wheelKinSign[4] = {+1, +1, +1, +1};

// Direction-aware wheel command scaling
// order = [FL(M8), FR(M5), BL(M6), BR(M7)]
float wheelScaleFwd[4] = {1.00f, 1.02f, 1.08f, 1.07f};
float wheelScaleRev[4] = {1.03f, 1.04f, 1.12f, 1.05f};

// Last body command
float cmdVx = 0.0f;
float cmdVy = 0.0f;
float cmdWz = 0.0f;
bool bodyCommandMode = false;

// ============================================================
//                         MOTOR STRUCT
// ============================================================
struct MotorState {
  uint8_t id;

  uint8_t rpwmPin;
  uint8_t lpwmPin;
  uint8_t encAPin;
  uint8_t encBPin;

  float ppr;

  // Feedforward coefficients
  float ffLowOffset;
  float ffLowSlope;
  float ffHighOffset;
  float ffHighSlope;

  // PID trim parameters
  float Kp;
  float Ki;
  float Kd;
  float clamp;

  // Encoder state
  volatile long encoderCount;
  volatile int lastEncoded;

  // Targets
  float commandedTargetRPM;
  float activeTargetRPM;
  float targetRPM;

  // Controller state
  float integral;
  float lastError;
  long lastEncoderCount;
  unsigned long lastTime;

  float filteredRPM;
  float appliedPWM;

  // Last measured / commanded values for logging
  float currentRPM;
  int lastFFPWM;
  int lastPWM;
};

// ============================================================
//                   MOTOR CONFIGS: 5,6,7,8
// ============================================================
MotorState motors[4] = {
  {5, M5_RPWM_PIN, M5_LPWM_PIN, M5_ENC_A_PIN, M5_ENC_B_PIN, 1972.0,
   17.0, 1.12, 13.4, 1.135,
   0, 0, 0, 0,
   0, 0,
   0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0},

  {6, M6_RPWM_PIN, M6_LPWM_PIN, M6_ENC_A_PIN, M6_ENC_B_PIN, 1972.0,
   19.0, 1.18, 14.7, 1.20,
   0, 0, 0, 0,
   0, 0,
   0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0},

  {7, M7_RPWM_PIN, M7_LPWM_PIN, M7_ENC_A_PIN, M7_ENC_B_PIN, 1923.0,
   18.5, 1.12, 14.6, 1.14,
   0, 0, 0, 0,
   0, 0,
   0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0},

  {8, M8_RPWM_PIN, M8_LPWM_PIN, M8_ENC_A_PIN, M8_ENC_B_PIN, 1979.2,
   18.5, 1.18, 13.5, 1.14,
   0, 0, 0, 0,
   0, 0,
   0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0}
};

// ============================================================
//                      ENCODER ISR HELPERS
// ============================================================
void IRAM_ATTR updateEncoderISR(uint8_t idx) {
  MotorState &m = motors[idx];

  int MSB = digitalRead(m.encAPin);
  int LSB = digitalRead(m.encBPin);
  int encoded = (MSB << 1) | LSB;
  int sum = (m.lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) m.encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) m.encoderCount--;

  m.lastEncoded = encoded;
}

void IRAM_ATTR encoderISR_M5() { updateEncoderISR(0); }
void IRAM_ATTR encoderISR_M6() { updateEncoderISR(1); }
void IRAM_ATTR encoderISR_M7() { updateEncoderISR(2); }
void IRAM_ATTR encoderISR_M8() { updateEncoderISR(3); }

// ============================================================
//                    MOTOR-SPECIFIC PID TRIM
// ============================================================
void updatePIDGains(MotorState &m) {
  float absTarget = fabsf(m.targetRPM);

  if (m.id == 5) {
    if (absTarget <= 40.0f) {
      m.Kp = 1.55; m.Ki = 0.65; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 70.0f) {
      m.Kp = 1.70; m.Ki = 0.80; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 100.0f) {
      m.Kp = 1.90; m.Ki = 0.90; m.Kd = 0.0; m.clamp = 22.0;
    } else if (absTarget <= 130.0f) {
      m.Kp = 2.10; m.Ki = 1.00; m.Kd = 0.0; m.clamp = 22.0;
    } else {
      m.Kp = 2.20; m.Ki = 1.10; m.Kd = 0.0; m.clamp = 26.0;
    }
    return;
  }

  if (m.id == 6) {
    if (absTarget <= 40.0f) {
      m.Kp = 1.55; m.Ki = 0.60; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 70.0f) {
      m.Kp = 1.70; m.Ki = 0.82; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 100.0f) {
      m.Kp = 1.90; m.Ki = 0.95; m.Kd = 0.0; m.clamp = 20.8;
    } else if (absTarget <= 130.0f) {
      m.Kp = 2.10; m.Ki = 1.05; m.Kd = 0.0; m.clamp = 24.0;
    } else {
      m.Kp = 2.20; m.Ki = 1.10; m.Kd = 0.0; m.clamp = 26.0;
    }
    return;
  }

  if (m.id == 7) {
    if (absTarget <= 40.0f) {
      m.Kp = 1.55; m.Ki = 0.60; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 70.0f) {
      m.Kp = 1.70; m.Ki = 0.80; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 100.0f) {
      m.Kp = 1.90; m.Ki = 0.95; m.Kd = 0.0; m.clamp = 22.0;
    } else if (absTarget <= 130.0f) {
      m.Kp = 2.10; m.Ki = 1.05; m.Kd = 0.0; m.clamp = 24.0;
    } else {
      m.Kp = 2.20; m.Ki = 1.10; m.Kd = 0.0; m.clamp = 26.0;
    }
    return;
  }

  if (m.id == 8) {
    if (absTarget <= 40.0f) {
      m.Kp = 1.55; m.Ki = 0.65; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 70.0f) {
      m.Kp = 1.75; m.Ki = 0.82; m.Kd = 0.0; m.clamp = 20.0;
    } else if (absTarget <= 100.0f) {
      m.Kp = 1.95; m.Ki = 0.95; m.Kd = 0.0; m.clamp = 22.0;
    } else if (absTarget <= 130.0f) {
      m.Kp = 2.10; m.Ki = 1.05; m.Kd = 0.0; m.clamp = 24.0;
    } else {
      m.Kp = 2.20; m.Ki = 1.10; m.Kd = 0.0; m.clamp = 26.0;
    }
    return;
  }
}

// ============================================================
//              DIRECTION-AWARE LOW-SPEED BOOST
// ============================================================
// Based on your ground logs:
// - M6 reverse is the weakest
// - M7 forward is also weak
// - M5 reverse needs some help
// - M8 is the strongest / least problematic
int extraLowSpeedBoostPWM(const MotorState &m, float signedTargetRPM) {
  float absTarget = fabsf(signedTargetRPM);
  if (absTarget > 40.0f) return 0;

  bool forward = (signedTargetRPM >= 0.0f);

  switch (m.id) {
    case 5: return forward ? 2 : 4;
    case 6: return forward ? 5 : 7;
    case 7: return forward ? 5 : 3;
    case 8: return forward ? 2 : 3;
    default: return 0;
  }
}
// ============================================================
//                    FEEDFORWARD PER MOTOR
// ============================================================
int feedForwardPWM(const MotorState &m, float tgtAbs, float signedTargetRPM) {
  if (tgtAbs <= 0.0f) return 0;

  float ff = 0.0f;
  if (tgtAbs <= 40.0f) {
    ff = m.ffLowOffset + m.ffLowSlope * tgtAbs;
  } else {
    ff = m.ffHighOffset + m.ffHighSlope * tgtAbs;
  }

  int pwm = (int)(ff + 0.5f);
  pwm += extraLowSpeedBoostPWM(m, signedTargetRPM);

  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  return pwm;
}

// ============================================================
//                      HELPER FUNCTIONS
// ============================================================
void stopMotor(MotorState &m) {
  m.commandedTargetRPM = 0.0f;
  m.activeTargetRPM = 0.0f;
  m.targetRPM = 0.0f;
  m.integral = 0.0f;
  m.lastError = 0.0f;
  m.appliedPWM = 0.0f;
  m.currentRPM = 0.0f;
  m.filteredRPM = 0.0f;
  m.lastFFPWM = 0;
  m.lastPWM = 0;

  ledcWrite(m.rpwmPin, 0);
  ledcWrite(m.lpwmPin, 0);
}

void stopAllMotors() {
  bodyCommandMode = false;
  cmdVx = 0.0f;
  cmdVy = 0.0f;
  cmdWz = 0.0f;
  for (int i = 0; i < 4; i++) stopMotor(motors[i]);
}

void setTargetRPM(MotorState &m, float newTarget) {
  if (fabsf(newTarget) < 0.01f) {
    stopMotor(m);
    return;
  }

  if (newTarget != m.commandedTargetRPM) {
    float deltaTarget = fabsf(newTarget - m.commandedTargetRPM);
    m.commandedTargetRPM = newTarget;

    if (deltaTarget > 40.0f) {
      m.integral = 0.0f;
    } else if (fabsf(m.commandedTargetRPM) >= 90.0f && deltaTarget <= 20.0f) {
      m.integral *= 0.92f;
    } else {
      m.integral *= 0.80f;
    }

    m.lastError = 0.0f;
  }
}

void updateMotorControl(MotorState &m, unsigned long now) {
  if (now - m.lastTime < CONTROL_PERIOD_MS) return;

  float dt = (now - m.lastTime) / 1000.0f;
  m.lastTime = now;

  // ---------- TARGET RAMP ----------
  float delta = m.commandedTargetRPM - m.activeTargetRPM;
  if (delta > MAX_TARGET_STEP_PER_UPDATE)  delta = MAX_TARGET_STEP_PER_UPDATE;
  if (delta < -MAX_TARGET_STEP_PER_UPDATE) delta = -MAX_TARGET_STEP_PER_UPDATE;

  m.activeTargetRPM += delta;
  m.targetRPM = m.activeTargetRPM;

  // ---------- CLEAN ZERO-TARGET STOP ----------
  if (fabsf(m.commandedTargetRPM) < 0.01f && fabsf(m.activeTargetRPM) < 0.01f) {
    m.targetRPM = 0.0f;
    m.integral = 0.0f;
    m.lastError = 0.0f;
    m.appliedPWM = 0.0f;
    m.filteredRPM = 0.0f;
    m.currentRPM = 0.0f;
    m.lastPWM = 0;
    m.lastFFPWM = 0;
    ledcWrite(m.rpwmPin, 0);
    ledcWrite(m.lpwmPin, 0);
    return;
  }

  updatePIDGains(m);

  long deltaTicks = m.encoderCount - m.lastEncoderCount;
  m.lastEncoderCount = m.encoderCount;

  // signed RPM
  float rpm = (deltaTicks * 60.0f) / (m.ppr * GEAR_RATIO * dt);

  // filter
  m.filteredRPM = RPM_ALPHA * rpm + (1.0f - RPM_ALPHA) * m.filteredRPM;
  rpm = m.filteredRPM;
  m.currentRPM = rpm;

  int ffPWM = feedForwardPWM(m, fabsf(m.targetRPM), m.targetRPM);
  m.lastFFPWM = ffPWM;

  float ffTerm = 0.0f;
  if (m.targetRPM > 0.0f) ffTerm = (float)ffPWM;
  else if (m.targetRPM < 0.0f) ffTerm = -(float)ffPWM;

  float error = m.targetRPM - rpm;
  m.integral += error * dt;

  if (m.integral > m.clamp) m.integral = m.clamp;
  if (m.integral < -m.clamp) m.integral = -m.clamp;

  float derivative = (error - m.lastError) / dt;
  float trim = m.Kp * error + m.Ki * m.integral + m.Kd * derivative;

  float desiredPWM = ffTerm + trim;
  if (desiredPWM > 255.0f) desiredPWM = 255.0f;
  if (desiredPWM < -255.0f) desiredPWM = -255.0f;

  // ---------- PWM SLEW ----------
  if (desiredPWM > m.appliedPWM + PWM_STEP_UP) {
    m.appliedPWM += PWM_STEP_UP;
  }
  else if (desiredPWM < m.appliedPWM - PWM_STEP_DOWN) {
    m.appliedPWM -= PWM_STEP_DOWN;
  }
  else {
    m.appliedPWM = desiredPWM;
  }

  if (m.appliedPWM > 255.0f) m.appliedPWM = 255.0f;
  if (m.appliedPWM < -255.0f) m.appliedPWM = -255.0f;

  int pwmMag = (int)fabsf(m.appliedPWM);
  m.lastPWM = (m.appliedPWM >= 0.0f) ? pwmMag : -pwmMag;

  if (m.appliedPWM > 0.5f) {
    ledcWrite(m.rpwmPin, pwmMag);
    ledcWrite(m.lpwmPin, 0);
  }
  else if (m.appliedPWM < -0.5f) {
    ledcWrite(m.rpwmPin, 0);
    ledcWrite(m.lpwmPin, pwmMag);
  }
  else {
    ledcWrite(m.rpwmPin, 0);
    ledcWrite(m.lpwmPin, 0);
  }

  m.lastError = error;
}

// ============================================================
//                  BODY KINEMATICS FUNCTIONS
// ============================================================
void bodyToWheelRPM(float vx, float vy, float wz, float wheelRPM[4]) {
  float a = LX_M + LY_M;

  // Row order:
  // w1 = FL(M8)
  // w2 = FR(M5)
  // w3 = BL(M6)
  // w4 = BR(M7)
  float w1 = (vx - vy - a * wz) / WHEEL_RADIUS_M;
  float w2 = (vx + vy + a * wz) / WHEEL_RADIUS_M;
  float w3 = (vx + vy - a * wz) / WHEEL_RADIUS_M;
  float w4 = (vx - vy + a * wz) / WHEEL_RADIUS_M;

  float wheelRadPerSec[4] = {w1, w2, w3, w4};

  for (int i = 0; i < 4; i++) {
    float rpm = wheelRadPerSec[i] * 60.0f / (2.0f * PI);
    float scale = (rpm >= 0.0f) ? wheelScaleFwd[i] : wheelScaleRev[i];
    wheelRPM[i] = wheelKinSign[i] * scale * rpm;
  }

  // Apply this ONCE, outside the loop
  if (fabsf(vy) > 0.05f && fabsf(vx) < 0.03f && fabsf(wz) < 0.03f) {
    if (vy > 0.0f) {
      // +Y : FL(M8) reverse, FR(M5) forward, BL(M6) forward, BR(M7) reverse
      wheelRPM[0] *= 1.03f; // M8
      wheelRPM[2] *= 1.03f; // M6
      wheelRPM[3] *= 1.02f; // M7
    } else {
      // -Y : FL(M8) forward, FR(M5) reverse, BL(M6) reverse, BR(M7) forward
      wheelRPM[1] *= 1.02f; // M5
      wheelRPM[2] *= 1.04f; // M6
      wheelRPM[3] *= 1.03f; // M7
    }
  }
}

void wheelRPMToBody(float physicalWheelRPM[4], float &vx, float &vy, float &wz) {
  float a = LX_M + LY_M;

  float w1 = physicalWheelRPM[0] * 2.0f * PI / 60.0f;
  float w2 = physicalWheelRPM[1] * 2.0f * PI / 60.0f;
  float w3 = physicalWheelRPM[2] * 2.0f * PI / 60.0f;
  float w4 = physicalWheelRPM[3] * 2.0f * PI / 60.0f;

  vx = WHEEL_RADIUS_M * 0.25f * (w1 + w2 + w3 + w4);
  vy = WHEEL_RADIUS_M * 0.25f * (-w1 + w2 + w3 - w4);
  wz = WHEEL_RADIUS_M * (-w1 + w2 - w3 + w4) / (4.0f * a);
}

void setBodyCommand(float vx, float vy, float wz) {
  cmdVx = vx;
  cmdVy = vy;
  cmdWz = wz;
  bodyCommandMode = true;

  float wheelRPM[4];
  bodyToWheelRPM(vx, vy, wz, wheelRPM);

  // mapping:
  // FL = M8 = motors[3]
  // FR = M5 = motors[0]
  // BL = M6 = motors[1]
  // BR = M7 = motors[2]
  setTargetRPM(motors[3], wheelRPM[0]); // M8
  setTargetRPM(motors[0], wheelRPM[1]); // M5
  setTargetRPM(motors[1], wheelRPM[2]); // M6
  setTargetRPM(motors[2], wheelRPM[3]); // M7

  Serial.println("=== BODY COMMAND SET ===");
  Serial.print("Target vx: "); Serial.print(vx, 4);
  Serial.print(" | vy: ");     Serial.print(vy, 4);
  Serial.print(" | wz: ");     Serial.println(wz, 4);

  Serial.print("Wheel targets RPM -> ");
  Serial.print("FL(M8): "); Serial.print(wheelRPM[0], 2);
  Serial.print(" | FR(M5): "); Serial.print(wheelRPM[1], 2);
  Serial.print(" | BL(M6): "); Serial.print(wheelRPM[2], 2);
  Serial.print(" | BR(M7): "); Serial.println(wheelRPM[3], 2);
}

void printBodyLog() {
  if (!bodyCommandMode) return;

  float physicalWheelRPM[4];
  physicalWheelRPM[0] = wheelKinSign[0] * motors[3].currentRPM; // FL=M8
  physicalWheelRPM[1] = wheelKinSign[1] * motors[0].currentRPM; // FR=M5
  physicalWheelRPM[2] = wheelKinSign[2] * motors[1].currentRPM; // BL=M6
  physicalWheelRPM[3] = wheelKinSign[3] * motors[2].currentRPM; // BR=M7

  float actVx, actVy, actWz;
  wheelRPMToBody(physicalWheelRPM, actVx, actVy, actWz);

  Serial.print("BODY | Vx_t: ");
  Serial.print(cmdVx, 4);
  Serial.print(" | Vx_a: ");
  Serial.print(actVx, 4);

  Serial.print(" | Vy_t: ");
  Serial.print(cmdVy, 4);
  Serial.print(" | Vy_a: ");
  Serial.print(actVy, 4);

  Serial.print(" | Wz_t: ");
  Serial.print(cmdWz, 4);
  Serial.print(" | Wz_a: ");
  Serial.println(actWz, 4);
}

// ============================================================
//                         LOGGING
// ============================================================
bool shouldPrintMotor(const MotorState &m) {
  bool running = (fabsf(m.commandedTargetRPM) > 0.0f || fabsf(m.targetRPM) > 0.0f || fabsf(m.appliedPWM) > 0.0f);
  if (!running) return false;
  if (logMotorId == -1) return true;
  return (m.id == logMotorId);
}

void printLogs() {
  if (!logEnabled) return;
  if (millis() - lastLogTime < LOG_PERIOD_MS) return;

  lastLogTime = millis();

  for (int i = 0; i < 4; i++) {
    MotorState &m = motors[i];
    if (!shouldPrintMotor(m)) continue;

    Serial.print("M");
    Serial.print(m.id);
    Serial.print(" | RPM: ");
    Serial.print(m.currentRPM, 2);
    Serial.print(" | Target: ");
    Serial.print(m.targetRPM, 2);
    Serial.print(" | Cmd: ");
    Serial.print(m.commandedTargetRPM, 2);
    Serial.print(" | FF: ");
    Serial.print(m.lastFFPWM);
    Serial.print(" | PWM: ");
    Serial.print(m.lastPWM);
    Serial.print(" | Err: ");
    Serial.print(m.targetRPM - m.currentRPM, 2);
    Serial.print(" | Int: ");
    Serial.print(m.integral, 2);
    Serial.print(" | Kp: ");
    Serial.print(m.Kp, 2);
    Serial.print(" | Ki: ");
    Serial.println(m.Ki, 2);
  }

  printBodyLog();
}

MotorState* getMotorById(int id) {
  for (int i = 0; i < 4; i++) {
    if (motors[i].id == id) return &motors[i];
  }
  return nullptr;
}

void printStatusOnce() {
  for (int i = 0; i < 4; i++) {
    MotorState &m = motors[i];

    Serial.print("M");
    Serial.print(m.id);
    Serial.print(" | RPM: ");
    Serial.print(m.currentRPM, 2);
    Serial.print(" | Cmd: ");
    Serial.print(m.commandedTargetRPM, 2);
    Serial.print(" | Target: ");
    Serial.print(m.targetRPM, 2);
    Serial.print(" | FF: ");
    Serial.print(m.lastFFPWM);
    Serial.print(" | PWM: ");
    Serial.print(m.lastPWM);
    Serial.print(" | Int: ");
    Serial.println(m.integral, 2);
  }
}

// ============================================================
//                      SERIAL COMMANDS
// ============================================================
void readCommands() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "STOP" || upper == "BSTOP") {
    stopAllMotors();
    Serial.println("All motors stopped");
    return;
  }

  if (upper == "STATUS") {
    printStatusOnce();
    return;
  }

  if (upper == "LOG ON") {
    logEnabled = true;
    Serial.println("Logging ON");
    return;
  }

  if (upper == "LOG OFF") {
    logEnabled = false;
    Serial.println("Logging OFF");
    return;
  }

  if (upper == "LOG ALL") {
    logMotorId = -1;
    Serial.println("Logging all running motors");
    return;
  }

  if (upper.startsWith("LOGM ")) {
    int id = upper.substring(5).toInt();
    if (getMotorById(id) != nullptr) {
      logMotorId = id;
      Serial.print("Logging only motor ");
      Serial.println(id);
    } else {
      Serial.println("Bad motor id for LOGM");
    }
    return;
  }

  // BODY vx vy wz   (m/s, m/s, rad/s)
  if (upper.startsWith("BODY ")) {
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    int count = sscanf(line.c_str(), "BODY %f %f %f", &vx, &vy, &wz);
    if (count == 3) {
      setBodyCommand(vx, vy, wz);
      return;
    } else {
      Serial.println("Bad BODY command. Use: BODY vx vy wz");
      return;
    }
  }

  if (upper.startsWith("ALL ")) {
    float rpm = upper.substring(4).toFloat();
    bodyCommandMode = false;
    for (int i = 0; i < 4; i++) setTargetRPM(motors[i], rpm);
    return;
  }

  int id = -1;
  float rpm = 0.0f;
  int spaceIdx = upper.indexOf(' ');

  if (spaceIdx > 0) {
    String first = upper.substring(0, spaceIdx);
    String second = upper.substring(spaceIdx + 1);

    if (first[0] == 'M') id = first.substring(1).toInt();
    else id = first.toInt();

    rpm = second.toFloat();

    MotorState* m = getMotorById(id);
    if (m != nullptr) {
      bodyCommandMode = false;
      setTargetRPM(*m, rpm);
      return;
    }
  }

  Serial.println("Bad command.");
  Serial.println("Use:");
  Serial.println("  BODY 0.10 0.00 0.00  -> +X");
  Serial.println("  BODY -0.10 0.00 0.00 -> -X");
  Serial.println("  BODY 0.00 0.10 0.00  -> +Y");
  Serial.println("  BODY 0.00 -0.10 0.00 -> -Y");
  Serial.println("  BODY 0.00 0.00 0.50  -> pure rotation");
  Serial.println("  ALL 20 / ALL -20");
  Serial.println("  5 20 / 5 -20");
  Serial.println("  STOP");
  Serial.println("  LOG ON / LOG OFF / LOG ALL / LOGM 5");
  Serial.println("  STATUS");
}

// ============================================================
//                            SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    MotorState &m = motors[i];

    pinMode(m.rpwmPin, OUTPUT);
    pinMode(m.lpwmPin, OUTPUT);
    pinMode(m.encAPin, INPUT_PULLUP);
    pinMode(m.encBPin, INPUT_PULLUP);

    if (!ledcAttach(m.rpwmPin, PWM_FREQ, PWM_RES_BITS)) {
      Serial.print("LEDC attach failed for RPWM motor ");
      Serial.println(m.id);
    }
    if (!ledcAttach(m.lpwmPin, PWM_FREQ, PWM_RES_BITS)) {
      Serial.print("LEDC attach failed for LPWM motor ");
      Serial.println(m.id);
    }

    ledcWrite(m.rpwmPin, 0);
    ledcWrite(m.lpwmPin, 0);

    m.commandedTargetRPM = 0.0f;
    m.activeTargetRPM = 0.0f;
    m.targetRPM = 0.0f;
    m.integral = 0.0f;
    m.lastError = 0.0f;
    m.lastEncoderCount = 0;
    m.lastTime = millis();
    m.filteredRPM = 0.0f;
    m.appliedPWM = 0.0f;
    m.encoderCount = 0;
    m.lastEncoded = 0;
    m.currentRPM = 0.0f;
    m.lastFFPWM = 0;
    m.lastPWM = 0;
  }

  attachInterrupt(digitalPinToInterrupt(motors[0].encAPin), encoderISR_M5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[0].encBPin), encoderISR_M5, CHANGE);

  attachInterrupt(digitalPinToInterrupt(motors[1].encAPin), encoderISR_M6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].encBPin), encoderISR_M6, CHANGE);

  attachInterrupt(digitalPinToInterrupt(motors[2].encAPin), encoderISR_M7, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].encBPin), encoderISR_M7, CHANGE);

  attachInterrupt(digitalPinToInterrupt(motors[3].encAPin), encoderISR_M8, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[3].encBPin), encoderISR_M8, CHANGE);

  Serial.println("Car 2 tuning code started");
  Serial.println("Wheel order in matrix:");
  Serial.println("  w1 = FL = M8");
  Serial.println("  w2 = FR = M5");
  Serial.println("  w3 = BL = M6");
  Serial.println("  w4 = BR = M7");
  Serial.println("Use 115200 baud + Newline");
  Serial.println("Commands:");
  Serial.println("  BODY 0.10 0.00 0.00  -> +X");
  Serial.println("  BODY -0.10 0.00 0.00 -> -X");
  Serial.println("  BODY 0.00 0.10 0.00  -> +Y");
  Serial.println("  BODY 0.00 -0.10 0.00 -> -Y");
  Serial.println("  BODY 0.00 0.00 0.50  -> pure rotation");
  Serial.println("  STOP");
  Serial.println("  LOG ON / LOG OFF / LOG ALL / LOGM 5");
  Serial.println("  STATUS");
}

// ============================================================
//                             LOOP
// ============================================================
void loop() {
  readCommands();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    updateMotorControl(motors[i], now);
  }

  printLogs();
}