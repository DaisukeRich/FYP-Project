/*
  Autonomous Steering Controller (Arduino) - INF ONLY
  ------------------------------------------------------------
  Purpose:
  - Receives steering command from Python over Serial:
      "INF:<norm>\n"
      where <norm> is a float in [-1.0, +1.0]
  - Converts norm -> target steering degrees (±45 deg)
  - Reads potentiometer feedback (actual steering)
  - Uses PID to drive steering motor through H-bridge (IN1/IN2/PWM)
  - Rear motor runs only when "armed" (after receiving valid commands)
  - Failsafe: if commands stop for ARM_TIMEOUT_MS -> disarm + stop motors

  Wiring:
  POT -> A0 (wiper), +5V, GND
  Steering H-bridge: IN1=10, IN2=11, PWM=6
  Rear H-bridge:     IN1=8,  IN2=9,  PWM=5
*/

#include <Arduino.h>

// =====================
// Pins
// =====================
const int POT_PIN    = A0;

// Steering motor driver pins
const int IN1_STEER  = 10;
const int IN2_STEER  = 11;
const int PWM_STEER  = 6;

// Rear motor driver pins
const int IN1_REAR   = 8;
const int IN2_REAR   = 9;
const int PWM_REAR   = 5;

// =====================
// Pot calibration (your measured 3 points)
// LEFT=826, CENTER=437, RIGHT=226
// =====================
const int POT_LEFT   = 826;
const int POT_CENTER = 437;
const int POT_RIGHT  = 226;

/*
  Steering angle definition (degrees):
  - CENTER = 0.0 deg
  - LEFT   = +MAX_DEG
  - RIGHT  = -MAX_DEG
*/
const float MAX_DEG = 45.0f;

// =====================
// Incoming target from Python
// =====================
float targetNorm = 0.0f;   // command from Python in [-1..+1]
float targetDeg  = 0.0f;   // converted target in degrees [-45..+45]

// =====================
// Arming / failsafe
// =====================
// Car will NOT move until it receives at least 1 valid INF command.
bool armed = false;

// If no INF updates for this long -> DISARM + stop motors
const unsigned long ARM_TIMEOUT_MS = 800;
unsigned long lastCmdTime = 0;   // last time we received a valid INF command

// =====================
// PID parameters (tune)
// =====================
// Gain (Kp, Ki and Kd) can be tune later if needed.
float Kp = 0.8f;
float Ki = 0.0f;
float Kd = 0.10f;

float integral  = 0.0f;
float lastError = 0.0f;

// Anti-windup
const float INTEGRAL_LIMIT = 60.0f;

// Control loop timing
const unsigned long LOOP_MS = 20;  // 50 Hz
unsigned long lastLoop = 0;

// Steering motor PWM behavior
const int PWM_MIN = 60;     // minimum PWM to overcome friction
const int PWM_MAX = 200;    // cap steering PWM
const float DEADBAND_DEG = 1.0f;

// Rear motor speed
const int REAR_PWM = 110;

// =====================
// Helper: clamp
// =====================
template <typename T>
T clampT(T x, T lo, T hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// =====================
// 3-point mapping: pot raw -> steering degrees (piecewise linear)
// =====================
// Converts the current pot ADC reading into an estimated steering angle.
float potToDeg(int raw) {
  int lo = min(POT_LEFT, POT_RIGHT);
  int hi = max(POT_LEFT, POT_RIGHT);
  raw = clampT(raw, lo, hi);

  // raw >= center => 0..+MAX (left)
  if (raw >= POT_CENTER) {
    float denom = float(POT_LEFT - POT_CENTER);
    if (fabs(denom) < 1e-6) return 0.0f;
    float t = float(raw - POT_CENTER) / denom;  // 0..1
    return t * MAX_DEG;
  }
  // raw < center => -MAX..0 (right)
  else {
    float denom = float(POT_CENTER - POT_RIGHT);
    if (fabs(denom) < 1e-6) return 0.0f;
    float t = float(raw - POT_RIGHT) / denom;   // 0..1
    return (-MAX_DEG) + t * (MAX_DEG);
  }
}

// =====================
// Drive steering motor with direction + PWM
// =====================
// pidOut sign decides direction. Magnitude decides PWM.
// errorDeg is used for deadband stop.
void driveSteer(float pidOut, float errorDeg) {
  if (fabs(errorDeg) <= DEADBAND_DEG) {
    // within deadband: stop motor (prevents hunting)
    analogWrite(PWM_STEER, 0);
    digitalWrite(IN1_STEER, LOW);
    digitalWrite(IN2_STEER, LOW);
    return;
  }

  bool turnLeft = (pidOut > 0);

  int pwm = (int)fabs(pidOut);
  pwm = clampT(pwm, PWM_MIN, PWM_MAX);

  if (turnLeft) {
    digitalWrite(IN1_STEER, LOW);
    digitalWrite(IN2_STEER, HIGH);
    analogWrite(PWM_STEER, pwm);
  } else {
    digitalWrite(IN1_STEER, HIGH);
    digitalWrite(IN2_STEER, LOW);
    analogWrite(PWM_STEER, pwm);
  }
}

// =====================
// Rear motor control
// =====================
void driveRear(bool enableRear) {
  if (!enableRear) {
    analogWrite(PWM_REAR, 0);
    digitalWrite(IN1_REAR, LOW);
    digitalWrite(IN2_REAR, LOW);
    return;
  }

  // forward direction
  digitalWrite(IN1_REAR, HIGH);
  digitalWrite(IN2_REAR, LOW);
  analogWrite(PWM_REAR, REAR_PWM);
}

// =====================
// Read Serial command from Python (INF ONLY)
// Format: "INF:<float>\n"  where float in [-1..+1]
// =====================
void readSerialCmd() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("INF:")) return;

  // Parse numeric part
  line.remove(0, 4);
  float v = line.toFloat();
  v = clampT(v, -1.0f, 1.0f);

  // Mark time of last valid command for failsafe
  lastCmdTime = millis();

  // Arm on first valid command
  if (!armed) {
    armed = true;

    // Safety: start neutral at the moment of arming
    targetNorm = 0.0f;
    targetDeg  = 0.0f;

    integral  = 0.0f;
    lastError = 0.0f;
  } else {
    // Normal operation: update target steering
    targetNorm = v;
    targetDeg  = targetNorm * MAX_DEG;  // norm -> degrees
  }
}

// =====================
// Stop motors + reset PID (safe state)
// =====================
void stopAllAndResetPID() {
  // stop steering
  analogWrite(PWM_STEER, 0);
  digitalWrite(IN1_STEER, LOW);
  digitalWrite(IN2_STEER, LOW);

  // stop rear
  driveRear(false);

  // reset PID state
  integral  = 0.0f;
  lastError = 0.0f;
}

void setup() {
  // MUST match Python BAUD
  Serial.begin(115200);
  Serial.setTimeout(5);

  pinMode(IN1_STEER, OUTPUT);
  pinMode(IN2_STEER, OUTPUT);
  pinMode(PWM_STEER, OUTPUT);

  pinMode(IN1_REAR, OUTPUT);
  pinMode(IN2_REAR, OUTPUT);
  pinMode(PWM_REAR, OUTPUT);

  stopAllAndResetPID();

  armed = false;
  lastCmdTime = 0;
  lastLoop = millis();
}

void loop() {
  // Read latest command (if any)
  readSerialCmd();

  unsigned long now = millis();

  // Failsafe: if inference stops sending -> disarm + stop
  if (armed && (now - lastCmdTime > ARM_TIMEOUT_MS)) {
    armed = false;
    targetDeg = 0.0f; // optional neutral
    stopAllAndResetPID();
  }

  // Run control loop at fixed rate
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  // If not armed, stay idle
  if (!armed) {
    stopAllAndResetPID();
    return;
  }

  // Feedback: read current steering angle
  int potRaw = analogRead(POT_PIN);
  float currentDeg = potToDeg(potRaw);

  // PID error
  float error = targetDeg - currentDeg;

  // Integral (with anti-windup)
  integral += error * dt;
  integral = clampT(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative
  float deriv = (error - lastError) / max(dt, 1e-3f);
  lastError = error;

  // PID output -> motor drive
  float pidOut = Kp * error + Ki * integral + Kd * deriv;
  pidOut = clampT(pidOut, -255.0f, 255.0f);

  driveSteer(pidOut, error);

  // Rear motor ON only when armed (inference active)
  driveRear(true);

  // Telemetry (Serial Monitor)
  Serial.print("ARM:");
  Serial.print(armed ? 1 : 0);
  Serial.print(",POT:");
  Serial.print(potRaw);
  Serial.print(",CUR:");
  Serial.print(currentDeg, 2);
  Serial.print(",TGT:");
  Serial.print(targetDeg, 2);
  Serial.print(",OUT:");
  Serial.println(pidOut, 1);
}
