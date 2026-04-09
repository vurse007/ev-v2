#include <SPI.h>
#include <Wire.h>
#include <math.h>

// ===================== PINS =====================
#define ENC1_CS 4
#define ENC2_CS 7

#define MPU_ADDR 0x68

#define MOTOR_L_PWM 5
#define MOTOR_L_IN1 8
#define MOTOR_L_IN2 9

#define MOTOR_R_PWM 6
#define MOTOR_R_IN1 2
#define MOTOR_R_IN2 3

// ===================== TIMING =====================
unsigned long lastLoopTime = 0;
const int LOOP_DT = 10; // ms loop timing

int count = 0;

// ===================== STATE =====================
double headingDeg = 0.0;
double yawDeg = 0.0;

double enc1Last = 0, enc2Last = 0;
double enc1Accum = 0, enc2Accum = 0;

double leftDist = 0;
double rightDist = 0;

unsigned long lastIMUTime = 0;

// ===================== CONSTANTS =====================
const double WHEEL_DIAMETER = 0.0508;
const double WHEEL_CIRC = PI * WHEEL_DIAMETER;

// ===================== PID =====================
class PID {
public:
  double kP, kI, kD;
  double error = 0, prevError = 0, totalError = 0;

  PID(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  double compute(double target, double current) {
    error = target - current;

    totalError += error;
    totalError = constrain(totalError, -100, 100); // anti-windup

    double derivative = error - prevError;
    prevError = error;

    return kP * error + kI * totalError + kD * derivative;
  }

  void reset() {
    error = prevError = totalError = 0;
  }
};

PID headingPID(4.5, 0, 600);
PID turnPID(1.2, 0, 15);
PID distancePID(2.0, 0, 35);

// ===================== HELPERS =====================
double wrap360(double a) {
  while (a >= 360) a -= 360;
  while (a < 0) a += 360;
  return a;
}

double angleError(double target, double current) {
  double err = target - current;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  return err;
}

// ===================== ENCODERS =====================
uint16_t readEncoder(int cs) {
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  uint8_t h = SPI.transfer(0x00);
  uint8_t l = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  return (((uint16_t)h << 8) | l) & 0x3FFF;
}

double rawToDeg(uint16_t val) {
  return (val / 16383.0) * 360.0;
}

double updateContinuous(double raw, double &last, double &accum) {
  double delta = raw - last;

  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;

  if (fabs(delta) < 0.02) delta = 0;

  accum += delta;
  last = raw;
  return accum;
}

void updateEncoders() {
  double d1 = rawToDeg(readEncoder(ENC1_CS));
  double d2 = rawToDeg(readEncoder(ENC2_CS));

  double c1 = updateContinuous(d1, enc1Last, enc1Accum);
  double c2 = updateContinuous(d2, enc2Last, enc2Accum);

  double fixed1 = c1 * 0.5;
  double fixed2 = -c2 * 0.5;

  leftDist  = (fixed1 / 360.0) * WHEEL_CIRC;
  rightDist = (fixed2 / 360.0) * WHEEL_CIRC;
}

// ===================== IMU =====================
int16_t gz;
double gyroOffset = 0;

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  gz = Wire.read() << 8 | Wire.read();
}

void updateIMU() {
  unsigned long now = micros();
  double dt = (now - lastIMUTime) / 1e6;
  lastIMUTime = now;

  readGyro();
  double gyroZ = (gz - gyroOffset) / 131.0;

  yawDeg -= gyroZ * dt;
  yawDeg = wrap360(yawDeg);

  headingDeg = yawDeg;
}

// ===================== MOTOR =====================
void setMotor(int pwm, int in1, int in2, double power) {
  power = constrain(power, -1.0, 1.0);

  if (power > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (power < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    power = -power;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwm, power * 255);
}

// ===================== MOTION =====================
void driveDistance(double meters, unsigned long timeoutMs, double speedScale) {

  distancePID.reset();
  headingPID.reset();

  headingPID.kP = 6;
  headingPID.kD = 467;

  int settled = 0;

  double startL = leftDist;
  double startR = rightDist;

  double targetHeading = headingDeg;

  unsigned long startTime = millis();

  while (true) {

    while (millis() - lastLoopTime < LOOP_DT);
    lastLoopTime = millis();

    updateIMU();
    updateEncoders();

    if (timeoutMs > 0 && millis() - startTime > timeoutMs) break;

    double avg = ((leftDist - startL) + (rightDist - startR)) / 2.0;
    double err = meters - avg;

    double base = distancePID.compute(meters, avg);
    base = constrain(base, -0.6, 0.6);
    base *= speedScale;

    double hErr = angleError(targetHeading, headingDeg);

    // deadband
    if (fabs(hErr) < 2.0) hErr = 0;

    double corr = headingPID.compute(0, -hErr);

    // scale correction
    corr *= (0.5 + 0.5 * fabs(base));

    // limit correction
    corr = constrain(corr, -fabs(base), fabs(base));

    double left = base + corr;
    double right = base - corr;

    // 🔥 PREVENT DIRECTION FLIP
    if (base > 0) {
      if (left < 0) left = 0;
      if (right < 0) right = 0;
    }
    if (base < 0) {
      if (left > 0) left = 0;
      if (right > 0) right = 0;
    }

    // minimum drive power
    if (fabs(base) < 0.15 && fabs(err) > 0.05) {
      base = 0.15 * (base >= 0 ? 1 : -1);
    }

    left = constrain(left, -1, 1);
    right = constrain(right, -1, 1);

    setMotor(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, left);
    setMotor(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, right);

    Serial.println(err);

    if (fabs(err) < 0.1) {
      headingPID.kP = 3.0;
      headingPID.kD = 0;
    }

    if (fabs(err) < 0.07) settled++;
    else settled = 0;

    if (settled >= 17) break;
  }

  // ================================
  // HEADING SETTLE PHASE
  // ================================
  const unsigned long HEADING_SETTLE_MS = 500;
  unsigned long settleStart = millis();
  headingPID.reset();

  while (millis() - settleStart < HEADING_SETTLE_MS) {
    while (millis() - lastLoopTime < LOOP_DT);
    lastLoopTime = millis();

    updateIMU();

    double hErr = angleError(targetHeading, headingDeg);
    double correction = headingPID.compute(0, -hErr);
    correction = constrain(correction, -0.35, 0.35);

    setMotor(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2,  correction);
    setMotor(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, -correction);

    Serial.print("[SETTLE] H=");
    Serial.print(headingDeg);
    Serial.print(" err=");
    Serial.println(hErr);
  }

  setMotor(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, 0);
  setMotor(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, 0);
  delay(100);
}

// ===================== TURN =====================
void turnTo(double targetDeg, unsigned long timeoutMs, double speedScale) {

  turnPID.reset();
  turnPID.kP = 0.09;
  turnPID.kD = 2.00;

  unsigned long startTime = millis();

  count=0;

  while (true) {

    while (millis() - lastLoopTime < LOOP_DT);
    lastLoopTime = millis();

    updateIMU();

    if (timeoutMs > 0 && millis() - startTime > timeoutMs) /*break*/;

    double err = angleError(targetDeg, headingDeg);

    double out = turnPID.compute(0, -err);
    out *= speedScale;
    out = constrain(out, -0.6, 0.6);

    Serial.println(err);

    setMotor(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, out);
    setMotor(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, -out);

    if (fabs(err) < 8) {
      turnPID.kP = 0.67;
      turnPID.kD = 0.6;
    }

    if (fabs(err) < 3) count++;

    if (count >= 75) break;
  }

  setMotor(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, 0);
  setMotor(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, 0);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  SPI.begin();
  Wire.begin();

  pinMode(ENC1_CS, OUTPUT);
  pinMode(ENC2_CS, OUTPUT);
  digitalWrite(ENC1_CS, HIGH);
  digitalWrite(ENC2_CS, HIGH);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);

  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  // wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(200);

  // calibrate gyro
  for (int i = 0; i < 500; i++) {
    readGyro();
    gyroOffset += gz;
    delay(2);
  }
  gyroOffset /= 500.0;

  // 🔥 FIX: initialize encoders
  enc1Last = rawToDeg(readEncoder(ENC1_CS));
  enc2Last = rawToDeg(readEncoder(ENC2_CS));

  lastIMUTime = micros();
}

// ===================== LOOP =====================
void loop() {

  updateIMU();   // always running
  updateEncoders();

  turnTo(-90, 2000, 1);
  delay(500);

  driveDistance(0.85, 3000, 1);
  delay(500);

  turnTo(0, 2000, 1);
  delay(500);

  driveDistance(4.0, 6000, 1);
  delay(500);

  turnTo(90, 2000, 1);
  delay(500);

  driveDistance(0.85, 3000, 1);

  // turnTo(180, 2000, 1);
  // delay(500);

  // turnTo(270, 2000, 1);
  // delay(500);

  // turnTo(0, 2000, 1);
  // delay(500);

  // driveDistance(5.0, 3000, 1.0);
  // delay(500);

  while (true);

  // turnTo(90, 2000, 0.5);
  // delay(500);

  // // driveDistance(0.5, 3000, 1.0);
  // // delay(2000);
}