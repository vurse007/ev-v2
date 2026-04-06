// =====================================================
// UPDATED ROBOT CONTROL CODE
// Hardware: H-bridge motors, MPU6050 IMU (I2C), SPI encoders
// =====================================================

// ========== MOTOR PINS (H-bridge) ==========
#define ENA 5    // Left motor PWM
#define IN1 8    // Left motor direction 1
#define IN2 9    // Left motor direction 2
#define ENB 6    // Right motor PWM
#define IN3 2    // Right motor direction 1
#define IN4 3    // Right motor direction 2

// GO button
#define GO 0

// Encoder CS pins (SPI absolute encoders)
#define CS_ENC_L 4
#define CS_ENC_R 7

// IMU I2C address (MPU6050)
#define MPU_ADDR 0x68

#include <Wire.h>
#include <SPI.h>
#include <math.h>

// ========== CONSTANTS ==========
const double MIN_TURN_OUTPUT = 0.2;
const double MIN_BASE_SPEED = -0.2;
const double MAX_BASE_SPEED = 0.4;
const double DISTANCE_ERROR_THRESHOLD = 1.0;  // in equivalent ticks
const int SETTLED_COUNT_THRESHOLD = 2;
const unsigned long HEADING_SETTLE_MS = 500;

// ========== ENCODER GLOBALS ==========
double lastRawL = 0, lastRawR = 0;
double encoderAngleL = 0, encoderAngleR = 0;  // Continuous angles in degrees

// ========== IMU GLOBALS ==========
int16_t gx, gy, gz;
double gyroBias = 0.0;

// ========== MOTION STATE ==========
bool motorRunning = false;
double heading = 0.0;  // Radians
int settledCount = 0;

// =====================================================
// Force motor pins safe at startup
// =====================================================
extern "C" void initVariant() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// =====================================================
// PID CLASS
// =====================================================
class PID {
  public:
    double kP, kI, kD;
    double error = 0;
    double prev_error = 0;
    double speed = 0;
    double total_error = 0;
    double derivative = 0;

    PID(double kp, double ki, double kd) : kP(kp), kI(ki), kD(kd) {}

    double calculate(double target, double current, double scale = 1.0) {
      error = target - current;
      total_error += ((error + prev_error) / 2.0);
      derivative = error - prev_error;
      speed = scale * (kP * error + kI * total_error + kD * derivative);
      prev_error = error;
      return speed;
    }

    void reset() {
      error = prev_error = speed = total_error = derivative = 0;
    }
};

// PID CONTROLLERS
PID heading_corrector(1.4, 0.0, 567);
PID turn_corrector(0.7, 0, 277);
PID distance_corrector(0.0045, 0, 0);

// =====================================================
// UTILITY FUNCTIONS
// =====================================================
double wrapAngle(double a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

// Convert meters to encoder ticks (for PID compatibility)
double metersToTicks(double meters) {
  const double WHEEL_DIAMETER_IN = 2.0;
  const double ENCODER_CPR = 13;  // Simulated CPR for compatibility
  const double GEAR_RATIO = 1.0;
  const double INCH_TO_M = 0.0254;

  double wheel_diameter_m = WHEEL_DIAMETER_IN * INCH_TO_M;
  double wheel_circumference_m = PI * wheel_diameter_m;

  double ticks_per_wheel_rev = ENCODER_CPR * GEAR_RATIO;
  double ticks_per_meter = ticks_per_wheel_rev / wheel_circumference_m;

  return meters * ticks_per_meter;
}

// Convert encoder degrees to equivalent ticks (for PID compatibility)
double degreesToTicks(double degrees) {
  // Original encoder: 13 CPR, so 360 degrees = 13 ticks
  return degrees * (13.0 / 360.0);
}

// =====================================================
// ENCODER FUNCTIONS (SPI Absolute Encoders)
// =====================================================
uint16_t readEncoderRaw(int csPin) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  uint8_t highByte = SPI.transfer(0x00);
  uint8_t lowByte = SPI.transfer(0x00);

  digitalWrite(csPin, HIGH);

  uint16_t raw = ((uint16_t)highByte << 8) | lowByte;
  return raw & 0x3FFF;  // 14-bit value
}

double encoderToDegrees(uint16_t val) {
  return (val / 16383.0) * 360.0;
}

void updateEncoderAngle(double raw, double &lastRaw, double &continuous) {
  double delta = raw - lastRaw;

  // Handle wraparound
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;

  // Deadband for noise
  if (fabs(delta) < 0.02) delta = 0;

  continuous += delta;
  lastRaw = raw;
}

void readEncoders() {
  uint16_t rawL = readEncoderRaw(CS_ENC_L);
  uint16_t rawR = readEncoderRaw(CS_ENC_R);

  double degL = encoderToDegrees(rawL);
  double degR = encoderToDegrees(rawR);

  updateEncoderAngle(degL, lastRawL, encoderAngleL);
  updateEncoderAngle(degR, lastRawR, encoderAngleR);
}

// Get encoder values in equivalent "ticks" for PID compatibility
double getEncoderL() {
  // Apply gear ratio (0.5) and convert to ticks
  return degreesToTicks(encoderAngleL * 0.5);
}

double getEncoderR() {
  // Flipped direction + gear ratio, then convert to ticks
  return degreesToTicks(-encoderAngleR * 0.5);
}

void resetEncoders() {
  encoderAngleL = 0;
  encoderAngleR = 0;
  // Re-initialize lastRaw values
  lastRawL = encoderToDegrees(readEncoderRaw(CS_ENC_L));
  lastRawR = encoderToDegrees(readEncoderRaw(CS_ENC_R));
}

// =====================================================
// IMU FUNCTIONS (I2C - MPU6050)
// =====================================================
void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);  // Gyro data register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// Get gyro Z in radians per second
double getGyroZ_rads() {
  readGyro();
  double gyroZ_dps = (gz - gyroBias) / 131.0;  // Convert to deg/sec (±250 dps range)
  return gyroZ_dps * (PI / 180.0);  // Convert to rad/sec
}

void calibrateGyro() {
  double sum = 0;
  const int samples = 1000;

  Serial.println("Calibrating gyro - keep robot still...");
  
  for (int i = 0; i < samples; i++) {
    readGyro();
    sum += gz;
    delay(2);
  }

  gyroBias = sum / samples;
}

void initIMU() {
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up (clear sleep bit)
  Wire.endTransmission(true);

  delay(100);

  // Optional: Set gyro range to ±250 dps (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 dps
  Wire.endTransmission(true);

  delay(10);
}

// =====================================================
// MOTOR CONTROL FUNCTIONS (H-bridge)
// =====================================================
void hardStopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// speed in [-1, 1]
void motorWriteL(double speed) {
  speed = constrain(speed, -1.0, 1.0);
  int pwm = abs(speed) * 255;

  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}

void motorWriteR(double speed) {
  speed = constrain(speed, -1.0, 1.0);
  int pwm = abs(speed) * 255;

  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, pwm);
}

// =====================================================
// DRIVE STRAIGHT FOR A GIVEN DISTANCE (METERS)
// =====================================================
void driveDistanceMeters(double meters, unsigned long timeoutMs, double speedScale) {
  heading_corrector.kP = 4.5;
  heading_corrector.kD = 467;
  Serial.println("=== driveDistanceMeters ===");

  const double TARGET_DISTANCE = metersToTicks(meters);

  heading_corrector.reset();
  distance_corrector.reset();
  settledCount = 0;

  const double targetHeading = heading;

  resetEncoders();

  unsigned long startTime = millis();
  unsigned long lastUpdateTime = millis();

  while (true) {
    unsigned long now = millis();
    double dt = (now - lastUpdateTime) / 1000.0;
    lastUpdateTime = now;

    // Timeout
    if (timeoutMs > 0 && (now - startTime) > timeoutMs) {
      Serial.println("STRAIGHT TIMEOUT");
      break;
    }

    // Update heading from IMU
    double gyroZ = getGyroZ_rads();
    heading += gyroZ * dt;

    // Read encoders (returns equivalent ticks)
    readEncoders();
    double avgDistance = (getEncoderL() + getEncoderR()) / 2.0;
    double distanceError = TARGET_DISTANCE - avgDistance;

    // Heading correction
    double headingCorrection = heading_corrector.calculate(targetHeading, heading, 1.0);

    // Base speed from distance PID
    double baseSpeed = distance_corrector.calculate(TARGET_DISTANCE, avgDistance, 1.0);
    baseSpeed = constrain(baseSpeed, MIN_BASE_SPEED, MAX_BASE_SPEED);
    baseSpeed *= speedScale;

    double leftSpeed = constrain(baseSpeed + headingCorrection, -1.0, 1.0);
    double rightSpeed = constrain(baseSpeed - headingCorrection, -1.0, 1.0);

    motorWriteL(leftSpeed);
    motorWriteR(rightSpeed);

    // Reduce gains near target
    if (fabs(distanceError) < 50) {
      heading_corrector.kP = 2.0;
      heading_corrector.kD = 0;
    }

    // Settle logic
    if (fabs(distanceError) < DISTANCE_ERROR_THRESHOLD) {
      settledCount++;
    } else {
      settledCount = 0;
    }

    // Debug output
    Serial.print("[STRAIGHT] H=");
    Serial.print(heading);
    Serial.print(" Hcorr=");
    Serial.print(headingCorrection);
    Serial.print(" base=");
    Serial.print(baseSpeed);
    Serial.print(" L=");
    Serial.print(leftSpeed);
    Serial.print(" R=");
    Serial.print(rightSpeed);
    Serial.print(" avg=");
    Serial.print(avgDistance);
    Serial.print("/");
    Serial.print(TARGET_DISTANCE);
    Serial.print(" settled=");
    Serial.println(settledCount);

    if (settledCount >= SETTLED_COUNT_THRESHOLD) {
      Serial.println("STRAIGHT COMPLETE");
      break;
    }
  }

  // Heading settle phase
  unsigned long settleStart = millis();
  heading_corrector.reset();
  unsigned long lastUpdateTime2 = millis();

  while (millis() - settleStart < HEADING_SETTLE_MS) {
    unsigned long now = millis();
    double dt = (now - lastUpdateTime2) / 1000.0;
    lastUpdateTime2 = now;

    double gyroZ = getGyroZ_rads();
    heading += gyroZ * dt;

    double headingError = targetHeading - heading;
    double correction = heading_corrector.calculate(targetHeading, heading, 1.0);
    correction = constrain(correction, -0.35, 0.35);

    motorWriteL(correction);
    motorWriteR(-correction);

    Serial.print("[SETTLE] H=");
    Serial.print(heading);
    Serial.print(" err=");
    Serial.println(headingError);

    delay(10);
  }

  motorWriteL(0);
  motorWriteR(0);
  delay(100);
}

// =====================================================
// TURN TO ABSOLUTE ANGLE (RADIANS) USING IMU
// =====================================================
void turnToAngle(double targetAngleRad, unsigned long timeoutMs, double speedScale) {
  Serial.println("=== turnToAngle() START ===");

  targetAngleRad = wrapAngle(targetAngleRad);

  turn_corrector.reset();
  turn_corrector.kP = 0.7;
  turn_corrector.kD = 101;
  turn_corrector.kI = 0;

  unsigned long startTime = micros();
  unsigned long lastTime = micros();

  while (true) {
    unsigned long now = micros();
    double dt = (now - lastTime) * 1e-6;
    lastTime = now;

    if (dt <= 0 || dt > 0.05) continue;

    // Timeout
    if (timeoutMs > 0 && (now - startTime) > timeoutMs * 1000UL) {
      Serial.println("turnToAngle() TIMEOUT");
      break;
    }

    // Update heading
    double gyroZ = getGyroZ_rads();
    heading = wrapAngle(heading + gyroZ * dt);

    // Wrapped angular error
    double angleError = wrapAngle(targetAngleRad - heading);

    // PID on error
    double correction = turn_corrector.calculate(0.0, -angleError);
    correction *= speedScale;
    correction = constrain(correction, -0.6, 0.6);

    // Static friction compensation
    if (fabs(angleError) > 0.05) {
      if (fabs(correction) < MIN_TURN_OUTPUT) {
        correction = copysign(MIN_TURN_OUTPUT, correction);
      }
    }

    motorWriteL(correction);
    motorWriteR(-correction);

    // Tighten gains near target
    if (fabs(angleError) < 0.15) {
      turn_corrector.kP = 6.5;
      turn_corrector.kD = 27.0 / 2.0;
    }

    // Debug
    Serial.print("[TURN] H=");
    Serial.print(heading);
    Serial.print(" tgt=");
    Serial.print(targetAngleRad);
    Serial.print(" err=");
    Serial.print(angleError);
    Serial.print(" out=");
    Serial.println(correction);

    // Exit condition
    if (fabs(angleError) < 0.03) {
      Serial.println("turnToAngle() COMPLETE");
      // break;  // Uncomment to exit on completion
    }
  }

  motorWriteL(0);
  motorWriteR(0);
  delay(150);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  hardStopMotors();

  delay(50);
  Serial.begin(115200);

  // GO button
  pinMode(GO, INPUT_PULLUP);

  // SPI for encoders
  SPI.begin();
  pinMode(CS_ENC_L, OUTPUT);
  pinMode(CS_ENC_R, OUTPUT);
  digitalWrite(CS_ENC_L, HIGH);
  digitalWrite(CS_ENC_R, HIGH);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  // Initialize encoder readings
  lastRawL = encoderToDegrees(readEncoderRaw(CS_ENC_L));
  lastRawR = encoderToDegrees(readEncoderRaw(CS_ENC_R));

  // Initialize IMU
  initIMU();

  heading = 0.0;

  Serial.println("System ready - press GO.");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  if (digitalRead(GO) == LOW && !motorRunning) {
    delay(50);
    if (digitalRead(GO) == LOW) {
      // Hard reset
      resetEncoders();
      heading = 0.0;
      heading_corrector.reset();
      turn_corrector.reset();
      distance_corrector.reset();

      motorRunning = true;
      Serial.println("GO pressed - starting calibration.");

      // Gyro calibration
      calibrateGyro();
      Serial.print("Gyro bias: ");
      Serial.println(gyroBias);

      heading = 0;

      delay(50);
      Serial.println("Calibration complete - Running sequence");

      // Movement parameters
      unsigned long driveTimeout = 7000;
      unsigned long turnTimeout = 1500;
      double driveScale = 1;
      double turnScale = 0.375;

      // === MOVEMENT SEQUENCE ===
      turnToAngle(-PI / 2.0, turnTimeout, turnScale);
      delay(50);

      driveDistanceMeters(0.85, 1000, 1);
      delay(50);

      turnToAngle(0, turnTimeout, turnScale);
      delay(50);

      driveDistanceMeters(7, driveTimeout, driveScale);
      delay(5);

      turnToAngle(PI / 2, turnTimeout, turnScale);
      delay(65);

      driveDistanceMeters(0.82, 1000, 1);

      Serial.println("Sequence complete.");
      motorRunning = false;

      // Wait for GO release
      while (digitalRead(GO) == LOW) {
        delay(10);
      }
      Serial.println("Ready for next GO.");
    }
  }
}