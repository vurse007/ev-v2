#define DIR_R 4
#define PWM_R 5
#define DIR_L 7
#define PWM_L 8

#define GO 0

// === ENCODER PINS ===
#define ENC_L 6
#define ENC_R 3

#include <ICM20689.h>
#include <SPI.h>
#include <math.h>

#define CS_IMU 17

// LED PINS (same as Tektite)
#define LED_R 10
#define LED_G 11
#define LED_B 9

const double MIN_TURN_OUTPUT = 0.2;  // minimum torque to overcome static friction

// ================= RUN LED FLASH =================
bool ledFlashOn = false;
unsigned long lastLEDToggle = 0;
const unsigned long LED_FLASH_INTERVAL = 50; // ms





void LEDWrite(double r, double g, double b) {
  analogWrite(LED_R, r * 255.0);
  analogWrite(LED_G, g * 255.0);
  analogWrite(LED_B, b * 255.0);
}




extern "C" void initVariant() {
  // Force motor pins immediately at core init
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  digitalWrite(DIR_L, LOW);
  digitalWrite(PWM_L, LOW);
  digitalWrite(DIR_R, LOW);
  digitalWrite(PWM_R, LOW);

  // Claim PWM hardware early
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}



// =====================================================
// CONVERSION: meters → encoder ticks
// =====================================================
double metersToTicks(double meters)
{
    const double WHEEL_DIAMETER_IN = 2.0;
    const double ENCODER_CPR = 13;
    const double GEAR_RATIO = 1.0;
    const double INCH_TO_M = 0.0254;

    double wheel_diameter_m = WHEEL_DIAMETER_IN * INCH_TO_M;
    double wheel_circumference_m = PI * wheel_diameter_m;

    double ticks_per_wheel_rev = ENCODER_CPR * GEAR_RATIO;
    double ticks_per_meter = ticks_per_wheel_rev / wheel_circumference_m;

    return meters * ticks_per_meter;
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

    PID(double kp, double ki, double kd): kP(kp), kI(ki), kD(kd) {}

    double calculate(double target, double current, double scale=1.0){
      error = target - current;
      total_error += ((error + prev_error) / 2.0);
      derivative = error - prev_error;
      speed = scale * (kP * error + kI * total_error + kD * derivative);
      prev_error = error;
      return speed;
    }

    void reset(){
      error = prev_error = speed = total_error = derivative = 0;
    }
};

ICM20689 imu(SPI, CS_IMU);

// PID CONTROLLERS
PID heading_corrector(0, 0, 0);
PID turn_corrector(0, 0, 0);
PID distance_corrector(0.0045, 0, 0);

double wrapAngle(double a) {
    while (a >  PI) a -= 2*PI;
    while (a < -PI) a += 2*PI;
    return a;
}


// Global motion state
bool motorRunning = false;

// Heading from IMU integration (radians)
double heading = 0.0;

// Encoders
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// Speed limits for driving
const double MIN_BASE_SPEED = -0.2;
const double MAX_BASE_SPEED =  0.4;

// Distance settle threshold
const double DISTANCE_ERROR_THRESHOLD = 1.0;
const int SETTLED_COUNT_THRESHOLD = 2;
int settledCount = 0;
// Gyro drift correction
double gyroBias = 0.0;


// =====================================================
// ENCODER INTERRUPTS
// =====================================================
void encoderISR_L() {
  encoderCountL++;
}

void encoderISR_R() {
  encoderCountR++;
}

// =====================================================
// MOTOR CONTROL HELPERS
// =====================================================
void hardStopMotors() {
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  digitalWrite(DIR_R, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(DIR_L, LOW);
  digitalWrite(PWM_L, LOW);
}

// speed in [-1, 1]
void motorWriteL(double speed) {
  speed = constrain(speed, -1.0, 1.0);

  if (speed >= 0) {
    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_L, (1.0 - speed) * 255.0);
  } else {
    digitalWrite(DIR_L, LOW);
    analogWrite(PWM_L, (1.0 + speed) * 255.0);
  }
}

void motorWriteR(double speed) {
  speed = constrain(speed, -1.0, 1.0);

  if (speed >= 0) {
    digitalWrite(DIR_R, LOW);
    analogWrite(PWM_R, (1.0 - speed) * 255.0);
  } else {
    digitalWrite(DIR_R, HIGH);
    analogWrite(PWM_R, (1.0 + speed) * 255.0);
  }
}


const unsigned long HEADING_SETTLE_MS = 500;  // 1 second

// =====================================================
// DRIVE STRAIGHT FOR A GIVEN DISTANCE (METERS)
// WITH TIMEOUT & SPEED SCALE
// =====================================================
void driveDistanceMeters(double meters, unsigned long timeoutMs, double speedScale)
{
  heading_corrector.kP = 2.1;
  heading_corrector.kD = 400;
  Serial.println("=== driveDistanceMeters (OLD LOGIC) ===");

  const double TARGET_DISTANCE = metersToTicks(meters);

  heading_corrector.reset();
  distance_corrector.reset();
  settledCount = 0;

  // LOCK ABSOLUTE HEADING (but keep global heading updating!)
  const double targetHeading = heading;

  encoderCountL = 0;
  encoderCountR = 0;

  unsigned long startTime = millis();
  unsigned long lastUpdateTime = millis();

  while (true) {

    flashPurpleIfRunning();

    unsigned long now = millis();
    double dt = (now - lastUpdateTime) / 1000.0;
    lastUpdateTime = now;

    // Timeout
    if (timeoutMs > 0 && (now - startTime) > timeoutMs) {
      Serial.println("STRAIGHT TIMEOUT");
      break;
    }

    // === UPDATE GLOBAL HEADING ===
    imu.readSensor();
    double gyroZ = imu.getGyroZ_rads() - gyroBias;
    heading += gyroZ * dt;

    // === DISTANCE ===
    double avgDistance = (encoderCountL + encoderCountR) / 2.0;
    double distanceError = TARGET_DISTANCE - avgDistance;

    // === HEADING CORRECTION (IDENTICAL TO OLD CODE) ===
    double headingCorrection =
        heading_corrector.calculate(targetHeading, heading, 1.0);

    // === BASE SPEED ===
    double baseSpeed =
        distance_corrector.calculate(TARGET_DISTANCE, avgDistance, 1.0);

    baseSpeed = constrain(baseSpeed, MIN_BASE_SPEED, MAX_BASE_SPEED);
    baseSpeed *= speedScale;

    double leftSpeed  = constrain(baseSpeed + headingCorrection, -1.0, 1.0);
    double rightSpeed = constrain(baseSpeed - headingCorrection, -1.0, 1.0);

    motorWriteL(leftSpeed);
    motorWriteR(rightSpeed);

    if (fabs(distanceError) < 50){
      heading_corrector.kP = 2.0;
      heading_corrector.kD = 0;
    }
    else if (fabs(distanceError) > (TARGET_DISTANCE - 50)){
      heading_corrector.kP = 6.0;
    }
    else{
      heading_corrector.kP = 5.0;
      heading_corrector.kD = 400;
    }

    if (fabs(targetHeading - heading) < 0.05){
      heading_corrector.kP = 4.0;
    }

    // === SETTLE LOGIC (UNCHANGED) ===
    if (fabs(distanceError) < DISTANCE_ERROR_THRESHOLD) {
      settledCount++;
    } else {
      settledCount = 0;
    }

    // DEBUG
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
  

    // ================================
  // HEADING SETTLE PHASE
  // ================================
  unsigned long settleStart = millis();
  heading_corrector.reset();
  startTime = millis();
  lastUpdateTime = millis();

  while (millis() - settleStart < HEADING_SETTLE_MS) {

    unsigned long now = millis();
    double dt = (now - lastUpdateTime) / 1000.0;
    lastUpdateTime = now;

    imu.readSensor();
    double gyroZ = imu.getGyroZ_rads() - gyroBias;
    heading += gyroZ * dt;  // small dt assumption (OK here)

    double headingError = targetHeading - heading;

    double correction =
        heading_corrector.calculate(targetHeading, heading, 1.0);
    // Clamp for safety
    correction = constrain(correction, -0.35, 0.35);

    motorWriteL( correction);
    motorWriteR(-correction);

    Serial.print("[SETTLE] H=");
    Serial.print(heading);
    Serial.print(" err=");
    Serial.println(headingError);

    delay(10);
  }

  // Full stop
  motorWriteL(0);
  motorWriteR(0);
  delay(100);
}


// =====================================================
// TURN TO ABSOLUTE ANGLE (RADIANS) USING IMU
// WITH TIMEOUT & SPEED SCALE
// =====================================================
void turnToAngle(double targetAngleRad,
                 unsigned long timeoutMs,
                 double speedScale)
{
  Serial.println("=== turnToAngle() START ===");

  // --- ALWAYS wrap target ---
  targetAngleRad = wrapAngle(targetAngleRad);

  // --- Reset controller ---
  turn_corrector.reset();
  turn_corrector.kP = 0.7;
  turn_corrector.kD = 167;
  turn_corrector.kI = 0;

  unsigned long startTime = micros();
  unsigned long lastTime  = micros();

  while (true) {
    unsigned long now = micros();
    double dt = (now - lastTime) * 1e-6;
    lastTime = now;

    // Protect integration
    if (dt <= 0 || dt > 0.05) continue;

    // Timeout
    if (timeoutMs > 0 && (now - startTime) > timeoutMs * 1000UL) {
      Serial.println("turnToAngle() TIMEOUT");
      break;
    }

    // --- Update heading ---
    imu.readSensor();
    double gyroZ = imu.getGyroZ_rads() - gyroBias;
    heading = wrapAngle(heading + gyroZ * dt);

    // --- WRAPPED ANGULAR ERROR ---
    double angleError = wrapAngle(targetAngleRad - heading);

    // --- PID on ERROR (not angle!) ---
    double correction = turn_corrector.calculate(0.0, -angleError);
    correction *= speedScale;

    // Clamp max turn power
    correction = constrain(correction, -0.6, 0.6);

    // =====================================================
    // STATIC FRICTION COMPENSATION (MIN TURN OUTPUT)
    // =====================================================
    if (fabs(angleError) > 0.05) {                 // still meaningfully off target
      if (fabs(correction) < MIN_TURN_OUTPUT) {    // not enough torque to move
        correction = copysign(MIN_TURN_OUTPUT, correction);
      }
    }


    motorWriteL( correction);
    motorWriteR(-correction);

    //Tighten gains near target
    if (fabs(angleError) < 0.05) {
      turn_corrector.kP = 7.5;
      turn_corrector.kD = 0;
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

    // --- EXIT CONDITION ---
    if (fabs(angleError) < 0.03) {
      Serial.println("turnToAngle() COMPLETE");
      // break;
    }
  }

  motorWriteL(0);
  motorWriteR(0);
  delay(150);
}

inline void flashPurpleIfRunning() {
  if (!motorRunning) return;

  unsigned long now = millis();
  if (now - lastLEDToggle >= LED_FLASH_INTERVAL) {
    lastLEDToggle = now;
    ledFlashOn = !ledFlashOn;

    if (ledFlashOn) {
      LEDWrite(0.6, 0.0, 0.6);  // 💜 purple
    } else {
      LEDWrite(0.0, 0.0, 0.0);  // off
    }
  }
}
// =====================================================
// SETUP
// =====================================================
void setup() {

    // FORCE motor pins into a known safe state IMMEDIATELY
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  digitalWrite(DIR_L, LOW);
  digitalWrite(PWM_L, LOW);
  digitalWrite(DIR_R, LOW);
  digitalWrite(PWM_R, LOW);

  delay(50);  // let motor driver settle
  Serial.begin(4800);

  // LED setup
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  LEDWrite(0.1, 0.1, 0.1);  // dim white idle

  pinMode(GO, INPUT_PULLUP);

  // Encoder setup
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

  hardStopMotors();
  delay(500);

  motorWriteL(0.0);
  motorWriteR(0.0);

  SPI.begin();

  int status = imu.begin();
  if (status < 0) {
    LEDWrite(1, 0, 0);    // 🔴 red = IMU failure
    Serial.println("IMU INIT FAILED");
    while (1);
  }

  status = imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);
  if (status < 0) {
    LEDWrite(1, 0, 0);    // 🔴 red = IMU failure
    Serial.println("GYRO CONFIG FAILED");
    while (1);
  }

  // No calibration here now
  heading = 0.0;

  Serial.println("System ready — press GO.");
}



// =====================================================
// MAIN LOOP: SEQUENCE
// =====================================================
void loop() {


    if (digitalRead(GO) == LOW && !motorRunning) {
    delay(50);
    if (digitalRead(GO) == LOW) {

      // ===== HARD POST-BOOT RESET =====
      encoderCountL = 0;
      encoderCountR = 0;
      heading = 0.0;
      heading_corrector.reset();
      turn_corrector.reset();
      distance_corrector.reset();


        motorRunning = true;
        Serial.println("GO pressed → starting calibration.");

        // ----- GYRO CALIBRATION (max 2.5 seconds) -----
        LEDWrite(1, 1, 0);  // yellow = calibration start

        double sum = 0;
        int samples = 1;
        unsigned long startCal = millis();

        // while (millis() - startCal < 1000) {  // max 2.5 seconds
        //     imu.readSensor();
        //     sum += imu.getGyroZ_rads();
        //     samples++;

        //     // Flash LED yellow
        //     if ((millis() / 150) % 2 == 0)
        //         LEDWrite(1, 1, 0);
        //     else
        //         LEDWrite(0.3, 0.3, 0);

        //     delay(3);
        // }

        gyroBias = sum / samples;

        LEDWrite(0, 1, 0); // green = calibration done
        Serial.print("Gyro bias: ");
        Serial.println(gyroBias);

        heading = 0; // reset heading after calibration

        delay(50); // slight buffer before movement
        Serial.println("Calibration complete → Running sequence");


      // Example distances & timeouts — tweak these:
      double driveDist = 0.5;          // meters
      double targetDist = 9.0;
      unsigned long driveTimeout = 7000;  // ms
      unsigned long turnTimeout  = 1500;  // ms

      double driveScale = 1;   // 80% of tuned speed
      double turnScale  = 0.375;   // 60% speed for turns

      double gyro[3];
      


      turnToAngle(-PI/2.0,turnTimeout,turnScale);
      delay(50);

      driveDistanceMeters(0.85, 1000, 1);
      delay(50);

      ///* REMOVE LATER */ turnToAngle(PI/2, turnTimeout, turnScale); turnToAngle(PI-0.00000001,turnTimeout,turnScale); driveDistanceMeters(0.85,1000,1);


      turnToAngle(0,turnTimeout,turnScale);
      delay(50);

      driveDistanceMeters(7, driveTimeout, driveScale); // 9.1 too much
      delay(5);


      turnToAngle(PI/2,turnTimeout,turnScale);

      delay(65);

      driveDistanceMeters(0.82, 1000, 1);

      LEDWrite(0.1, 0.1, 0.1);  // back to idle white



      // while(1){
      //   imu.readSensor();

      //   imu.readGyro(gyro); 
        
      //   Serial.print(gyro[2]); 
        
      //   Serial.println();
      // }
      



    //   // 1) Turn to -90° (west)
    //   turnToAngle(-PI/2.0, turnTimeout, turnScale);
    //   delay(1000);
    //   // 2) Drive forward
    //   driveDistanceMeters(driveDist, driveTimeout, driveScale);
    //   delay(1000);
    //   // 3) Turn to 0° (north)
    //   turnToAngle(0.0, turnTimeout, turnScale);
    //   delay(1000);

    //   // 4) Drive forward (target)
    //   driveDistanceMeters(targetDist, driveTimeout, driveScale);
      //delay(1000);

      // 5) Turn to +90° (east)
      //turnToAngle(PI / 2.0, turnTimeout, turnScale);



      // 6) Drive forward
      //driveDistanceMeters(driveDist, driveTimeout, driveScale);

      Serial.println("Sequence complete.");
      motorRunning = false;

    

      // Wait for GO to be released to avoid retriggering
      while (digitalRead(GO) == LOW) {
        delay(10);
      }
      Serial.println("Ready for next GO.");
    }
  }
}

