#define DIR_R 4
#define PWM_R 5
#define DIR_L 7
#define PWM_L 8

#define GO 0

#define ENC_L 6
#define ENC_R 3

#include <ICM20689.h>
#include <SPI.h>

#define CS_IMU 17

// =====================
// DISTANCE PLACEHOLDERS
// =====================
const double D1_meters = 0.5;   // <<< FILL IN
const double D2_meters = 8.0;   // <<< FILL IN

// =====================
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

// =====================
// PID CLASS
// =====================
class PID {
  public:
    double kP, kI, kD;
    double error, prev_error, total_error, derivative, speed;

    PID(double kp, double ki, double kd):
      kP(kp), kI(ki), kD(kd),
      error(0), prev_error(0),
      total_error(0), derivative(0), speed(0) {}

    double calculate(double target, double current, double scale=1.0, bool H=false){
      error = target - current;

      total_error += (error + prev_error) / 2.0;
      if (prev_error * error < 0) total_error = 0;

      derivative = error - prev_error;
      speed = scale * (kP * error + kI * total_error + kD * derivative);
      prev_error = error;

      return speed;
    }

    void reset(){
      error = prev_error = total_error = derivative = speed = 0;
    }
};

ICM20689 imu(SPI, CS_IMU);

PID heading_corrector(0.7, 0, 900);
PID distance_corrector(0.0045, 0, 0.567);

// =====================
// ENCODERS
// =====================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

void encoderISR_L(){ encoderCountL++; }
void encoderISR_R(){ encoderCountR++; }

// =====================
// GLOBAL VARIABLES
// =====================
double heading = 0;
unsigned long lastUpdateTime = 0;

double D1_ticks = 0;
double D2_ticks = 0;

// =====================
// TURN SETTLE + TIMER
// =====================
unsigned long turnStartTime = 0;
unsigned long settleStartTime = 0;
bool settleRunning = false;

const unsigned long TURN_SETTLE_TIME = 150; // ms
const double TURN_REFINED_RANGE = 0.03;     // ~2°

double normalizeAngle(double a){
  while (a > PI) a -= 2*PI;
  while (a < -PI) a += 2*PI;
  return a;
}

double headingErrorAbs(double targetRad){
  return normalizeAngle(targetRad - heading);
}

// =====================
// MOTOR CONTROL
// =====================
void motorWriteL(double speed){
  if (speed > 0) {
    analogWrite(PWM_L, (1 - speed) * 255.0);
    digitalWrite(DIR_L, HIGH);
  } else {
    analogWrite(PWM_L, (1 + speed) * 255.0);
    digitalWrite(DIR_L, LOW);
  }
}

void motorWriteR(double speed){
  if (speed > 0) {
    analogWrite(PWM_R, (1 - speed) * 255.0);
    digitalWrite(DIR_R, LOW);
  } else {
    analogWrite(PWM_R, (1 + speed) * 255.0);
    digitalWrite(DIR_R, HIGH);
  }
}

// =====================
// TURN ABSOLUTE (WITH TIMEOUT + SETTLE)
// =====================
bool turn_abs(double targetDeg, int timeout_ms, double scale=1.0){
    double targetRad = targetDeg * PI / 180.0;

    heading_corrector.reset();
    turnStartTime = millis();
    settleStartTime = 0;
    settleRunning = false;

    while (true){
        imu.readSensor();
        double dt = (millis() - lastUpdateTime) / 1000.0;
        lastUpdateTime = millis();
        heading += imu.getGyroZ_rads() * dt;
        heading = normalizeAngle(heading);

        double error = headingErrorAbs(targetRad);

        double turnSpeed = heading_corrector.calculate(targetRad, heading, scale, true);
        turnSpeed = constrain(turnSpeed, -0.4, 0.4);

        motorWriteL(turnSpeed);
        motorWriteR(-turnSpeed);

        // settle logic
        if (fabs(error) <= TURN_REFINED_RANGE){
            if (!settleRunning){
                settleRunning = true;
                settleStartTime = millis();
            }
        } else {
            settleRunning = false;
        }

        // settled?
        if (settleRunning && millis() - settleStartTime >= TURN_SETTLE_TIME)
            break;

        // timeout?
        if (timeout_ms > 0 && millis() - turnStartTime >= timeout_ms)
            break;
    }

    motorWriteL(0);
    motorWriteR(0);
    return true;
}

bool turn_rel(double deltaDeg, int timeout_ms, double scale=1.0){
    double currentDeg = heading * 180.0 / PI;
    double target = currentDeg + deltaDeg;
    while (target >= 360) target -= 360;
    while (target < 0) target += 360;
    return turn_abs(target, timeout_ms, scale);
}

// =====================
// STRAIGHT DRIVE
// =====================
bool drive_to(double targetTicks){
    double avg = (encoderCountL + encoderCountR) / 2.0;
    double baseSpeed = distance_corrector.calculate(targetTicks, avg);
    baseSpeed = constrain(baseSpeed, -0.3, 0.4);

    double headingCorr = heading_corrector.calculate(0, heading);

    motorWriteL(baseSpeed + headingCorr);
    motorWriteR(baseSpeed - headingCorr);

    return fabs(targetTicks - avg) < 10;
}

// =====================
// STATE MACHINE
// =====================
enum State {
  IDLE,
  TURN_270,
  DRIVE1,
  TURN_0,
  DRIVE2,
  TURN_90,
  DRIVE3,
  DONE
};

State state = IDLE;

// =====================
// SETUP
// =====================
void setup(){
  Serial.begin(4800);

  D1_ticks = metersToTicks(D1_meters);
  D2_ticks = metersToTicks(D2_meters);

  pinMode(GO, INPUT_PULLUP);
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

  SPI.begin();
  imu.begin();
  imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);

  lastUpdateTime = millis();

  Serial.println("READY — Press GO");
}

// =====================
// LOOP
// =====================
void loop(){
  // IMU UPDATE
  imu.readSensor();
  double dt = (millis() - lastUpdateTime) / 1000.0;
  lastUpdateTime = millis();
  heading += imu.getGyroZ_rads() * dt;
  heading = normalizeAngle(heading);

  if (digitalRead(GO) == LOW && state == IDLE){
    encoderCountL = encoderCountR = 0;
    heading_corrector.reset();
    distance_corrector.reset();
    Serial.println("STARTING SEQUENCE");
    state = TURN_270;
    delay(500);
  }

  switch(state){

    case TURN_270:
      if (turn_abs(270, 3000)) {
        encoderCountL = encoderCountR = 0;
        state = DRIVE1;
      }
      break;

    case DRIVE1:
      if (drive_to(D1_ticks)){
        encoderCountL = encoderCountR = 0;
        distance_corrector.reset();
        state = TURN_0;
      }
      break;

    case TURN_0:
      if (turn_abs(0, 3000)){
        encoderCountL = encoderCountR = 0;
        state = DRIVE2;
      }
      break;

    case DRIVE2:
      if (drive_to(D2_ticks)){
        encoderCountL = encoderCountR = 0;
        state = TURN_90;
      }
      break;

    case TURN_90:
      if (turn_abs(90, 3000)){
        encoderCountL = encoderCountR = 0;
        state = DRIVE3;
      }
      break;

    case DRIVE3:
      if (drive_to(D1_ticks)){
        state = DONE;
      }
      break;

    case DONE:
      motorWriteL(0);
      motorWriteR(0);
      break;
  }
}
