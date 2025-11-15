#define DIR_R 4
#define PWM_R 5
#define DIR_L 7
#define PWM_L 8

#define GO 0

#define ENC_L 6
#define ENC_R 3

#include <ICM20689.h>
#include <SPI.h>
#include <math.h>

#define CS_IMU 17

// ---------------------------------------------------------------------------
// Convert meters to encoder ticks
// ---------------------------------------------------------------------------
double metersToTicks(double meters)
{
    const double WHEEL_DIAMETER_IN = 2.0;
    const double ENCODER_CPR = 13.5;
    const double GEAR_RATIO = 1.0;

    const double INCH_TO_M = 0.0254;

    double wheel_diameter_m = WHEEL_DIAMETER_IN * INCH_TO_M;
    double wheel_circumference_m = PI * wheel_diameter_m;

    double ticks_per_wheel_rev = ENCODER_CPR * GEAR_RATIO;
    double ticks_per_meter = ticks_per_wheel_rev / wheel_circumference_m;

    return meters * ticks_per_meter;
}

// ---------------------------------------------------------------------------
// PID class
// ---------------------------------------------------------------------------
class PID {
  public:
    double kP, kI, kD;
    double error, prev_error, total_error, derivative;

    PID(double kp, double ki, double kd)
    : kP(kp), kI(ki), kD(kd),
      error(0), prev_error(0),
      total_error(0), derivative(0) {}

    double calculate(double target, double current, double scale = 1.0){
        error = target - current;
        total_error += (error + prev_error) * 0.5;
        derivative = error - prev_error;

        double output = scale * (kP*error + kI*total_error + kD*derivative);
        prev_error = error;
        return output;
    }

    void reset(){
        error = prev_error = total_error = derivative = 0;
    }
};

// ---------------------------------------------------------------------------
// Hardware + PID Globals
// ---------------------------------------------------------------------------
ICM20689 imu(SPI, CS_IMU);

PID heading_pid(2.1, 0, 1500);
PID distance_pid(0.005, 0, 0);

// USER-SET TOTAL DISTANCE (meters) — CHANGE THIS ONLY
const double TARGET_DISTANCE_M = 7.0;

// Lateral offset target (meters)
const double LATERAL_OFFSET_M = 0.5;

double ticksPerMeter = 0;

// Encoder counts
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

bool motorRunning = false;
unsigned long lastUpdate = 0;

double heading = 0.0;

void encoderISR_L(){ encoderCountL++; }
void encoderISR_R(){ encoderCountR++; }

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

// ---------------------------------------------------------------------------
// Motor write
// ---------------------------------------------------------------------------
void motorWriteL(double s){
    if(s > 0){
        analogWrite(PWM_L, (1.0 - s)*255.0);
        digitalWrite(DIR_L, HIGH);
    } else {
        analogWrite(PWM_L, (1.0 + s)*255.0);
        digitalWrite(DIR_L, LOW);
    }
}
void motorWriteR(double s){
    if(s > 0){
        analogWrite(PWM_R, (1.0 - s)*255.0);
        digitalWrite(DIR_R, LOW);
    } else {
        analogWrite(PWM_R, (1.0 + s)*255.0);
        digitalWrite(DIR_R, HIGH);
    }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup(){
    Serial.begin(4800);

    pinMode(GO, INPUT_PULLUP);

    pinMode(ENC_L, INPUT_PULLUP);
    pinMode(ENC_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

    hardStopMotors();
    delay(500);

    SPI.begin();
    int status = imu.begin();
    if(status < 0){
        Serial.println("IMU INIT FAILED");
        while(1);
    }
    imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);

    ticksPerMeter = metersToTicks(1.0);

    Serial.println("READY — press GO.");
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------
void loop(){

    // Start run
    if(digitalRead(GO) == LOW && !motorRunning){
        Serial.println("RUN START");

        encoderCountL = encoderCountR = 0;
        heading_pid.reset();
        distance_pid.reset();
        heading = 0.0;

        imu.readSensor();
        lastUpdate = millis();
        motorRunning = true;
    }

    if(!motorRunning) return;

    // --- Compute dt ---
    unsigned long now = millis();
    double dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    // --- Update IMU heading ---
    imu.readSensor();
    heading += imu.getGyroZ_rads() * dt;

    // --- Distance traveled ---
    long avgTicks = (encoderCountL + encoderCountR) / 2;
    double D_m = avgTicks / ticksPerMeter;   // meters traveled

    // --- Compute amplitude A dynamically ---
    //
    // A = (2π * lateralOffset) / totalDistance
    //
    double A = (2 * PI * LATERAL_OFFSET_M) / TARGET_DISTANCE_M;  
    if(A > 0.50) A = 0.50;   // clamp for safety (~28° max)

    // --- Compute S-curve heading target ---
    //
    // heading(D) = -A * sin(2π * (D / TotalDist))
    //
    double progress = D_m / TARGET_DISTANCE_M;
    if(progress > 1.0) progress = 1.0;

    double targetHeading = -A * sin(2 * PI * progress);

    // --- Compute base forward speed ---
    double targetDistTicks = metersToTicks(TARGET_DISTANCE_M);
    double baseSpeed = distance_pid.calculate(targetDistTicks, avgTicks);
    baseSpeed = constrain(baseSpeed, -0.2, 0.4);

    // --- Heading PID ---
    double turn = heading_pid.calculate(targetHeading, heading);

    double left = constrain(baseSpeed + turn, -0.2, 1.0);
    double right= constrain(baseSpeed - turn, -0.2, 1.0);

    motorWriteL(left);
    motorWriteR(right);

    // Debug
    Serial.print("D=");
    Serial.print(D_m);
    Serial.print("  prog=");
    Serial.print(progress);
    Serial.print("  A=");
    Serial.print(A);
    Serial.print("  targetH=");
    Serial.print(targetHeading);
    Serial.print("  head=");
    Serial.print(heading);
    Serial.print("  base=");
    Serial.print(baseSpeed);
    Serial.print("  turn=");
    Serial.print(turn);
    Serial.print("\n");

    // Stop when close enough
    if(fabs(targetDistTicks - avgTicks) < 1.0){
        motorWriteL(0);
        motorWriteR(0);
        motorRunning = false;
        Serial.println("FINISHED");
    }
}


// arc