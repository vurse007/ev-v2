// MACROS FOR PINS
// ------
#define DIR_R 4
#define PWM_R 5
#define DIR_L 7
#define PWM_L 8
// Encoders
#define ENC_L 6
#define ENC_R 3
// GO button
#define GO 0
// LED pins
#define LED_R 10
#define LED_G 11
#define LED_B 9
// IMU
#define CS_IMU 17


//INCLUDES
#include <math.h>
#include <float.h>
#include <string>
#include <SPI.h>
#include <ICM20689.h>


//like a folder we can keep minimized
namespace setup {
    // SENSING VALUES
    // IMU
    ICM20689 imu(SPI, CS_IMU);
    double heading = 0.0;
    double gyroBias = 0.0;
    // Motion state
    bool motorRunning = false;
    double distanceError = 0.0;
    bool* distanceRefined = false;
    // Encoders
    volatile long encoderCountL = 0;
    volatile long encoderCountR = 0;


    // LED
    void LEDWrite(double r, double g, double b) {
        analogWrite(LED_R, r * 255.0);
        analogWrite(LED_G, g * 255.0);
        analogWrite(LED_B, b * 255.0);
    }


    // METERS TO TICKS
    double metersToTicks(double meters) {
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

    // REQUIRED CONSTANTS + PID
    const double MIN_TURN_OUTPUT = 0.2;
    const double MIN_BASE_SPEED = -0.2;
    const double MAX_BASE_SPEED =  0.4;

    class PID { 
    public:
        struct constants {
            double kP, kI, kD;
        };
        constants general_constants = {0,0,0};
        constants refined_constants = {0,0,0};
        bool distanceRefined* = false;
        double error = 0;
        double prev_error = 0;
        double total_error = 0;

        PID(constants gc, constants rr, bool* dR): general_constants(gc), refined_constants(rr), distanceRefined(dR) {}

        double calculate(double target, double current) {
            error = target - current;
            total_error += ((error + prev_error) / 2.0);
            double derivative = error - prev_error;
            prev_error = error;
            if (fabs(error) < range){
                return refined_constants.kP * error + refined_constants.kI * total_error + refined_constants.kD * derivative;
            }
            return general_constants.kP * error + general_constants.kI * total_error + general_constants.kD * derivative;
        }

        void reset() {
            error = prev_error = total_error = 0;
        }
    };



    // ENCODER INTERRUPTS
    void encoderISR_L() { encoderCountL++; }
    void encoderISR_R() { encoderCountR++; }



    // MOTOR CONTROL
    void hardStopMotors() {
        digitalWrite(DIR_R, LOW);
        digitalWrite(PWM_R, LOW);
        digitalWrite(DIR_L, LOW);
        digitalWrite(PWM_L, LOW);
    }
    void motorWriteL(double speed) {
        speed = constrain(speed, -1.0, 1.0);
        if (speed > 0) {
            analogWrite(PWM_L, (1.0 - speed) * 255.0);
            digitalWrite(DIR_L, HIGH);
        } else {
            analogWrite(PWM_L, (1.0 + speed) * 255.0);
            digitalWrite(DIR_L, LOW);
        }
    }
    void motorWriteR(double speed) {
        speed = constrain(speed, -1.0, 1.0);
        if (speed > 0) {
            analogWrite(PWM_R, (1.0 - speed) * 255.0);
            digitalWrite(DIR_R, LOW);
        } else {
            analogWrite(PWM_R, (1.0 + speed) * 255.0);
            digitalWrite(DIR_R, HIGH);
        }
    }


    // IMU INIT
    void calibrate_intense(){
        // ----- GYRO CALIBRATION -----
        double sum = 0;
        int samples = 0;
        unsigned long startCal = millis();

        while (millis() - startCal < 2500) {
        imu.readSensor();
        sum += imu.getGyroZ_rads();
        samples++;

        if ((millis() / 150) % 2 == 0)
            LEDWrite(1, 1, 0);
        else
            LEDWrite(0.3, 0.3, 0);

        delay(3);
        }

        gyroBias = sum / samples;
        heading = 0;
        LEDWrite(0, 1, 0);
    }


}

namespace calculator {
    double derivative(double (*func)(double), double x, std::string h = "ag"){
        long long double dx=0;
        if (h == "ag"){
            dx=sqrt(DBL_EPSILON) * fmax(fabs(x),1.0);
        }
        else {
            dx=stoll(h);
        }
        return (func(x+dx) - func(x-dx) / (2*dx));
    }

    // Adaptive arc length
    double arcLengthAdaptive(double (*func)(double), double a, double b, double baseStep = 0.01) {
        double length = 0.0;
        double x = a;

        while (x < b) {
            //derivative at current x
            double dydx = derivative(func, x);

            //adaptive dx
            double dx = baseStep / sqrt(1 + dydx * dydx);

            //prevent overshoot past b
            if (x + dx > b) dx = b - x;

            // find derivative at x+dx
            double dydx_next = derivative(func, x + dx);

            //calc arc length using trapezoid rule
            double ds = 0.5 * dx * (sqrt(1 + dydx * dydx) + sqrt(1 + dydx_next * dydx_next));
            length += ds;

            //move to next x
            x += dx;
        }

        return length;
    }
}

using namespace setup;
using namespace calculator;

void sech(double x){
    return 1/cosh(x);
}
void CURVE(double x){
    if (x >= 0.0 && x <= 2.67) {
        return pow(sech(x - 2.67), 2);   // sech^2(x - 2.67)
    } 
    else if (x > 2.67 && x < 7.33) {
        return 1.0;                       // constant 1
    } 
    else if (x >= 7.33 && x <= 10.0) {
        return pow(sech(x - 7.33), 2);   // sech^2(x - 7.33)
    } 
    else {
        return 0.0;                       // outside domain
    }
}

void setup() {
    Serial.begin(4800);

    //pid objects
    PID headingPID({2.1,0,400}, {2.1,0,400}, 0, );
    PID angularPID({0.7,0,101}, {6.5,0,10});
    PID drivePID({0.0045,0,0}, {1,0,0});

    // =========================
    // MOTOR PINS
    // =========================
    pinMode(DIR_L, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(DIR_R, OUTPUT);
    pinMode(PWM_R, OUTPUT);

    hardStopMotors();
    delay(500);

    motorWriteL(0.0);
    motorWriteR(0.0);

    // =========================
    // LED PINS
    // =========================
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    LEDWrite(0.1, 0.1, 0.1);

    // =========================
    // GO BUTTON
    // =========================
    pinMode(GO, INPUT_PULLUP);

    // =========================
    // ENCODERS
    // =========================
    pinMode(ENC_L, INPUT_PULLUP);
    pinMode(ENC_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

    // =========================
    // IMU (SPI)
    // =========================
    SPI.begin();

    int status = imu.begin();
    if (status < 0) {
        LEDWrite(1, 0, 0);
        Serial.println("IMU INIT FAILED");
        while (1);
    }

    status = imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);
    if (status < 0) {
        LEDWrite(1, 0, 0);
        Serial.println("GYRO CONFIG FAILED");
        while (1);
    }

    // =========================
    // INITIAL STATE
    // =========================
    heading = 0.0;
    gyroBias = 0.0;
    motorRunning = false;

    //calculations
    distanceError = arcLengthAdaptive(CURVE, 0.0, 10.0);

    Serial.println("System ready — press GO.");
}


void loop() {
    // go button
    if (digitalRead(GO) == LOW){
        motorRunning = true;

        // movement init
        encoderCountL=0;
        encoderCountR=0;

        heading = 0.0;
        calibrate_intense();


        // movement here


        motorWriteL(0.0);
        motorWriteR(0.0);
        motorRunning = false;

        while (digitalRead(GO) == LOW){
            delay(10);
            //prevent rapid press registering
        }
    }
}