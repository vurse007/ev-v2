// #define DIR_R 4
// #define PWM_R 5
// #define DIR_L 7
// #define PWM_L 8

// #define GO 0

// // === ENCODER PINS ===
// #define ENC_L 6
// #define ENC_R 3

// #include <ICM20689.h>
// #include <SPI.h>
// #include <math.h>

// #define CS_IMU 17

// // LED PINS (same as Tektite)
// #define LED_R 10
// #define LED_G 11
// #define LED_B 9


// void LEDWrite(double r, double g, double b) {
//   analogWrite(LED_R, r * 255.0);
//   analogWrite(LED_G, g * 255.0);
//   analogWrite(LED_B, b * 255.0);
// }


// // =====================================================
// // CONVERSION: meters → encoder ticks
// // =====================================================
// double metersToTicks(double meters)
// {
//     const double WHEEL_DIAMETER_IN = 2.0;
//     const double ENCODER_CPR = 13;
//     const double GEAR_RATIO = 1.0;
//     const double INCH_TO_M = 0.0254;

//     double wheel_diameter_m = WHEEL_DIAMETER_IN * INCH_TO_M;
//     double wheel_circumference_m = PI * wheel_diameter_m;

//     double ticks_per_wheel_rev = ENCODER_CPR * GEAR_RATIO;
//     double ticks_per_meter = ticks_per_wheel_rev / wheel_circumference_m;

//     return meters * ticks_per_meter;
// }

// // =====================================================
// // PID CLASS
// // =====================================================
// class PID { 
//   public:

//     double kP, kI, kD;
//     double error = 0;
//     double prev_error = 0;
//     double speed = 0;
//     double total_error = 0;
//     double derivative = 0;

//     PID(double kp, double ki, double kd): kP(kp), kI(ki), kD(kd) {}

//     double calculate(double target, double current, double scale=1.0){
//       error = target - current;
//       total_error += ((error + prev_error) / 2.0);
//       derivative = error - prev_error;
//       speed = scale * (kP * error + kI * total_error + kD * derivative);
//       prev_error = error;
//       return speed;
//     }

//     void reset(){
//       error = prev_error = speed = total_error = derivative = 0;
//     }
// };

// ICM20689 imu(SPI, CS_IMU);

// // PID CONTROLLERS
// PID heading_corrector(0.7, 0, 0);
// PID turn_corrector(0.7, 0, 101);
// PID distance_corrector(0.0045, 0, 0);

// double wrapAngle(double a) {
//     while (a >  PI) a -= 2*PI;
//     while (a < -PI) a += 2*PI;
//     return a;
// }


// // Global motion state
// bool motorRunning = false;

// // Heading from IMU integration (radians)
// double heading = 0.0;

// // Encoders
// volatile long encoderCountL = 0;
// volatile long encoderCountR = 0;

// // Speed limits for driving
// const double MIN_BASE_SPEED = -0.2;
// const double MAX_BASE_SPEED =  0.4;

// // Distance settle threshold
// const double DISTANCE_ERROR_THRESHOLD = 1.0;
// const int SETTLED_COUNT_THRESHOLD = 2;
// int settledCount = 0;
// // Gyro drift correction
// double gyroBias = 0.0;


// // =====================================================
// // ENCODER INTERRUPTS
// // =====================================================
// void encoderISR_L() {
//   encoderCountL++;
// }

// void encoderISR_R() {
//   encoderCountR++;
// }

// // =====================================================
// // MOTOR CONTROL HELPERS
// // =====================================================
// void hardStopMotors() {
//   pinMode(DIR_R, OUTPUT);
//   pinMode(PWM_R, OUTPUT);
//   pinMode(DIR_L, OUTPUT);
//   pinMode(PWM_L, OUTPUT);

//   digitalWrite(DIR_R, LOW);
//   digitalWrite(PWM_R, LOW);
//   digitalWrite(DIR_L, LOW);
//   digitalWrite(PWM_L, LOW);
// }

// // speed in [-1, 1]
// void motorWriteL(double speed) {
//   speed = constrain(speed, -1.0, 1.0);
//   if (speed > 0) {
//     analogWrite(PWM_L, (1.0 - speed) * 255.0);
//     digitalWrite(DIR_L, HIGH);
//   } else {
//     analogWrite(PWM_L, (1.0 + speed) * 255.0);
//     digitalWrite(DIR_L, LOW);
//   }
// }

// // speed in [-1, 1]
// void motorWriteR(double speed) {
//   speed = constrain(speed, -1.0, 1.0);
//   if (speed > 0) {
//     analogWrite(PWM_R, (1.0 - speed) * 255.0);
//     digitalWrite(DIR_R, LOW);
//   } else {
//     analogWrite(PWM_R, (1.0 + speed) * 255.0);
//     digitalWrite(DIR_R, HIGH);
//   }
// }

// // =====================================================
// // DRIVE STRAIGHT FOR A GIVEN DISTANCE (METERS)
// // WITH TIMEOUT & SPEED SCALE
// // =====================================================
// void driveDistanceMeters(double meters, unsigned long timeoutMs, double speedScale) {
//   Serial.println("=== driveDistanceMeters() START ===");
//   Serial.print("Target meters: "); Serial.println(meters);
//   Serial.print("Timeout (ms): "); Serial.println(timeoutMs);
//   Serial.print("Speed scale: "); Serial.println(speedScale);

//   double targetTicks = metersToTicks(meters);

//   distance_corrector.reset();
//   heading_corrector.reset();
//   settledCount = 0;

//   // Do NOT reset heading; we want absolute heading across the run.
//   // But we DO lock the starting heading for this segment:
//   double startHeading = heading;

//   // Reset encoders
//   encoderCountL = 0;
//   encoderCountR = 0;

//   unsigned long startTime = micros();
//   unsigned long last = micros();

//   while (true) {
//     unsigned long long now = micros();
//     double dt = (now - last) / 1000000;
//     last = now;

//     if (dt <= 0 || dt > 0.05) {
//     return;   // skip update, don't integrate
//     }


//     // Timeout check (0 = no timeout)
//     if (timeoutMs > 0 && (now - startTime) > timeoutMs) {
//       Serial.println("driveDistanceMeters() TIMEOUT → stopping and continuing sequence.");
//       break;
//     }

//     // === UPDATE HEADING FROM IMU ===
//     imu.readSensor();
//     double gyroZ = imu.getGyroZ_rads() - gyroBias;

//     heading += gyroZ * dt;
//     heading = wrapAngle(heading);

//     // === ENCODER DISTANCE ===
//     double avgTicks = (encoderCountL + encoderCountR) / 2.0;
//     double distanceError = targetTicks - avgTicks;

//     // === PID FOR DISTANCE (BASE SPEED) ===
//     double baseSpeed = distance_corrector.calculate(targetTicks, avgTicks);
//     baseSpeed = constrain(baseSpeed, MIN_BASE_SPEED, MAX_BASE_SPEED);

//     // Apply speed scaling
//     baseSpeed *= speedScale;

//     // === PID FOR HEADING CORRECTION ===
//     double headingCorrection = heading_corrector.calculate(startHeading, heading);
//     headingCorrection *= speedScale;

//     double leftSpeed  = baseSpeed + headingCorrection;
//     double rightSpeed = baseSpeed - headingCorrection;

//     leftSpeed  = constrain(leftSpeed,  -1.0, 1.0);
//     rightSpeed = constrain(rightSpeed, -1.0, 1.0);

//     motorWriteL(leftSpeed);
//     motorWriteR(rightSpeed);

//     // === SETTLE LOGIC ===
//     if (fabs(distanceError) < DISTANCE_ERROR_THRESHOLD) {
//       settledCount++;
//     } else {
//       settledCount = 0;
//     }

//     // DEBUG
//     Serial.print("[DRIVE] H: ");
//     Serial.print(heading);
//     Serial.print(" rad, Hcorr: ");
//     Serial.print(headingCorrection);
//     Serial.print(", base: ");
//     Serial.print(baseSpeed);
//     Serial.print(", L: ");
//     Serial.print(leftSpeed);
//     Serial.print(", R: ");
//     Serial.print(rightSpeed);
//     Serial.print(", EncL: ");
//     Serial.print(encoderCountL);
//     Serial.print(", EncR: ");
//     Serial.print(encoderCountR);
//     Serial.print(", avgTicks: ");
//     Serial.print(avgTicks);
//     Serial.print(", targetTicks: ");
//     Serial.print(targetTicks);
//     Serial.print(", err: ");
//     Serial.print(distanceError);
//     Serial.print(", settled: ");
//     Serial.println(settledCount);

//     if (settledCount >= SETTLED_COUNT_THRESHOLD) {
//       Serial.println("driveDistanceMeters() target reached and settled.");
//       break;
//     }
//   }

//   motorWriteL(0.0);
//   motorWriteR(0.0);
//   delay(200);
//   Serial.println("=== driveDistanceMeters() END ===");
// }

// // =====================================================
// // TURN TO ABSOLUTE ANGLE (RADIANS) USING IMU
// // WITH TIMEOUT & SPEED SCALE
// // =====================================================
// void turnToAngle(double targetAngleRad, unsigned long timeoutMs, double speedScale) {
//   Serial.println("=== turnToAngle() START ===");
//   Serial.print("Target angle (rad): "); Serial.println(targetAngleRad);
//   Serial.print("Timeout (ms): "); Serial.println(timeoutMs);
//   Serial.print("Speed scale: "); Serial.println(speedScale);

//   turn_corrector.reset();

//   turn_corrector.kP = 0.7;
//   turn_corrector.kD = 101;

//   unsigned long long startTime = micros();
//   unsigned long long last = micros();

//   while (true) {
//     unsigned long long now = micros();
//     double dt = (now - last) / 1000000.0;
//     last = now;

//     // Timeout check
//     if (timeoutMs > 0 && (now - startTime) > timeoutMs * 1000) {
//       Serial.println("turnToAngle() TIMEOUT → stopping and continuing sequence.");
//       break;
//     }


    


//     // Update heading from IMU
//     imu.readSensor();
//     double gyroZ = imu.getGyroZ_rads() - gyroBias;

//     heading += gyroZ * dt;
//     heading = wrapAngle(heading);

    

//     // PID correction toward target angle
//     double correction = turn_corrector.calculate(targetAngleRad, heading);

//     // Apply speed scaling
//     correction *= speedScale;

    

//     // Turn in place: wheels opposite directions
//     double leftSpeed  =  correction;
//     double rightSpeed = -correction;

//     leftSpeed  = constrain(leftSpeed,  -0.6, 0.6);
//     rightSpeed = constrain(rightSpeed, -0.6, 0.6);

//     motorWriteL(leftSpeed);
//     motorWriteR(rightSpeed);

//     double angleError = targetAngleRad - heading;

//     if (fabs(angleError) <= 0.15){
//       // PID turn_corrector(0.6, 0, 89);
//       turn_corrector.kP = 6.7;
//       turn_corrector.kD = 0;
//     }

//     // DEBUG
//     Serial.print("[TURN] H: ");
//     Serial.print(heading);
//     Serial.print(" rad, target: ");
//     Serial.print(targetAngleRad);
//     Serial.print(", err: ");
//     Serial.print(angleError);
//     Serial.print(", corr: ");
//     Serial.print(correction);
//     Serial.print(", L: ");
//     Serial.print(leftSpeed);
//     Serial.print(", R: ");
//     Serial.println(rightSpeed);

//     // Stop when close enough (≈ 2–3 degrees)
//     if (fabs(angleError) < 0.03) {
//       Serial.println("turnToAngle() reached target heading.");
//       //break;
//     }
//   }

//   motorWriteL(0.0);
//   motorWriteR(0.0);
//   delay(200);
//   Serial.println("=== turnToAngle() END ===");
// }

// // =====================================================
// // SETUP
// // =====================================================
// void setup() {
//   Serial.begin(4800);

//   // LED setup
//   pinMode(LED_R, OUTPUT);
//   pinMode(LED_G, OUTPUT);
//   pinMode(LED_B, OUTPUT);
//   LEDWrite(0.1, 0.1, 0.1);  // dim white idle

//   pinMode(GO, INPUT_PULLUP);

//   // Encoder setup
//   pinMode(ENC_L, INPUT_PULLUP);
//   pinMode(ENC_R, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

//   hardStopMotors();
//   delay(500);

//   motorWriteL(0.0);
//   motorWriteR(0.0);

//   SPI.begin();

//   int status = imu.begin();
//   if (status < 0) {
//     LEDWrite(1, 0, 0);    // 🔴 red = IMU failure
//     Serial.println("IMU INIT FAILED");
//     while (1);
//   }

//   status = imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);
//   if (status < 0) {
//     LEDWrite(1, 0, 0);    // 🔴 red = IMU failure
//     Serial.println("GYRO CONFIG FAILED");
//     while (1);
//   }

//   // No calibration here now
//   heading = 0.0;

//   Serial.println("System ready — press GO.");
// }



// // =====================================================
// // MAIN LOOP: SEQUENCE
// // =====================================================
// void loop() {
//     if (digitalRead(GO) == LOW && !motorRunning) {
//     delay(50);
//     if (digitalRead(GO) == LOW) {

//         motorRunning = true;
//         Serial.println("GO pressed → starting calibration.");

//         // ----- GYRO CALIBRATION (max 2.5 seconds) -----
//         LEDWrite(1, 1, 0);  // yellow = calibration start

//         double sum = 0;
//         int samples = 0;
//         unsigned long startCal = millis();

//         while (millis() - startCal < 2500) {  // max 2.5 seconds
//             imu.readSensor();
//             sum += imu.getGyroZ_rads();
//             samples++;

//             // Flash LED yellow
//             if ((millis() / 150) % 2 == 0)
//                 LEDWrite(1, 1, 0);
//             else
//                 LEDWrite(0.3, 0.3, 0);

//             delay(3);
//         }

//         gyroBias = sum / samples;

//         LEDWrite(0, 1, 0); // green = calibration done
//         Serial.print("Gyro bias: ");
//         Serial.println(gyroBias);

//         heading = 0; // reset heading after calibration

//         delay(250); // slight buffer before movement
//         Serial.println("Calibration complete → Running sequence");


//       // Example distances & timeouts — tweak these:
//       double driveDist = 0.5;          // meters
//       double targetDist = 8.0;
//       unsigned long driveTimeout = 6000;  // ms
//       unsigned long turnTimeout  = 4000;  // ms

//       double driveScale = 1;   // 80% of tuned speed
//       double turnScale  = 0.3;   // 60% speed for turns

//       double gyro[3];
      


//       turnToAngle(PI/2.0,turnTimeout,turnScale);
//       // while(1){
//       //   imu.readSensor();

//       //   imu.readGyro(gyro); 
        
//       //   Serial.print(gyro[2]); 
        
//       //   Serial.println();
//       // }
      



//     //   // 1) Turn to -90° (west)
//     //   turnToAngle(-PI/2.0, turnTimeout, turnScale);
//     //   delay(1000);
//     //   // 2) Drive forward
//     //   driveDistanceMeters(driveDist, driveTimeout, driveScale);
//     //   delay(1000);
//     //   // 3) Turn to 0° (north)
//     //   turnToAngle(0.0, turnTimeout, turnScale);
//     //   delay(1000);

//     //   // 4) Drive forward (target)
//     //   driveDistanceMeters(targetDist, driveTimeout, driveScale);
//       //delay(1000);

//       // 5) Turn to +90° (east)
//       //turnToAngle(PI / 2.0, turnTimeout, turnScale);



//       // 6) Drive forward
//       //driveDistanceMeters(driveDist, driveTimeout, driveScale);

//       Serial.println("Sequence complete.");
//       motorRunning = false;

    

//       // Wait for GO to be released to avoid retriggering
//       while (digitalRead(GO) == LOW) {
//         delay(10);
//       }
//       Serial.println("Ready for next GO.");
//     }
//   }
// }
