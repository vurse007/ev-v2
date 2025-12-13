// #define DIR_R 4
// #define PWM_R 5
// #define DIR_L 7
// #define PWM_L 8

// #define GO 0

// // === ENCODER PINS - CHANGE THESE TO MATCH YOUR HARDWARE ===
// #define ENC_L 6  // Left encoder pin 
// #define ENC_R 3   // Right encoder pin WORKING

// #include <ICM20689.h>
// #include <SPI.h>

// #define CS_IMU 17

// double metersToTicks(double meters)
// {
//     // ==== EDIT THESE FOR YOUR SETUP ====
//     const double WHEEL_DIAMETER_IN = 2.0;   // example: 4-inch wheel
//     const double ENCODER_CPR = 13.5;           // counts per revolution // 14
//     const double GEAR_RATIO = 1.0;          // motor revs per wheel rev
//     // ====================================

//     const double INCH_TO_M = 0.0254;        // meters per inch

//     // convert wheel diameter to meters
//     double wheel_diameter_m = WHEEL_DIAMETER_IN * INCH_TO_M;
//     double wheel_circumference_m = PI * wheel_diameter_m;

//     double ticks_per_wheel_rev = ENCODER_CPR * GEAR_RATIO;
//     double ticks_per_meter = ticks_per_wheel_rev / wheel_circumference_m;

//     return meters * ticks_per_meter;
// }


// class PID {
//   public:

//     double kP;
//     double kI;
//     double kD;

//     double error;
//     double prev_error;
//     double speed;
//     double total_error;
//     double derivative;

//     PID(double kp, double ki, double kd): kP(kp), kI(ki), kD(kd), 
//         error(0), prev_error(0), speed(0), total_error(0), derivative(0)
//     {}

//     double calculate(double target, double current, double scale=1.0, bool H=false){
//       //prop
//       error = target - current;

//       //integral
//       total_error += ((error + prev_error) / 2.0);

//       //derivative
//       derivative = error - prev_error;

//       //calc speed
//       speed = scale * (kP * error + kI * total_error + kD * derivative);

//       //update previous loop values
//       prev_error = error;

//       return speed;
//     }

//     void reset(){
//       error = 0;
//       prev_error = 0;
//       speed = 0;
//       total_error = 0;
//       derivative = 0;
//     }
// };

// ICM20689 imu(SPI, CS_IMU);

// // === PID CONTROLLERS (moved to global scope) ===
// PID heading_corrector(2.1, 0, 1500);
// PID distance_corrector(0.005, 0.0, 0);  // Tune these gains for your robot

// bool motorRunning = false;
// unsigned long startTime = 0;
// unsigned long lastUpdateTime = 0;

// double heading = 0.0;

// // === ENCODER VARIABLES ===
// volatile long encoderCountL = 0;
// volatile long encoderCountR = 0;

// // === TARGET DISTANCE (in encoder counts) ===
// const double TARGET_DISTANCE = metersToTicks(8);  // Adjust this value for desired distance
// const double MIN_BASE_SPEED = -0.2;      // Minimum speed to keep robot moving
// const double MAX_BASE_SPEED = 0.4;      // Maximum speed

// // === BREAKOUT CONDITION VARIABLES ===
// const double DISTANCE_ERROR_THRESHOLD = 1.0;  // How close to target counts as "arrived"
// const int SETTLED_COUNT_THRESHOLD = 15;        // How many consecutive loops within threshold
// int settledCount = 0;                          // Counter for consecutive loops within threshold

// // === ENCODER INTERRUPT FUNCTIONS ===
// void encoderISR_L() {
//   encoderCountL++;
// }

// void encoderISR_R() {
//   encoderCountR++;
// }

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

// void setup() {
//   Serial.begin(4800);

//   pinMode(GO, INPUT_PULLUP);

//   // === ENCODER SETUP ===
//   pinMode(ENC_L, INPUT_PULLUP);
//   pinMode(ENC_R, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_L, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_R, RISING);

//   hardStopMotors();
//   delay(1000);
//   motorWriteL(0.0);
//   motorWriteR(0.0);

//   SPI.begin();
//   int status = imu.begin();
//   if (status < 0) {
//     Serial.println("IMU INIT FAILED");
//     while (1);
//   }

//   status = imu.setGyroRange(ICM20689::GYRO_RANGE_500DPS);
//   if (status < 0) {
//     Serial.println("GYRO CONFIG FAILED");
//     while (1);
//   }

//   Serial.println("System ready — press GO.");
// }

// void loop() {
//   unsigned long currentTime = millis();

//   if (digitalRead(GO) == LOW && !motorRunning) {
//     Serial.println("GO pressed → PID controlled driving to target distance");
    
//     // Reset PIDs
//     heading_corrector.reset();
//     distance_corrector.reset();
//     heading = 0.0;
    
//     // Reset encoder counts
//     encoderCountL = 0;
//     encoderCountR = 0;
    
//     // Reset settled counter
//     settledCount = 0;
    
//     imu.readSensor();
//     startTime = millis();
//     lastUpdateTime = millis();
//     motorRunning = true;
//   }

//   if (motorRunning) {
//     unsigned long now = millis();
//     double dt = (now - lastUpdateTime) / 1000.0;
//     lastUpdateTime = now;

//     // === UPDATE HEADING FROM IMU ===
//     imu.readSensor();
//     double gyroZ = imu.getGyroZ_rads();
//     heading += gyroZ * dt;

//     // === CALCULATE AVERAGE DISTANCE FROM ENCODERS ===
//     double avgDistance = (encoderCountL + encoderCountR) / 2.0;

//     // === CALCULATE DISTANCE ERROR ===
//     double distanceError = TARGET_DISTANCE - avgDistance;

//     // === USE HEADING PID FOR STEERING CORRECTION ===
//     // Target heading = 0 (go straight)
//     double headingCorrection = heading_corrector.calculate(0.0, heading, 1, true);



//     // === USE DISTANCE PID FOR SPEED CONTROL ===
//     // Target = TARGET_DISTANCE, Current = avgDistance
//     double baseSpeed = distance_corrector.calculate(TARGET_DISTANCE, avgDistance, 1, false);
    
//     // Constrain base speed between min and max
//     baseSpeed = constrain(baseSpeed, MIN_BASE_SPEED, MAX_BASE_SPEED);
//     // === APPLY STEERING CORRECTION TO MOTORS ===
//     double leftSpeed = constrain(baseSpeed + headingCorrection, 0, 1.0);
//     double rightSpeed = constrain(baseSpeed - headingCorrection, 0, 1.0);

//     motorWriteL(leftSpeed);
//     motorWriteR(rightSpeed);

//     // === CHECK IF WITHIN THRESHOLD ===
//     if (distanceError < DISTANCE_ERROR_THRESHOLD) {
//       settledCount++;
//     } else {
//       settledCount = 0;  // Reset counter if we're not within threshold
//     }

  

//     // === PRINT DEBUG INFO ===
//     Serial.print("Heading: ");
//     Serial.print(heading);
//     Serial.print(" rad, HCorrection: ");
//     Serial.print(headingCorrection);
//     Serial.print(", BaseSpeed: ");
//     Serial.print(baseSpeed);
//     Serial.print(", L: ");
//     Serial.print(leftSpeed);
//     Serial.print(", R: ");
//     Serial.print(rightSpeed);
//     Serial.print(", EncL: ");
//     Serial.print(encoderCountL);
//     Serial.print(", EncR: ");
//     Serial.print(encoderCountR);
//     Serial.print(", AvgDist: ");
//     Serial.print(avgDistance);
//     Serial.print("/");
//     Serial.print(TARGET_DISTANCE);
//     Serial.print(", Error: ");
//     Serial.print(distanceError);
//     Serial.print(", SettledCount: ");
//     Serial.print(settledCount);
//     Serial.print("/");
//     Serial.println(SETTLED_COUNT_THRESHOLD);

//     // === CHECK IF SETTLED AT TARGET FOR ENOUGH LOOPS ===
//     if (settledCount >= SETTLED_COUNT_THRESHOLD) {
//       Serial.println("Target reached and settled → stopping motors");
//       Serial.print("Final encoder counts - Left: ");
//       Serial.print(encoderCountL);
//       Serial.print(", Right: ");
//       Serial.println(encoderCountR);
      
//       motorWriteL(0.0);
//       motorWriteR(0.0);
//       motorRunning = false;
//     }
//   }
// }

// // === Motor control ===
// void motorWriteL(double speed) {
//   if (speed > 0) {
//     analogWrite(PWM_L, (1.0 - speed) * 255.0);
//     digitalWrite(DIR_L, HIGH);
//   } else {
//     analogWrite(PWM_L, (1.0 + speed) * 255.0);
//     digitalWrite(DIR_L, LOW);
//   }
// }

// void motorWriteR(double speed) {
//   if (speed > 0) {
//     analogWrite(PWM_R, (1.0 - speed) * 255.0);
//     digitalWrite(DIR_R, LOW);
//   } else {
//     analogWrite(PWM_R, (1.0 + speed) * 255.0);
//     digitalWrite(DIR_R, HIGH);
//   }
// }

// // tuned straights