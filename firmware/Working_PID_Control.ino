/*
   7MRI0060 - Applied Medical Robotics Module
   October 2024
   Author: Alejandro Granados and Harry Robertshaw

   Purpose: Run a PID controller

   Tasks:
    1. Set pins to code​
    2. Compute degrees and bound them to [-360,360]​
    3. Compute elapsed time (deltaT) and control the frequency of printing​
    4. Set demand position in degrees and compute errors (compared to previous, accumulation, and differential).
       Hint: make sure you update previous error at the end.
    5. Compute PID​
    6. Send voltage to motors​
    7. Plot demand versus current position in degrees
*/

// --------- Constants from your template ---------
// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 12;//3575.0855;
const float GR  = 60;//297.924;
const float CPR = 2;//3; // decoding multiplier (set by how you count edges); keep from your template

// --------- Encoder counters (updated in ISRs) ---------
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;

// (Kept but not strictly needed with this ISR approach)
int aLastState_m1;
int aLastState_m2;

// --------- Encoder pins (A is interrupt pin on UNO, B is regular input) ---------
const int encoderPinA_m1 = 2;   // interrupt-capable on UNO
const int encoderPinB_m1 = 10;  // regular input
const int encoderPinA_m2 = 3;   // interrupt-capable on UNO
const int encoderPinB_m2 = 11;  // regular input

// --------- H-bridge pins for direction ---------
const int motorPin1_m1 = 5;
const int motorPin2_m1 = 4;
const int motorPin1_m2 = 8;
const int motorPin2_m2 = 7;

// --------- Enable (PWM) pins ---------
const int enablePin_m1 = 6;     // PWM-capable
const int enablePin_m2 = 9;     // PWM-capable

// --------- Position state ---------
long  currentPosition_m1 = 0;   // raw counts (snapshot of counter_m1)
long  currentPosition_m2 = 0;   // raw counts (snapshot of counter_m2)
float demandPositionInDegrees_m1 = -270.0;
float demandPositionInDegrees_m2 = -270.0; 
float currentPositionInDegrees_m1 = 0.0f;
float currentPositionInDegrees_m2 = 0.0f;

// --------- Timing ---------
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;           // microseconds

// --------- PID gains (start with P only; add D then I) ---------
float Kp_m1 = 10, Kd_m1 = 0.2, Ki_m1 = 0;
float Kp_m2 = 10, Kd_m2 = 0.2, Ki_m2 = 0;

// --------- Error bookkeeping ---------
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

void setup() {
  Serial.begin(115200);

  // -------------------- Task 1: Set pins to code --------------------
  // Inputs for encoders
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);

  // Outputs for H-bridge direction pins and PWM pins
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m1,  OUTPUT);
  pinMode(enablePin_m2,  OUTPUT);

  // Attach interrupts on the A channels (UNO supports interrupts on D2 & D3)
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);

  delay(3000);                // small delay so you can open Serial Plotter if needed
  previousTime = micros();

  // Optional CSV header for plotting
  Serial.println("t_ms,demand_m1_deg,angle_m1_deg,err_m1_deg,out_m1_pwm,demand_m2_deg,angle_m2_deg,err_m2_deg,out_m2_pwm");
}

void loop() {
  // -------------------- Task 2: Compute degrees & bound to [-360, 360] --------------------
  // Take atomic snapshots of the counters
  noInterrupts();
  currentPosition_m1 = counter_m1;
  currentPosition_m2 = counter_m2;
  interrupts();

  // Convert counts to degrees
  // Using lecture idea: theta_deg = counts * 360 / (PPR * GR * CPR)
  // (CPR accounts for how many edges per pulse you count; keep as in your template)
  currentPositionInDegrees_m1 = ((float)currentPosition_m1 * 360.0f) / (PPR * GR * CPR);
  currentPositionInDegrees_m2 = ((float)currentPosition_m2 * 360.0f) / (PPR * GR * CPR);

  // Keep angles in a sane window (±360). For simplicity, re-zero the counter if exceeded.
  if (currentPositionInDegrees_m1 > 360.0f || currentPositionInDegrees_m1 < -360.0f) {
    noInterrupts(); counter_m1 = 0; interrupts();
    currentPositionInDegrees_m1 = 0.0f;
  }
  if (currentPositionInDegrees_m2 > 360.0f || currentPositionInDegrees_m2 < -360.0f) {
    noInterrupts(); counter_m2 = 0; interrupts();
    currentPositionInDegrees_m2 = 0.0f;
  }

  // -------------------- Task 3: deltaT (µs) and print rate control --------------------
  currentTime = micros();
  deltaT = currentTime - previousTime;

  // Use the deltaT threshold to control control/print frequency (e.g., every ~0.4 ms here).
  // You can set this to ~10000 for ~10 ms if you want slower logging.
  if (deltaT > 10000) {

    // Convert microseconds to seconds for the I and D math
    float dt = deltaT / 1e6f;

    // -------------------- Task 4: Errors (P, I, D) --------------------
    float error_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float error_m2 = demandPositionInDegrees_m2 - currentPositionInDegrees_m2;

    // Integral (accumulated) error with simple anti-windup clamp
    errorPositionInDegrees_sum_m1 += error_m1 * dt;
    errorPositionInDegrees_sum_m2 += error_m2 * dt;
    const float I_CLAMP = 2000.0f;
    if (errorPositionInDegrees_sum_m1 >  I_CLAMP) errorPositionInDegrees_sum_m1 =  I_CLAMP;
    if (errorPositionInDegrees_sum_m1 < -I_CLAMP) errorPositionInDegrees_sum_m1 = -I_CLAMP;
    if (errorPositionInDegrees_sum_m2 >  I_CLAMP) errorPositionInDegrees_sum_m2 =  I_CLAMP;
    if (errorPositionInDegrees_sum_m2 < -I_CLAMP) errorPositionInDegrees_sum_m2 = -I_CLAMP;

    // Derivative (rate of change of error)
    float dErr_m1 = (error_m1 - errorPositionInDegrees_prev_m1) / dt;
    float dErr_m2 = (error_m2 - errorPositionInDegrees_prev_m2) / dt;

    // -------------------- Task 5: PID output --------------------
    float u_m1 = Kp_m1 * error_m1 + Ki_m1 * errorPositionInDegrees_sum_m1 + Kd_m1 * dErr_m1;
    float u_m2 = Kp_m2 * error_m2 + Ki_m2 * errorPositionInDegrees_sum_m2 + Kd_m2 * dErr_m2;

    // Constrain to motor range [-255, 255]
    // if (u_m1 >  255.0f) u_m1 =  255.0f;
    // if (u_m1 < -255.0f) u_m1 = -255.0f;
    // if (u_m2 >  255.0f) u_m2 =  255.0f;
    // if (u_m2 < -255.0f) u_m2 = -255.0f;
    if (u_m1 >  100.0f) u_m1 =  100.0f;
    if (u_m1 < -100.0f) u_m1 = -100.0f;
    if (u_m2 >  100.0f) u_m2 =  100.0f;
    if (u_m2 < -100.0f) u_m2 = 100.0f;

    // -------------------- Task 6: Send voltage to motors --------------------
    // Helper lambdas to set direction + speed for each motor
    auto driveMotor = [](int pin1, int pin2, int en, float u){
      int pwm = (int)fabs(u);
      if (pwm > 255) pwm = 255;

      if (u > 5) {                 // forward
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(en, pwm);
      } else if (u < -5) {         // reverse
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(en, pwm);
      } else {                     // small deadband -> stop
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        analogWrite(en, 0);
      }
    };

    driveMotor(motorPin1_m1, motorPin2_m1, enablePin_m1, -u_m1);
    driveMotor(motorPin1_m2, motorPin2_m2, enablePin_m2, u_m2);

    // -------------------- Task 7: Plot demand vs current (CSV for Serial Plotter) --------------------
    unsigned long t_ms = millis();
    Serial.print(t_ms); Serial.print(",");
    Serial.print(demandPositionInDegrees_m1); Serial.print(",");
    Serial.print(currentPositionInDegrees_m1); Serial.print(",");
    Serial.print(error_m1); Serial.print(",");
    Serial.print((int)u_m1); Serial.print(",");
    Serial.print(demandPositionInDegrees_m2); Serial.print(",");
    Serial.print(currentPositionInDegrees_m2); Serial.print(",");
    Serial.print(error_m2); Serial.print(",");
    Serial.println((int)u_m2);

    // Update previous error & previous time (end of Task 4 hint)
    errorPositionInDegrees_prev_m1 = error_m1;
    errorPositionInDegrees_prev_m2 = error_m2;
    previousTime = currentTime;
  }
}

// -------------------- Interrupt Service Routines (ISRs) --------------------
// Quadrature decode using A-channel interrupt: direction is decided by the state of B.
void updateEncoder_m1() {
  bool A = digitalRead(encoderPinA_m1);
  bool B = digitalRead(encoderPinB_m1);
  // If A == B, count up; else count down (one common convention)
  if (A == B) counter_m1++; else counter_m1--;
}

void updateEncoder_m2() {
  bool A = digitalRead(encoderPinA_m2);
  bool B = digitalRead(encoderPinB_m2);
  if (A == B) counter_m2++; else counter_m2--;
}
