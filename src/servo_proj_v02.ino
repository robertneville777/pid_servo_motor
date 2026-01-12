/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

#define ENCODER_USE_INTERRUPTS

/* VARIABLES */

// ANGLE SENSOR/ENCODER
long oldPosition = -999;
long newPosition = -999;
float angleMeasured_deg = 0.0;
Encoder myEnc(2, 3);

// PID
unsigned long lastTime_ms;
float motor_cmd;          // PID output
float angleDesired_deg;  // PID setpoint
float errSum, lastErr;
float kp, ki, kd;
// float kp = 1;
// float ki = 0;
// float kd = 0;
int SampleTime_ms = 1; // 1 ms

// MOTOR
int enA = 9; // speed pin
int in1 = 8; // dir   pin
int in2 = 7; // dir   pin
uint8_t finalMotorCmd = 0; // Value to pin to motor driver

void setup() {

  // Motor setup
  pinMode(enA, OUTPUT);   // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW); // Turn off motors - Initial state
  digitalWrite(in2, LOW);
  
  // Encoder setup
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");
  
}

void loop() {

  // Get angle
  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    newPosition %= 2400; // Stay within 0 to 360 degrees
    angleMeasured_deg = ((float)newPosition/2400)*360; // Convert encoder counts to degrees
  }

  // // Compute PID motor command given current angle and desired angle
  motorPidCmd(); // Compute pid command to motor

  // // Run motor
  runMotor();

  // Serial.println(angleMeasured_deg); // Print current angle
  Serial.println(finalMotorCmd); // Print current angle

}

// Function to compute motor command (PID)
void motorPidCmd() {
  unsigned long now_ms = millis();
  int timeChange_ms = (now_ms - lastTime_ms);
  if(timeChange_ms >= SampleTime_ms)
  {
    /* Compute all the working error variables */
    angleDesired_deg = 0;
    float error = angleDesired_deg - angleMeasured_deg; // 0 - 15 = -15
    // errSum += error;
    // float dErr = (error - lastErr);

    kp = 20;
    // ki = 0;
    // kd = 0;

    /* Compute PID Output */
    motor_cmd = kp * error; // + ki * errSum + kd * dErr; // -15

    /* Remember some variables for next time */
    lastErr = error;
    lastTime_ms = now_ms;
  }

}

// Function to command motor
void runMotor() {
  
  // motor_cmd = -15;

  // Set motor direction
  if (motor_cmd > 0) {      // Pos cmd is CCW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (motor_cmd < 0) { // Neg cmd is CW
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {// motor_cmd == 0   // Don't run motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // Remove sign since direction has been computed
  finalMotorCmd = (uint8_t)(abs(motor_cmd));

  // Cap pin cmd to 255, the upper limit of analogWrite
  if(finalMotorCmd > 255) {
    finalMotorCmd = 255;
  }

  // Send cmd to motor
  analogWrite(enA, finalMotorCmd);

}

void calibrate() {
  // cal code here
}
