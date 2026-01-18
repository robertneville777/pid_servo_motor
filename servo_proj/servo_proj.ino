/* 
 * Sketch for servo motor using 775 DC motor and optical rotary encoder. 
 */

#include <Encoder.h>
#include <DueFlashStorage.h>

#define ENCODER_USE_INTERRUPTS

/* VARIABLES */

// ANGLE SENSOR/ENCODER
long oldPosition = -999;
long newPosition = -999;
float angleMeasured_deg = 0.0;
Encoder myEnc(2, 3);

// MOTOR
int enA = 9; // speed pin
int in1 = 8; // dir   pin
int in2 = 7; // dir   pin
uint8_t finalMotorCmd = 0; // Value to pin to motor driver

// PID
unsigned long lastTime_ms;
float motor_cmd;          // PID output
float angleDesired_deg;   // PID setpoint
float errSum, lastErr;
float kp, ki, kd;
int SampleTime_ms = 1; // 1 ms

// SERIAL COMMUNICATION
const byte numChars = 10;
char receivedChars[numChars];       // array to store received data
char tempChars[numChars];           // temporary array for use when parsing
boolean newData = false;
char pcCmdType[numChars];           // Command from keyboard, e.g. 'angle', 'p', 'i', or 'd'
const char pcCmdAngle[] = "angle";
const char pcCmdP[]     = "p";
const char pcCmdI[]     = "i";
const char pcCmdD[]     = "d";
const char pcCmdCal[]   = "cal";
const char pcCmdSave[]  = "save";
float pcCmdValue = 0;               // Value from keyboard to set

// FLASH NVM
DueFlashStorage dueFlashStorage;  // Create flash storage object
struct config {
  uint8_t initStatus;             // Flash is 255 at first run. Set initStatus to 
  float   savedMeasuredAngle;     // indicate values have been initialized
  float   savedDesiredAngle;
  float   savedKp;
  float   savedKi;
  float   savedKd;
};
config configFromNvm; // Struct read from NVM
config configToNvm;   // Struct to write to NVM
uint8_t tempConfig[sizeof(configToNvm)]; // Temporary u8 array to store struct 
                                         // when reading/writing to NVM
uint8_t* b; // pointer for reading from flash

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
  
  // Load NVM values
  b = dueFlashStorage.readAddress(0); // read byte array starting at address 0
  memcpy(&configFromNvm, b, sizeof(configFromNvm)); // Copy from flash memory to config struct

  // Flash values by default are 255. When config values are saved,
  // the initStatus is set to 0 to indicate there are saved values.
  if(configFromNvm.initStatus == 0) { // If there are saved values, load them
    // Set angles
    uint32_t savedEncPos = (uint32_t)((configFromNvm.savedMeasuredAngle * 2400)/360); // Convert saved angle to encoder position
    myEnc.write(savedEncPos);
    angleDesired_deg = configFromNvm.savedDesiredAngle;

    // Set PID parameters
    kp = configFromNvm.savedKp;
    ki = configFromNvm.savedKi;
    kd = configFromNvm.savedKd;
  }
  else {
    // There are no saved values. Use program's initial values
  }

}

void loop() {

  // Get encoder angle
  getAngle();

  // Compute motor command using PID
  motorPidCmd();

  // // Run motor
  runMotor();

  // Get angle and PID values from user if available. Used next loop.
  getKeyboardInput();

  // Print angle and PID data
  printData();

}

// Function to read encoder angle
inline void getAngle() {

  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    newPosition %= 2400; // Stay within 0 to 360 degrees
    angleMeasured_deg = ((float)newPosition/2400)*360; // Convert encoder counts to degrees
  }

}

// Function that prints out angle and PID data
inline void printData() {

  Serial.print("Desired Angle: ");
  Serial.print(angleDesired_deg);

  Serial.print(", Measured Angle: ");
  Serial.print(angleMeasured_deg);

  Serial.print(", Kp: ");
  Serial.print(kp);

  Serial.print(", Ki: ");
  Serial.print(ki);

  Serial.print(", Kd: ");
  Serial.println(kd);

}

// Function that takes angle and PID cmds from serial and sets the appropriate values.
void getKeyboardInput() {
    static uint8_t ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }

    if (newData == true) { // After keyboard value has been input, convert to int and set angle
        
        char* strtokIndx; // this is used by strtok() as an index

        strcpy(tempChars, receivedChars);

        strtokIndx = strtok(tempChars, " ");  // Get the first part - the string
        strcpy(pcCmdType, strtokIndx);        // Copy to pcCmdType

        strtokIndx = strtok(NULL, ",");       // This continues where the previous call left off
        pcCmdValue = atof(strtokIndx);        // Convert to float

        if(strcmp(pcCmdType, pcCmdAngle) == 0) {    // if cmd string is "angle"
          angleDesired_deg = pcCmdValue;            // set desired angle
        }
        else if(strcmp(pcCmdType, pcCmdP) == 0) {   // if cmd string is "p"
          kp = pcCmdValue;                          // set kp
        }
        else if(strcmp(pcCmdType, pcCmdI) == 0) {   // if cmd string is "i"
          ki = pcCmdValue;                          // set ki
        }
        else if(strcmp(pcCmdType, pcCmdD) == 0) {   // if cmd string is "d"
          kd = pcCmdValue;                          // set kd
        }
        else if(strcmp(pcCmdType, pcCmdCal) == 0) { // if cmd string is "cal"
          newPosition = myEnc.readAndReset();       // reset measured angle to 0
        }
        else if(strcmp(pcCmdType, pcCmdSave) == 0){ // if cmd string is "save"

          configToNvm = {
            .initStatus = 0,
            .savedMeasuredAngle = angleMeasured_deg,
            .savedDesiredAngle  = angleDesired_deg,
            .savedKp = kp,
            .savedKi = ki,
            .savedKd = kd
          };

          memcpy(tempConfig, &configToNvm, sizeof(configToNvm));     // Copy struct to array
          dueFlashStorage.write(0, tempConfig, sizeof(configToNvm)); // Write array to flash at address 0

        }
        else {
          // Not a recognized command. Could print out some error message.
        }

        newData = false;
    }
}

// Function to compute motor command (PID)
void motorPidCmd() {
  unsigned long now_ms = millis();
  int timeChange_ms = (now_ms - lastTime_ms);
  if(timeChange_ms >= SampleTime_ms)
  {
    /* Compute all the working error variables */
    float error = angleDesired_deg - angleMeasured_deg;
    errSum += error;
    float dErr = (error - lastErr);

    /* Compute PID Output */
    motor_cmd = kp * error + ki * errSum + kd * dErr;

    /* Remember some variables for next time */
    lastErr = error;
    lastTime_ms = now_ms;
  }

}

// Function to command motor
void runMotor() {

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
