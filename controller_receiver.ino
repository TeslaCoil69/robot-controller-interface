#include <AccelStepper.h>

// Controller Data Receiver
// Receives comma-delimited controller data from Python program
// Format: LX,LY,RX,RY,LT,RT,A,B,X,Y,LB,RB,SELECT,START,DPAD_UP,DPAD_DOWN,DPAD_LEFT,DPAD_RIGHT

// Stepper motor pins for Arduino Mega
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define Y_STEP_PIN 60
#define Y_DIR_PIN 61

// Stepper motor settings
#define MAX_SPEED 1000.0  // Maximum speed in steps per second
#define MAX_ACCEL 500.0   // Maximum acceleration in steps per second per second
#define STEPS_PER_DEGREE 20  // Adjust based on your stepper motor and gearing

// Smoothing settings
#define SMOOTHING_FACTOR 0.1  // Lower = smoother but more lag (0.0 to 1.0)
#define RAMP_FACTOR 0.05      // Lower = smoother acceleration (0.0 to 1.0)
#define DEADZONE 5           // Ignore joystick movements smaller than this

// Create stepper motor instances
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Controller data structure
struct ControllerData {
  int leftX;      // Left stick X
  int leftY;      // Left stick Y
  int rightX;     // Right stick X
  int rightY;     // Right stick Y
  int leftTrigger;  // Left trigger
  int rightTrigger; // Right trigger
  bool buttonA;   // A button
  bool buttonB;   // B button
  bool buttonX;   // X button
  bool buttonY;   // Y button
  bool leftBumper;  // Left bumper
  bool rightBumper; // Right bumper
  bool select;    // Select button
  bool start;     // Start button
  bool dpadUp;    // D-pad Up
  bool dpadDown;  // D-pad Down
  bool dpadLeft;  // D-pad Left
  bool dpadRight; // D-pad Right
};

ControllerData controllerData;
String inputString = "";    // String to hold incoming data
bool stringComplete = false;  // Whether the string is complete

// Current gimbal position and smoothed values
float currentX = 0;
float currentY = 0;
float smoothedX = 0;
float smoothedY = 0;
float targetSpeedX = 0;
float targetSpeedY = 0;
float currentSpeedX = 0;
float currentSpeedY = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  inputString.reserve(200);  // Reserve 200 bytes for the input string
  
  // Initialize stepper motors
  setupSteppers();
  
  // Initialize all controller data to neutral/off
  resetControllerData();
}

void setupSteppers() {
  // Configure X axis stepper
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(MAX_ACCEL);
  
  // Configure Y axis stepper
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(MAX_ACCEL);
}

void loop() {
  // Check if we have received a complete string
  if (stringComplete) {
    parseControllerData(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Update gimbal position based on joystick input
  updateGimbal();
  
  // Run the steppers
  stepperX.run();
  stepperY.run();
}

void updateGimbal() {
  // Apply deadzone to joystick inputs
  float inputX = abs(controllerData.leftX) < DEADZONE ? 0 : controllerData.leftX;
  float inputY = abs(controllerData.leftY) < DEADZONE ? 0 : controllerData.leftY;
  
  // Calculate target speeds (joystick input to speed conversion)
  targetSpeedX = inputX * (MAX_SPEED / 100.0);  // Scale joystick to max speed
  targetSpeedY = inputY * (MAX_SPEED / 100.0);
  
  // Smooth speed changes using ramping
  currentSpeedX = currentSpeedX + (targetSpeedX - currentSpeedX) * RAMP_FACTOR;
  currentSpeedY = currentSpeedY + (targetSpeedY - currentSpeedY) * RAMP_FACTOR;
  
  // Apply smoothing to position changes
  smoothedX = smoothedX + (currentSpeedX - smoothedX) * SMOOTHING_FACTOR;
  smoothedY = smoothedY + (currentSpeedY - smoothedY) * SMOOTHING_FACTOR;
  
  // Update current position
  currentX += smoothedX * 0.001;  // Scale factor for position update
  currentY += smoothedY * 0.001;
  
  // Limit the range of motion
  currentX = constrain(currentX, -45.0, 45.0);
  currentY = constrain(currentY, -45.0, 45.0);
  
  // Convert degrees to steps
  long xSteps = currentX * STEPS_PER_DEGREE;
  long ySteps = currentY * STEPS_PER_DEGREE;
  
  // Move steppers to target positions
  stepperX.moveTo(xSteps);
  stepperY.moveTo(ySteps);
  
  // Set speeds for smooth movement
  stepperX.setSpeed(currentSpeedX);
  stepperY.setSpeed(currentSpeedY);
}

// SerialEvent occurs whenever new data comes in the hardware serial RX
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseControllerData(String data) {
  // Split the string into values
  int values[18];  // Array to hold all values
  int valueIndex = 0;
  int startIndex = 0;
  
  // Parse comma-separated values
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',' || data[i] == '\n') {
      values[valueIndex++] = data.substring(startIndex, i).toInt();
      startIndex = i + 1;
    }
  }
  
  // Update controller data structure
  if (valueIndex == 18) {  // Make sure we got all values
    controllerData.leftX = values[0];
    controllerData.leftY = values[1];
    controllerData.rightX = values[2];
    controllerData.rightY = values[3];
    controllerData.leftTrigger = values[4];
    controllerData.rightTrigger = values[5];
    controllerData.buttonA = values[6] == 1;
    controllerData.buttonB = values[7] == 1;
    controllerData.buttonX = values[8] == 1;
    controllerData.buttonY = values[9] == 1;
    controllerData.leftBumper = values[10] == 1;
    controllerData.rightBumper = values[11] == 1;
    controllerData.select = values[12] == 1;
    controllerData.start = values[13] == 1;
    controllerData.dpadUp = values[14] == 1;
    controllerData.dpadDown = values[15] == 1;
    controllerData.dpadLeft = values[16] == 1;
    controllerData.dpadRight = values[17] == 1;
    
    // Print received data for debugging
    printControllerData();
  }
}

void resetControllerData() {
  controllerData.leftX = 0;
  controllerData.leftY = 0;
  controllerData.rightX = 0;
  controllerData.rightY = 0;
  controllerData.leftTrigger = 0;
  controllerData.rightTrigger = 0;
  controllerData.buttonA = false;
  controllerData.buttonB = false;
  controllerData.buttonX = false;
  controllerData.buttonY = false;
  controllerData.leftBumper = false;
  controllerData.rightBumper = false;
  controllerData.select = false;
  controllerData.start = false;
  controllerData.dpadUp = false;
  controllerData.dpadDown = false;
  controllerData.dpadLeft = false;
  controllerData.dpadRight = false;
}

void printControllerData() {
  Serial.println("Controller Data:");
  Serial.print("Left Stick: (");
  Serial.print(controllerData.leftX);
  Serial.print(",");
  Serial.print(controllerData.leftY);
  Serial.println(")");
  
  Serial.print("Right Stick: (");
  Serial.print(controllerData.rightX);
  Serial.print(",");
  Serial.print(controllerData.rightY);
  Serial.println(")");
  
  Serial.print("Triggers: L=");
  Serial.print(controllerData.leftTrigger);
  Serial.print(" R=");
  Serial.println(controllerData.rightTrigger);
  
  Serial.print("Buttons: A=");
  Serial.print(controllerData.buttonA);
  Serial.print(" B=");
  Serial.print(controllerData.buttonB);
  Serial.print(" X=");
  Serial.print(controllerData.buttonX);
  Serial.print(" Y=");
  Serial.println(controllerData.buttonY);
  
  Serial.print("Bumpers: L=");
  Serial.print(controllerData.leftBumper);
  Serial.print(" R=");
  Serial.println(controllerData.rightBumper);
  
  Serial.print("Menu: Select=");
  Serial.print(controllerData.select);
  Serial.print(" Start=");
  Serial.println(controllerData.start);
  
  Serial.print("D-pad: U=");
  Serial.print(controllerData.dpadUp);
  Serial.print(" D=");
  Serial.print(controllerData.dpadDown);
  Serial.print(" L=");
  Serial.print(controllerData.dpadLeft);
  Serial.print(" R=");
  Serial.println(controllerData.dpadRight);
  Serial.println();
} 