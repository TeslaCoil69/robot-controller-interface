#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPS++.h>

// Controller Data Receiver
// Receives data from RYL890 module on Serial1 and forwards to computer over Serial
// Format: LX,LY,RX,RY,LT,RT,A,B,X,Y,LB,RB,SELECT,START,DPAD_UP,DPAD_DOWN,DPAD_LEFT,DPAD_RIGHT

// Function prototypes
void resetControllerData();
void printControllerData();
void parseControllerData(String data);
void setupSteppers();
void setupServos();
void updateGimbal();
void updateServos();
void calibrateAccelerometer();
void handleButtons();
void armMotors();
void disarmMotors();
void resetMotors();
void updateGPS();
void reportGPS();

// Stepper motor pins for Arduino Mega
#define X_STEP_PIN 22    // Digital pin 22 for X stepper step
#define X_DIR_PIN 23     // Digital pin 23 for X stepper direction
#define Y_STEP_PIN 24    // Digital pin 24 for Y stepper step
#define Y_DIR_PIN 25     // Digital pin 25 for Y stepper direction
#define X_ENABLE_PIN 26  // Digital pin 26 for X stepper enable
#define Y_ENABLE_PIN 27  // Digital pin 27 for Y stepper enable

// Servo motor pins
#define STEERING_SERVO_PIN 9   // PWM pin for steering servo
#define DRIVE_SERVO_PIN 10     // PWM pin for drive servo

// Stepper motor settings
#define MAX_SPEED 1000.0  // Maximum speed in steps per second
#define MAX_ACCEL 500.0   // Maximum acceleration in steps per second per second
#define STEPS_PER_DEGREE 20  // Adjust based on your stepper motor and gearing
#define NORMAL_SPEED_MULTIPLIER 1.0    // Normal speed (100%)
#define SLOW_SPEED_MULTIPLIER 0.25     // Slow speed (25%) when left bumper is pressed

// Servo settings
#define STEERING_CENTER 90     // Center position for steering servo
#define DRIVE_STOP 90          // Stop position for drive servo
#define STEERING_RANGE 45      // Maximum steering angle from center
#define DRIVE_MIN 0            // Minimum drive position (full reverse)
#define DRIVE_MAX 180          // Maximum drive position (full forward)

// ADXL345 settings
#define ADXL345_ADDR 0x53  // I2C address for ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);  // Unique sensor ID
float targetPitch = 0.0;   // Target pitch angle (degrees)
float targetRoll = 0.0;    // Target roll angle (degrees)
float currentPitch = 0.0;  // Current pitch angle (degrees)
float currentRoll = 0.0;   // Current roll angle (degrees)
#define STABILIZATION_GAIN 0.5  // How aggressively to correct (0.0 to 1.0)
#define STABILIZATION_THRESHOLD 0.5  // Minimum angle to correct (degrees)
#define JOYSTICK_DEADZONE 5      // Deadzone for joystick input (-5 to 5)
#define STABILIZATION_DEADZONE 0.2  // Deadzone for stabilization (degrees)

// Create stepper motor instances
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Create servo instances
Servo steeringServo;
Servo driveServo;

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
String inputString = "";    // String to hold incoming data from Serial1
bool stringComplete = false;  // Whether the string is complete

// Current gimbal position
float currentX = 0;
float currentY = 0;

// Watchdog settings
#define WATCHDOG_TIMEOUT 1000  // Timeout in milliseconds (1 second)
unsigned long lastPacketTime = 0;  // Timestamp of last received packet

// Arming system
#define STATUS_LED_PIN 13    // Built-in LED on most Arduino boards
bool isArmed = false;        // Armed state
bool lastSelectState = false; // Previous select button state for edge detection
bool lastStartState = false;  // Previous start button state for edge detection

// GPS settings
#define GPS_BAUD 9600  // Most GPS modules use 9600 baud
TinyGPSPlus gps;       // GPS parser object
unsigned long lastGPSTime = 0;  // Last time GPS data was reported
#define GPS_REPORT_INTERVAL 1000  // Report GPS data every second

void setup() {
  // Initialize serial communications
  Serial.begin(115200);    // Main serial port for computer communication
  Serial1.begin(57600);    // Serial1 for RYL890 module
  Serial2.begin(GPS_BAUD); // Serial2 for GPS module
  inputString.reserve(200);  // Reserve 200 bytes for the input string
  
  // Initialize I2C for ADXL345
  Wire.begin();
  
  // Initialize accelerometer
  if(!accel.begin()) {
    Serial.println("Could not find ADXL345 accelerometer!");
    while(1);
  }
  
  // Set accelerometer range and data rate
  accel.setRange(ADXL345_RANGE_2_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  
  // Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);  // Start disarmed
  
  // Initialize stepper motors
  setupSteppers();
  
  // Initialize servos
  setupServos();
  
  // Initialize all controller data to neutral/off
  resetControllerData();
  
  // Initialize watchdog timer
  lastPacketTime = millis();
  
  // Initialize stepper enable pins
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, HIGH);  // HIGH = disabled (active low)
  digitalWrite(Y_ENABLE_PIN, HIGH);  // HIGH = disabled (active low)
  
  // Start in disarmed state
  disarmMotors();
  
  // Calibrate accelerometer
  calibrateAccelerometer();
}

void setupSteppers() {
  // Configure X axis stepper
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(MAX_ACCEL);
  
  // Configure Y axis stepper
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(MAX_ACCEL);
}

void setupServos() {
  // Attach servos to their respective pins
  steeringServo.attach(STEERING_SERVO_PIN);
  driveServo.attach(DRIVE_SERVO_PIN);
  
  // Set initial positions
  steeringServo.write(STEERING_CENTER);
  driveServo.write(DRIVE_STOP);
}

void loop() {
  // Check if we have received a complete string from Serial1
  if (stringComplete) {
    // Forward the received data to the computer
    Serial.println(inputString);
    
    // Parse the data for local use (gimbal control)
    parseControllerData(inputString);
    
    // Update watchdog timestamp
    lastPacketTime = millis();
    
    // Handle button presses
    handleButtons();
    
    // Reset for next message
    inputString = "";
    stringComplete = false;
  }
  
  // Check watchdog timer
  if (millis() - lastPacketTime > WATCHDOG_TIMEOUT) {
    // No data received for 1 second, disarm motors
    if (isArmed) {
      isArmed = false;
      disarmMotors();
    }
  } else if (isArmed) {
    // Only update positions if armed
    updateGimbal();
    updateServos();
  }
  
  // Update GPS data
  updateGPS();
  
  // Report GPS data periodically
  if (millis() - lastGPSTime >= GPS_REPORT_INTERVAL) {
    reportGPS();
    lastGPSTime = millis();
  }
  
  // Run the steppers
  stepperX.run();
  stepperY.run();
}

void handleButtons() {
  // Handle select button for arming/disarming
  if (controllerData.select && !lastSelectState) {
    // Toggle armed state
    isArmed = !isArmed;
    if (isArmed) {
      armMotors();
    } else {
      disarmMotors();
    }
  }
  lastSelectState = controllerData.select;
  
  // Handle start button for calibration only
  if (controllerData.start && !lastStartState) {
    // Only recalibrate if armed
    if (isArmed) {
      calibrateAccelerometer();
    }
  }
  lastStartState = controllerData.start;
}

void calibrateAccelerometer() {
  // Take multiple readings and average them for calibration
  const int numReadings = 10;
  float sumPitch = 0;
  float sumRoll = 0;
  
  // Flash LED during calibration
  for(int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
  
  for(int i = 0; i < numReadings; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    // Calculate angles from accelerometer data
    float pitch = atan2(event.acceleration.y, sqrt(event.acceleration.x * event.acceleration.x + 
                                                  event.acceleration.z * event.acceleration.z)) * 180.0 / PI;
    float roll = atan2(event.acceleration.x, sqrt(event.acceleration.y * event.acceleration.y + 
                                                 event.acceleration.z * event.acceleration.z)) * 180.0 / PI;
    
    sumPitch += pitch;
    sumRoll += roll;
    delay(10);
  }
  
  // Set target angles as the average of calibration readings
  targetPitch = sumPitch / numReadings;
  targetRoll = sumRoll / numReadings;
  
  // Restore LED state based on armed status
  digitalWrite(STATUS_LED_PIN, isArmed ? HIGH : LOW);
}

void updateGimbal() {
  // Get current orientation from accelerometer
  sensors_event_t event;
  accel.getEvent(&event);
  
  // Calculate current angles
  currentPitch = atan2(event.acceleration.y, sqrt(event.acceleration.x * event.acceleration.x + 
                                                 event.acceleration.z * event.acceleration.z)) * 180.0 / PI;
  currentRoll = atan2(event.acceleration.x, sqrt(event.acceleration.y * event.acceleration.y + 
                                                event.acceleration.z * event.acceleration.z)) * 180.0 / PI;
  
  // Calculate stabilization corrections
  float pitchError = targetPitch - currentPitch;
  float rollError = targetRoll - currentRoll;
  
  // Apply deadband to prevent jitter
  if(abs(pitchError) < STABILIZATION_THRESHOLD) pitchError = 0;
  if(abs(rollError) < STABILIZATION_THRESHOLD) rollError = 0;
  
  // Apply additional deadzone for stabilization
  if(abs(pitchError) < STABILIZATION_DEADZONE) pitchError = 0;
  if(abs(rollError) < STABILIZATION_DEADZONE) rollError = 0;
  
  // Print stabilization corrections only when they're being applied
  if(isArmed && (abs(pitchError) > STABILIZATION_DEADZONE || abs(rollError) > STABILIZATION_DEADZONE)) {
    Serial.print("Stab: P=");
    Serial.print(pitchError, 1);  // 1 decimal place
    Serial.print(" R=");
    Serial.println(rollError, 1);  // 1 decimal place
  }
  
  // Convert joystick values (-100 to 100) to target positions
  const float SCALE_FACTOR = 0.1;  // Adjust this to change sensitivity
  
  // Apply joystick deadzone
  int leftX = abs(controllerData.leftX) < JOYSTICK_DEADZONE ? 0 : controllerData.leftX;
  int leftY = abs(controllerData.leftY) < JOYSTICK_DEADZONE ? 0 : controllerData.leftY;
  
  // Calculate target positions with both joystick and stabilization inputs
  float targetX = currentX + (leftX * SCALE_FACTOR);
  float targetY = currentY + (leftY * SCALE_FACTOR);
  
  // Add stabilization corrections when armed
  if(isArmed) {
    targetX += rollError * STABILIZATION_GAIN;
    targetY += pitchError * STABILIZATION_GAIN;
  }
  
  // Limit the range of motion
  targetX = constrain(targetX, -45.0, 45.0);
  targetY = constrain(targetY, -45.0, 45.0);
  
  // Convert degrees to steps
  long xSteps = targetX * STEPS_PER_DEGREE;
  long ySteps = targetY * STEPS_PER_DEGREE;
  
  // Check if movement is needed for each axis
  bool xNeedsMovement = (xSteps != stepperX.currentPosition());
  bool yNeedsMovement = (ySteps != stepperY.currentPosition());
  
  // Enable/disable steppers based on movement needs
  digitalWrite(X_ENABLE_PIN, xNeedsMovement ? LOW : HIGH);  // LOW = enabled
  digitalWrite(Y_ENABLE_PIN, yNeedsMovement ? LOW : HIGH);  // LOW = enabled
  
  // Apply speed multiplier based on left bumper
  float speedMultiplier = controllerData.leftBumper ? SLOW_SPEED_MULTIPLIER : NORMAL_SPEED_MULTIPLIER;
  
  // Update stepper speeds
  stepperX.setMaxSpeed(MAX_SPEED * speedMultiplier);
  stepperY.setMaxSpeed(MAX_SPEED * speedMultiplier);
  stepperX.setAcceleration(MAX_ACCEL * speedMultiplier);
  stepperY.setAcceleration(MAX_ACCEL * speedMultiplier);
  
  // Move steppers to target positions
  stepperX.moveTo(xSteps);
  stepperY.moveTo(ySteps);
  
  // Update current position
  currentX = targetX;
  currentY = targetY;
}

void updateServos() {
  // Convert right joystick values (-100 to 100) to servo positions
  // Right X controls steering
  int steeringValue = map(controllerData.rightX, -100, 100, 
                         STEERING_CENTER - STEERING_RANGE, 
                         STEERING_CENTER + STEERING_RANGE);
  steeringServo.write(steeringValue);
  
  // Right Y controls drive (forward/backward) using full servo range
  int driveValue = map(controllerData.rightY, -100, 100,
                      DRIVE_MAX,  // Full forward at -100
                      DRIVE_MIN); // Full reverse at 100
  driveServo.write(driveValue);
}

// SerialEvent1 occurs whenever new data comes in Serial1 (RYL890)
void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
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
  }
}

void resetMotors() {
  // Enable both steppers for reset
  digitalWrite(X_ENABLE_PIN, LOW);  // LOW = enabled
  digitalWrite(Y_ENABLE_PIN, LOW);  // LOW = enabled
  
  // Reset gimbal to center position
  currentX = 0;
  currentY = 0;
  stepperX.moveTo(0);
  stepperY.moveTo(0);
  
  // Reset servos to center/stop positions
  steeringServo.write(STEERING_CENTER);
  driveServo.write(DRIVE_STOP);
  
  // Disable steppers after movement
  digitalWrite(X_ENABLE_PIN, HIGH);  // HIGH = disabled
  digitalWrite(Y_ENABLE_PIN, HIGH);  // HIGH = disabled
}

void armMotors() {
  // Turn on status LED
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void disarmMotors() {
  // Turn off status LED
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Reset all motors to safe positions
  resetMotors();
}

void resetControllerData() {
  // Reset all controller values to neutral/off positions
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

// SerialEvent2 occurs whenever new data comes in Serial2 (GPS)
void serialEvent2() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

void updateGPS() {
  // Feed GPS data to the parser
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

void reportGPS() {
  // Only report if we have valid GPS data
  if (gps.location.isValid()) {
    // Format: GPS,lat,lon,alt,speed,sats,hdop
    Serial.print("GPS,");
    Serial.print(gps.location.lat(), 6);  // Latitude with 6 decimal places
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);  // Longitude with 6 decimal places
    Serial.print(",");
    Serial.print(gps.altitude.meters());  // Altitude in meters
    Serial.print(",");
    Serial.print(gps.speed.kmph());       // Speed in km/h
    Serial.print(",");
    Serial.print(gps.satellites.value()); // Number of satellites
    Serial.print(",");
    Serial.print(gps.hdop.value());       // Horizontal dilution of precision
    Serial.println();
  } else {
    // Report GPS status if no fix
    Serial.println("GPS,NO_FIX");
  }
}