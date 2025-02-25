// Import required libraries
#include "AdvancedButton.h"
#include "ClearPathServo.h"

// Enable Program Options
// #define UseDynamicAcceleration  // Enable dynamic acceleration
// #define UseEnableFeedback       // Enable waiting for servo to enable

// Pin Definitions
const byte clearPathEnablePin = 13;   // Output pin for enable
const byte clearPathPulsePin = 5;     // Output pin for pulse
const byte clearPathDirectionPin = 6; // Output pin for direction
const byte clearPathHLFBPin = 7;      // Input pin for High Level Feedback (HLFB), This output pin on the servo should be configured to reflect the enabled state of the servo
const byte driverGoButtonPin = 8;     // Input pin for driver go button
const byte wheelSpeedPin = 3;         // Must be an interrupt pin, Uno = 2 or 3

// Program Constants
const static int MIN_BUTTON_DELAY_MS = 100;              // Minimum delay between button presses
const static int K_MPH_TO_IPM = 1056;                    // Converts MPH to in/min
const static int SERVO_PPR = 200;                        // Servo pulses per revolution
const static int MIN_ENABLE_DELAY_MS = 1000;             // Minimum delay after enabling/disabling servo
const static int MIN_ACCELERATION_PPS = 100;             // Minimum acceleration in PPS
const static int MAX_ACCELERATION_PPS = 5000;            // Maximum acceleration in PPS
const static int MAX_VEHICLE_SPEED_MPH = 20;             // Maximum vehicle speed in MPH
const static double WHEEL_CIRCUMFERENCE_INCH = 62.83185; // Wheel circumference in inches
const static float DRIVE_LINE_RATIO = 2.0;               // Drive line ratio 4:1 = 4.0, 2:1 = 2.0, etc...

// Calculated Constants
const static float MAX_VEHICLE_SPEED_IPM = MAX_VEHICLE_SPEED_MPH * K_MPH_TO_IPM;                          // Calculated max vehicle speed in IPM
const static float MAX_SERVO_RPM = (MAX_VEHICLE_SPEED_IPM / WHEEL_CIRCUMFERENCE_INCH) * DRIVE_LINE_RATIO; // Calculated max RPM of servo to reach max vehicle speed

// Create Objects
ClearPathServo servo(clearPathPulsePin, clearPathDirectionPin, clearPathEnablePin);
AdvancedButton driverGoButton(driverGoButtonPin, MIN_BUTTON_DELAY_MS);

// Program Variables
unsigned long prevWheelSpeedMillis = 0;       // Stores last time wheel speed was calculated
unsigned long prevSerialMillis = 0;           // Stores last time serial was printed
volatile double calculatedWheelSpeed_IPM = 0; // Stores calculated wheel speed in IPM
volatile double currentVehicleSpeed_MPH = 0;  // Stores current vehicle speed in MPH

/// Handle wheel speed calculation
/// NOTE: This function MUST be as short as possible to prevent issues with main code
void handleWheelSpeedInterrupt()
{
  unsigned long currentMillis = millis();

  // First interrupt only
  if (prevWheelSpeedMillis == 0)
  {
    prevWheelSpeedMillis = currentMillis;
    calculatedWheelSpeed_IPM = 0;
    currentVehicleSpeed_MPH = 0;
    return;
  }

  // Calculate time from last interrupt
  unsigned long timeFromLastInterrupt = currentMillis - prevWheelSpeedMillis;

  // Calculate wheel speed
  calculatedWheelSpeed_IPM = WHEEL_CIRCUMFERENCE_INCH / (timeFromLastInterrupt / 1000.0 / 60.0);
  currentVehicleSpeed_MPH = calculatedWheelSpeed_IPM / K_MPH_TO_IPM;

  // Save interrupt time
  prevWheelSpeedMillis = currentMillis;
}

/// Setup interrupts
void setupInterrupts()
{
  noInterrupts(); // Disable all interups

  // Setup wheel speed interrupt
  pinMode(wheelSpeedPin, INPUT_PULLUP); // Set pin as input with pullup, interrupt will trigger on LOW
  attachInterrupt(digitalPinToInterrupt(wheelSpeedPin), handleWheelSpeedInterrupt, LOW);

  interrupts(); // Re-enable interrupts
}

/// Handle servo enable/disable
void handleServoEnableDisable()
{

  // Enable Servo
  if (driverGoButton.pressed() && !servo.isRunning())
  {
    servo.enableTorque();

// Optionally wait for servo to enable
#ifdef UseEnableFeedback
    // Wait for servo to enable
    while (!digitalRead(clearPathHLFBPin) == HIGH)
    {
      delay(1);
    }
#endif

    delay(MIN_ENABLE_DELAY_MS);
  }

  // Disable Servo
  if (!driverGoButton.pressed() && !servo.isRunning())
  {
    servo.disableTorque();
    delay(MIN_ENABLE_DELAY_MS);
  }
}

// Initialize the program. Only runs once
void setup()
{
  Serial.begin(9600);

  // Init servo and buttons
  driverGoButton.begin();

  servo.setMaxSpeed(1000);
  servo.setAcceleration(200);

  setupInterrupts();
}

// Main program loop. Runs repeatedly
void loop()
{
  unsigned long currentMillis = millis();

  // Update button state
  driverGoButton.update();

  // Handle servo enable/disable
  handleServoEnableDisable();

  // Run servo when commanded
  bool run = driverGoButton.pressed() && servo.isEnabled();
  servo.runWhileButton(run);

  // Print debug info
  if ((currentMillis - prevSerialMillis) >= 250)
  {
    Serial.println("");
    Serial.println(servo.isRunning());
    Serial.println(servo.isAccelerating());
    Serial.println(servo.isAtSpeed());
    Serial.println(servo.isDecelerating());

    prevSerialMillis = currentMillis;
  }
}
