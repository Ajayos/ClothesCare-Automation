#include <Stepper.h>

// Constants
const int stepsPerRevolution = 200;   // Number of steps per motor revolution
const int rainSensorPin = A0;         // Rain sensor analog input pin
const int lightSensorPin = A1;        // Light sensor analog input pin
const int rainThreshold = 500;        // Rain sensor threshold value
const int lightThreshold = 300;       // Light sensor threshold value

// Motor Configuration
const int motorSteps = 200;           // Number of steps per revolution for the stepper motor
const int motorPin1 = 2;              // Motor driver module input pin 1 (STEP)
const int motorPin2 = 3;              // Motor driver module input pin 2 (DIR)

// Create stepper motor object
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2);

// Function to open the door
void openDoor() {
  myStepper.setSpeed(100); // Set the speed of the stepper motor (adjust as needed)
  myStepper.step(stepsPerRevolution); // Rotate the motor one revolution to open the door
}

// Function to close the door
void closeDoor() {
  myStepper.setSpeed(100); // Set the speed of the stepper motor (adjust as needed)
  myStepper.step(-stepsPerRevolution); // Rotate the motor one revolution in the opposite direction to close the door
}

void setup() {
  // Initialize serial communication if needed
  // Serial.begin(9600);

  // Set the motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  // Read sensor values
  int rainSensorValue = analogRead(rainSensorPin);
  int lightSensorValue = analogRead(lightSensorPin);

  // Check if it's raining
  if (rainSensorValue > rainThreshold) {
    // Open the door
    openDoor();
  }
  // Check if there is no sunlight (for snow protection)
  else if (lightSensorValue < lightThreshold) {
    // Close the door
    closeDoor();
  }

  // Delay between sensor readings (adjust as needed)
  delay(1000);
}
