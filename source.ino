#include <Stepper.h>

// Define the number of steps per revolution for the stepper motor
const int stepsPerRevolution = 200;

// Create an instance of the Stepper class
Stepper motor(stepsPerRevolution, 8, 9, 10, 11);

// Pin connections for rain and light sensors
const int rainSensorPin = A0;
const int lightSensorPin = A1;

// Threshold values for rain and sunlight detection
const int rainThreshold = 500;
const int lightThreshold = 200;

// Variables to store the current state of the door
boolean doorOpen = false;

void setup() {
  // Set the motor speed
  motor.setSpeed(100);
  
  // Set the rain and light sensor pins as inputs
  pinMode(rainSensorPin, INPUT);
  pinMode(lightSensorPin, INPUT);
}

void loop() {
  // Read analog values from rain and light sensors
  int rainSensorValue = analogRead(rainSensorPin);
  int lightSensorValue = analogRead(lightSensorPin);
  
  // Check if it's raining
  if (rainSensorValue > rainThreshold) {
    if (!doorOpen) {
      openDoor();
    }
  }
  // Check if there is no sunlight (for snow protection)
  else if (lightSensorValue < lightThreshold) {
    if (doorOpen) {
      closeDoor();
    }
  }
}

// Function to open the door
void openDoor() {
  motor.step(stepsPerRevolution);
  doorOpen = true;
}

// Function to close the door
void closeDoor() {
  motor.step(-stepsPerRevolution);
  doorOpen = false;
}