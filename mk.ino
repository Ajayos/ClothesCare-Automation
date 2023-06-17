#include <Stepper.h>

// Define the number of steps per revolution for the stepper motor
const int stepsPerRevolution = 200;

// Pin connections for rain and light sensors
const int rainSensorPin = A0;
const int lightSensorPin = A1;

// Threshold values for rain and sunlight detection
const int rainThreshold = 500;
const int lightThreshold = 200;

// Variables to store the current state of the door
boolean doorOpen = false;

// Pin connections for manual control
const int manualControlPinF = 2;  // Forward
const int manualControlPinR = 3;  // Reverse
const int manualControlPinS = 4;  // Stop

// Create an instance of the Stepper class
Stepper motor(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // Set the motor speed
  motor.setSpeed(100);
  
  // Set the rain and light sensor pins as inputs
  pinMode(rainSensorPin, INPUT);
  pinMode(lightSensorPin, INPUT);
  
  // Set the manual control pins as inputs with pull-up resistors
  pinMode(manualControlPinF, INPUT_PULLUP);
  pinMode(manualControlPinR, INPUT_PULLUP);
  pinMode(manualControlPinS, INPUT_PULLUP);
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
  
  // Manual control
  if (digitalRead(manualControlPinF) == LOW) {
    forward();
  } else if (digitalRead(manualControlPinR) == LOW) {
    reverse();
  } else if (digitalRead(manualControlPinS) == LOW) {
    stop();
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

// Function to move the motor forward manually
void forward() {
  digitalWrite(manualControlPinF, HIGH);
  digitalWrite(manualControlPinR, LOW);
  digitalWrite(manualControlPinS, LOW);
}

// Function to move the motor in reverse manually
void reverse() {
  digitalWrite(manualControlPinF, LOW);
  digitalWrite(manualControlPinR, HIGH);
  digitalWrite(manualControlPinS, LOW);
}

// Function to stop the motor
void stop() {
  digitalWrite(manualControlPinF, LOW);
  digitalWrite(manualControlPinR, LOW);
  digitalWrite(manualControlPinS, HIGH);
}
