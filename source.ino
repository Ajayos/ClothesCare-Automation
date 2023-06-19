#include <Wire.h>

const int lightSensorAddr = 0x23;  // I2C address of the light sensor
const int lightThreshold = 100;    // Light threshold value
const int rainSensorPin = A0;      // Pin connected to the rain sensor
const int forwardButtonPin = 6;   // Pin connected to the forward button
const int backwardButtonPin = 7;  // Pin connected to the backward button

// Define the pin connections for the stepper motor driver
const int stepPin = 2;
const int dirPin = 3;
const int sleepPin = 4;  // Sleep control pin
const int resetPin = 5;  // Reset control pin

const int mk = 8;

const int rainThreshold = 700;     // Threshold value for rain detection

const int stepsPerRevolution = 200; // Define the steps per revolution for the stepper motor

const int stepDelay = 2000; // Define the delay between steps (in microseconds)

boolean motorOn = false; // Motor status
boolean isLight = false; // Light status
boolean saveMovement = false; // Save movement for automatic backward operation
boolean forwardButtonPressed = false; // Forward button state
boolean backwardButtonPressed = false; // Backward button state

void setup() {
  Wire.begin();                  // Initialize I2C communication
  pinMode(rainSensorPin, INPUT); // Set the rain sensor pin as input
  pinMode(forwardButtonPin, INPUT_PULLUP); // Set the forward button pin as input
  pinMode(backwardButtonPin, INPUT_PULLUP); // Set the backward button pin as input
  pinMode(stepPin, OUTPUT);      // Set the step pin as output
  pinMode(dirPin, OUTPUT);       // Set the direction pin as output
  pinMode(sleepPin, OUTPUT);     // Set the sleep pin as output
  pinMode(resetPin, OUTPUT);     // Set the reset pin as output
   pinMode(mk, OUTPUT);     // Set the "On" indicator LED pin as output
  // pinMode(rainLightPin, OUTPUT);     // Set the "Rain" indicator LED pin as output
  // pinMode(lightLightPin, OUTPUT);     // Set the "Light" indicator LED pin as output
  // pinMode(forwardLightPin, OUTPUT);     // Set the "Forward" indicator LED pin as output
  // pinMode(backwardLightPin, OUTPUT);     // Set the "Backward" indicator LED pin as output
}

void loop() {
  int rainSensorValue = analogRead(rainSensorPin);  // Read the rain sensor value
  int lightLevel = getLightLevel();                 // Read the light level

  if (lightLevel > lightThreshold) {
    isLight = true;
    // digitalWrite(rainLightPin, HIGH); // Turn on the "Rain" indicator LED
  } else {
    isLight = false;
    // digitalWrite(rainLightPin, LOW); // Turn off the "Rain" indicator LED
  }

  forwardButtonPressed = digitalRead(forwardButtonPin) == LOW;
  backwardButtonPressed = digitalRead(backwardButtonPin) == LOW;

  if (forwardButtonPressed) {
    digitalWrite(mk, LOW); // Turn on the "On" indicator LED
    
    forward();  // Run motor forward
    saveMovement = true;
  }
  if (backwardButtonPressed) {
    
    digitalWrite(mk, HIGH); // Turn on the "On" indicator LED
    backward(); // Run motor backward
    saveMovement = false;
  }
  if (rainSensorValue < rainThreshold) {
    if (isLight && !motorOn) {
      
      forward();  // Run motor forward
      motorOn = true;
      saveMovement = true;
    }
  } else {
    if (isLight && motorOn && !saveMovement) {
      backward();  // Run motor backward
      motorOn = false;
    }
  }

  delay(500);  // Delay for stability
}

int getLightLevel() {
  Wire.beginTransmission(lightSensorAddr);
  Wire.write(0x10);  // Send measurement command
  Wire.endTransmission();

  Wire.requestFrom(lightSensorAddr, 2);
  if (Wire.available()) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int lightLevel = (highByte << 8) + lowByte;
    return lightLevel;
  }

  return 0;  // Return 0 if no data received
}

void forward() {
  // digitalWrite(forwardLightPin, HIGH); // Turn on the "Forward" indicator LED

  digitalWrite(sleepPin, HIGH);  // Set sleep pin to HIGH to keep the motor awake initially
  digitalWrite(resetPin, HIGH);

  digitalWrite(dirPin, HIGH);  // Set the motor direction to forward

  for (int i = 0; i < (10 * stepsPerRevolution); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  digitalWrite(sleepPin, LOW);  // Put the motor to sleep
  digitalWrite(resetPin, LOW);

  // digitalWrite(forwardLightPin, LOW); // Turn off the "Forward" indicator LED
}

void backward() {
  // digitalWrite(backwardLightPin, HIGH); // Turn on the "Backward" indicator LED

  digitalWrite(sleepPin, HIGH);  // Set sleep pin to HIGH to keep the motor awake initially
  digitalWrite(resetPin, HIGH);

  digitalWrite(dirPin, LOW);  // Set the motor direction to backward

  for (int i = 0; i < (10 * stepsPerRevolution); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  digitalWrite(sleepPin, LOW);  // Put the motor to sleep
  digitalWrite(resetPin, LOW);

  // digitalWrite(backwardLightPin, LOW); // Turn off the "Backward" indicator LED
}
