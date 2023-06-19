# ClothesCare-Automation
Automated weather-based clothes protection system using Arduino, rain/sunlight sensors, and a stepper motor.

# Door Control System with Rain and Sunlight Detection

The Door Control System with Rain and Sunlight Detection is an automated system designed to protect clothes from rain and snow. The project utilizes an Arduino board, rain and sunlight sensors, a stepper motor, a stepper motor driver module, and a chain drive mechanism to control the opening and closing of a door based on weather conditions. The system provides both automated and manual control options.

## Components Used

- Arduino board (e.g., Arduino Uno)
- Rain sensor module
- Light sensor module
- Stepper motor
- Stepper motor driver module
- Chain drive mechanism (e.g., sprockets, chain, and guides)
- Jumper wires
- Power supply (for Arduino board and motor)

## Description

The Door Control System with Rain and Sunlight Detection is designed to protect clothes from rain and snow by automating the opening and closing of a door using a chain drive mechanism. The system utilizes an Arduino board, rain and sunlight sensor modules, a stepper motor, a stepper motor driver module, and the chain drive components. The Arduino code reads analog values from the rain and sunlight sensors to determine weather conditions and control the stepper motor, which in turn operates the chain drive mechanism.

### Arduino Code Functionality

The Arduino code implements the following functionality:

- Reading analog values from the rain and sunlight sensor modules to detect weather conditions.
- Opening the door if rain is detected above the rain threshold.
- Closing the door if no sunlight is detected below the light threshold.
- Providing manual control options to override the automated system.

### Wiring and Ports

To connect the components, use the following wiring and ports:

#### Chain Drive Mechanism

- Connect the stepper motor to the chain drive mechanism, ensuring proper alignment and tension.
- Install sprockets and guides to guide the chain and provide smooth movement.

#### Wiring Connections

- Rain Sensor:
  - Connect the VCC pin of the rain sensor to the 5V pin on the Arduino board.
  - Connect the GND pin of the rain sensor to the GND pin on the Arduino board.
  - Connect the analog output pin of the rain sensor to any analog input pin on the Arduino board.

- Sunlight Sensor:
  - Connect the VCC pin of the sunlight sensor to the 5V pin on the Arduino board.
  - Connect the GND pin of the sunlight sensor to the GND pin on the Arduino board.
  - Connect the analog output pin of the sunlight sensor to any analog input pin on the Arduino board.

- Stepper Motor:
  - Connect the stepper motor's coil wires to the corresponding pins on the stepper motor driver module. Refer to the datasheet or documentation of your stepper motor for the specific wire configuration.
  - Connect the stepper motor driver module's control pins (STEP, DIR, and EN) to digital output pins on the Arduino board. Note the chosen pins for each connection.

- Stepper Motor Driver Module:
  - Connect the VCC and GND pins of the stepper motor driver module to the 5V and GND pins on the Arduino board, respectively.
  - Connect the control pins (STEP, DIR, and EN) of the stepper motor driver module to digital output pins on the Arduino board. Note the chosen pins for each connection.

#### Manual Control Ports

- Manual Button:
  - Connect one terminal of the manual button to a digital input pin on the Arduino board.
  - Connect the other terminal of the manual button to the GND pin on the Arduino board.

- Manual Potentiometer:
  - Connect one end of the potentiometer to the 5V.

# Door Control System with Rain and Sunlight Detection

This project implements a door control system using an Arduino board. The system includes rain and sunlight detection capabilities to protect clothes from rain and lack of sunlight. The code controls a stepper motor connected to a chain drive mechanism, allowing automated opening and closing based on weather conditions. Additionally, manual control is provided to override the automated functionality.


## Project Overview

The weather-based door control system is designed to protect the door and its contents from rain and snow. The system detects weather conditions using the rain and sunlight sensors. When rain is detected, the stepper motor is activated to open the Huifebg umbrella door mechanism, providing a protective cover for the door. Similarly, when there is no sunlight detected (indicating snowfall), the door mechanism closes to prevent snow from entering.

The code in this repository, written in Arduino programming language, controls the stepper motor and communicates with the rain and sunlight sensors. It uses sensor readings to determine the appropriate actions for the door mechanism. Manual control functionality is also implemented, allowing users to override the automated system and control the stepper motor manually if needed.


![Arduino](images/Arduino.jpg) ![Rain Sensor](images/rain_sensor.jpg) ![Sunlight Sensor](images/sunlight_sensor.jpg) ![Stepper Motor](images/stepper_motor.jpg) ![Motor Driver](images/motor_driver.jpg) ![Huifebg Umbrella Door](images/Jumper_wires.jpg)


## Components Used

### Arduino Board

The Arduino board acts as the central control unit for the project. It runs the Arduino code and facilitates communication between the sensors and the stepper motor. The program logic is implemented on the Arduino board, making it the brain of the weather-based door control system.

### Rain Sensor

The rain sensor is used to detect the presence of rain. It measures the moisture level in the environment and provides an analog output signal that is read by the Arduino board. Based on the readings, the Arduino determines whether to open the door or not to protect it from rain.

### Sunlight Sensor

The sunlight sensor detects the intensity of sunlight. It provides an analog output that is read by the Arduino board. When the sunlight sensor detects a low light intensity, indicating the absence of sunlight (e.g., during snowfall), the Arduino triggers the closing of the door mechanism to prevent snow from entering.

### Stepper Motor

The stepper motor is responsible for the movement of the door mechanism. It provides precise control over the opening and closing actions. The Arduino board communicates with the stepper motor via a motor driver module to command the desired rotations and direction. By controlling the stepper motor, the system can automate the door operation based on the weather conditions.

### Stepper Motor Driver Module

The stepper motor driver module acts as an interface between the Arduino board and the stepper motor. It receives commands from the Arduino and provides the necessary current and voltage levels to drive the stepper motor. The driver module ensures smooth and controlled movement of the motor, enabling precise door control in response to the weather conditions.

### Huifebg Umbrella Door Mechanism

The Huifebg umbrella door mechanism is a specialized component designed to provide weather protection for the door. It consists of an umbrella-like cover that can be opened or closed. When activated by the stepper motor, the umbrella door opens to shield the door and its contents from rain. Conversely, it closes to prevent snow from entering during snowfall. The Huifebg umbrella door mechanism ensures the safety and integrity of the door under various weather conditions.

## Source Code

```cpp
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
```

## Code Explanation

The code consists of the following main sections:

### Libraries

The required libraries are included at the beginning of the code. In this project, the `Stepper` library is used to control the stepper motor.

### Variable Declarations

- `stepsPerRevolution`: This variable holds the number of steps per revolution for the stepper motor.

### Object Creation

- `motor`: An instance of the `Stepper` class is created using the `stepsPerRevolution` variable and specifying the pin connections for the motor coils.

### Pin Declarations

- `rainSensorPin`: The pin to which the rain sensor module is connected.
- `lightSensorPin`: The pin to which the light sensor module is connected.

### Thresholds

- `rainThreshold`: The threshold value for rain detection.
- `lightThreshold`: The threshold value for sunlight detection.

### Setup Function

The `setup()` function is called once when the Arduino board is powered on. It initializes the motor speed and sets the rain and light sensor pins as inputs.

### Loop Function

The `loop()` function is the main execution loop of the program.