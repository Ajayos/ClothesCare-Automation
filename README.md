# ClothesCare-Automation
Automated weather-based clothes protection system using Arduino, rain/sunlight sensors, and a stepper motor.
**Project Note: Door Control System with Rain and Sunlight Detection**

Overview:
This project implements a door control system using an Arduino board. The system includes rain and sunlight detection capabilities to protect clothes from rain and lack of sunlight. The code controls a DC motor connected to the door mechanism, allowing automated opening and closing based on weather conditions. Additionally, manual control is provided to override the automated functionality.

Components:
- Arduino board (e.g., Arduino Uno)
- Rain sensor module
- Light sensor module
- DC motor
- Motor driver module
- Jumper wires
- Power supply (for Arduino board and motor)

Code Description:
The Arduino board reads analog values from the rain sensor and light sensor modules to determine weather conditions. The rain sensor, connected to pin A0, detects rain by measuring the analog output. The light sensor, connected to pin A1, detects sunlight based on the analog output. The code sets threshold values to determine rain and sunlight presence.

If rain is detected (analog value above the rain threshold), the code opens the door by running the motor in one direction. If no sunlight is detected (analog value below the light threshold), the code closes the door to protect from snow. The motor driver module is utilized to control the motor's direction and provide sufficient power.

Manual control is implemented through three digital pins. By sending signals to the respective pins, users can manually control the motor's forward, reverse, and stop functions, overriding the automated system.

Note:
This door control system with rain and sunlight detection offers an automated solution for protecting clothes from rain and lack of sunlight. The rain sensor detects rain, prompting the system to open the door. Similarly, the light sensor detects the absence of sunlight, leading to door closure for snow protection. Manual control is available for user intervention.

To use the system, assemble the required components and wire them according to the provided pin connections. Upload the provided Arduino code to the board and ensure that the necessary libraries for the sensors are installed. Connect an appropriate power supply for the Arduino and motor. Once powered on, the system will operate automatically based on weather conditions, while manual control can be utilized when desired.

Additional Code Parts:
1. Added manual control functionality using digital pins:
```cpp
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
```

2. Updated the main loop to include manual control:
```cpp
// Main loop
void loop() {
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
  if (digitalRead(manualControlPinF)) {
    forward();
  } else if (digitalRead(manualControlPinR)) {
    reverse();
  } else if (digitalRead(manualControlPinS)) {
    stop();
  }
}


```


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


