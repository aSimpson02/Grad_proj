# Posture Correction 

This is my graduation project, which consists of three primary components that involve using various sensors and a micro vibrator motor with the Arduino platform. The code is divided into three files:

File 1: Sensor initialisation and data reading from BNO055 and Nano 33 BLE.
File 2: Testing the micro vibrator motor.
File 3: Integrating BNO055 sensor data with the vibrator motor.


## Code Structure

File 1: main_sensor_readings.ino
This file sets up and reads data from the BNO055 and Nano 33 BLE IMU sensors, which is then printed to the serial monitor.

Librarie:

* Wire.h
* Adafruit_Sensor.h
* Adafruit_BNO055.h
* Arduino_LSM9DS1.h
  
Setup:
* Initialises I2C interface.
* Sets up serial communication.
* Initialises BNO055 sensor.
* Initialises Nano 33 BLE IMU sensor.

Loop:
* Reads orientation data from the BNO055 sensor.
* Prints BNO055 data to the serial monitor.
* Reads and prints acceleration, gyroscope, and magnetic field data from the Nano 33 BLE IMU sensor.




File 2: motor_test.ino
This file tests the functionality of a micro vibrator motor connected to the Arduino.

Setup:
* Sets the motor pin as an output.


Loop:
* Turns the motor on for 1 second.
* Turns the motor off for 1 second.




File 3: sensor_and_motor_integration.ino
This file integrates data reading from the BNO055 sensor and uses the micro vibrator motor to give feedback based on the sensor's orientation data.

Libraries:
* Wire.h
* Adafruit_Sensor.h
* Adafruit_BNO055.h
* Arduino_LSM9DS1.h

Setup:
* Initialises the I2C interface.
* Sets up serial communication.
* Initialises the BNO055 sensor.
* Initialises the Nano 33 BLE IMU sensor.
* Sets the vibrator pin as the output and ensures it's initially off.

Loop:
* Reads orientation data from the BNO055 sensor.
* Prints BNO055 data to the serial monitor.
* Turns the vibrator motor on if the orientation is outside the specified range.
* Reads and prints acceleration and gyroscope data from the Nano 33 BLE IMU sensor.



## Usage
Requirements:
Check you have the Arduino IDE installed on your device.
Required libraries needed to upload this porject in the Arduino IDE:
* Adafruit_Sensor
* Adafruit_BNO055
* Arduino_LSM9DS1

## Steps
1. Upload main_sensor_readings.ino to your Arduino to check sensor/data readings.
2. Upload motor_test.ino to Arduino for testing the micro vibrator motor.
3. Upload sensor_and_motor_integration.ino to integrate sensor/data readings from the vibrator motor for feedback on user's orientation.

## Hardware Connections
Connect the BNO055 sensor to the Arduino via I2C (SDA and SCL).
Connect the Nano 33 BLE IMU to the Arduino via I2C (SDA and SCL).
Connect the micro vibrator motor to a digital pin (pin 4 in this proj) from a transistor to control the motor.
