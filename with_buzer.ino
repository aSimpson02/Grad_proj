//importng libraries 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino_LSM9DS1.h>

//BNO055 I2C address
#define BNO055_ADDRESS 0x28

//BNO055 object
Adafruit_BNO055 bno055 = Adafruit_BNO055(BNO055_ADDRESS);

//assigning buzzer pin
#define vibrator 4

void setup() {
  //I2C interface
  Wire.begin();

  //serial communication
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing BNO055...");

  //checking BNO055 sensor works 
  if (!bno055.begin()) {
    Serial.println("Error initializing BNO055!");
    while (1);
  } else {
    Serial.println("BNO055 initialized successfully!");
  }

  // checking Nano 33 BLE serial data works 
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  } else {
    Serial.println("IMU initialized successfully!");
  }

  //buzzer pin
  pinMode(vibrator, OUTPUT);
  digitalWrite(vibrator, LOW);
}

void loop() {
  //data from BNO055 sensor
  imu::Vector<3> euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);

  //BNO055 to serial monitor
  Serial.print("BNO055 Orientation (degrees): ");
  Serial.print(euler.x());
  Serial.print(", ");
  Serial.print(euler.y());
  Serial.print(", ");
  Serial.println(euler.z());



  
  if (euler.y() > -13 || euler.y() < -15) {
    digitalWrite(vibrator, HIGH); 
  } else {  
    digitalWrite(vibrator, LOW); 
  }


  float x, y, z;

  //accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.print("Nano 33 BLE Acceleration (m/s^2): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  // gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    Serial.print("Nano 33 BLE Gyroscope (degrees/s): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }



  // delay for serial output
  //delay(100);
}
