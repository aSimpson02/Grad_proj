//libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino_LSM9DS1.h>

//BNO055 I2C address
#define BNO055_ADDRESS 0x28

//BNO055 object
Adafruit_BNO055 bno055 = Adafruit_BNO055(BNO055_ADDRESS);

void setup() {
  //I2C interface
  Wire.begin();

  //serial monitor
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing BNO055...");

  //BNO055 sensor
  if (!bno055.begin()) {
    Serial.println("Error initializing BNO055!");
    while (1);
  } else {
    Serial.println("BNO055 initialized successfully!");
  }

  //Nano 33 BLE IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  } else {
    Serial.println("IMU initialized successfully!");
  }
}

void loop() {
  //BNO055 sensr data
  imu::Vector<3> euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Print BNO055 data
  Serial.print("BNO055 Orientation (degrees): ");
  Serial.print(euler.x());
  Serial.print(", ");
  Serial.print(euler.y());
  Serial.print(", ");
  Serial.println(euler.z());


  float x, y, z;

  //nano 33 ble accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.print("Nano 33 BLE Acceleration (m/s^2): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  //nano 33 ble gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    Serial.print("Nano 33 BLE Gyroscope (degrees/s): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  //nano 33 ble magnometer data
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    Serial.print("Nano 33 BLE Magnetic Field (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  delay(100);
}
