#include "Arduino.h"
#include "MPU9250_ea.h"

//#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 22
#define SCL_PIN 23
//#endif

MPU9250_ea mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  delay(2000);
  Serial.println("started");

//#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C
  Wire.setTimeOut(1000); // Timeout in ms
  mySensor.setWire(&Wire);

//#endif
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  uint8_t sensorId;
  int result;

  result = mySensor.readId(&sensorId);
  if (result == 0) {
    //Serial.println("sensorId: " + String(sensorId));
  } else {
    //Serial.println("Cannot read sensorId " + String(result));
  }

  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    //Serial.println("accelX: " + String(aX));
    //Serial.println("accelY: " + String(aY));
    //Serial.println("accelZ: " + String(aZ));
    //Serial.println("accelSqrt: " + String(aSqrt));
  } else {
    Serial.println("Cannod read accel values " + String(result));
  }

  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    //Serial.println("gyroX: " + String(gX));
    //Serial.println("gyroY: " + String(gY));
    //Serial.println("gyroZ: " + String(gZ));
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }

  result = mySensor.magUpdate();
  if (result != 0) {
    //Serial.println("cannot read mag so call begin again");
    mySensor.beginMag();
    result = mySensor.magUpdate();
  }
  if (result == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("horizontal direction: " + String(mDirection));
  } else {
    //Serial.println("Cannot read mag values " + String(result));
  }

  Serial.print("ACC:");
  Serial.print(aX);
  Serial.print(",");
  Serial.print(aY);
  Serial.print(",");
  Serial.print(aZ);
  Serial.print(";GYRO:");
  Serial.print(gX);
  Serial.print(",");
  Serial.print(gY);
  Serial.print(",");
  Serial.print(gZ);
  Serial.print(";TEMP:");
  Serial.println(0.0);

  //Serial.println("at " + String(millis()) + "ms");
  //Serial.println(""); // Add an empty line
  delay(500);
}