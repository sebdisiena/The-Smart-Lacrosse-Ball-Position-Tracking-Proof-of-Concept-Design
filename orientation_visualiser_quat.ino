#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 ballIMU = Adafruit_BNO055();
unsigned long clocktime;

// system variables
float w;
float x;
float y;
float z;

void setup(void)
{
  Serial.begin(115200);
  ballIMU.begin();
  delay(1000);
  ballIMU.setExtCrystalUse(true);
}

void loop(void)
{
  uint8_t system, gyr, accel, mg = 0;
  ballIMU.getCalibration(&system, &gyr, &accel, &mg);
  imu::Quaternion quat = ballIMU.getQuat();

  w = quat.w();
  x = quat.x();
  y = quat.y();
  z = quat.z();

  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.println(quat.z());

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

