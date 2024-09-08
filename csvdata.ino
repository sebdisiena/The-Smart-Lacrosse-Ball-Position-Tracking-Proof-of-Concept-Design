#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 ballIMU = Adafruit_BNO055();
unsigned long clocktime;

float x_line;
float y_line;
float z_line;

float x_gyro;
float y_gyro;
float z_gyro;

float x_mag;
float y_mag;
float z_mag;

float w;
float x;
float y;
float z;

void setup(void) {
  Serial.begin(115200);
  ballIMU.begin();
  delay(1000);
  ballIMU.setExtCrystalUse(true);
}

void loop(void) {
  uint8_t system, gyro, acc, mag = 0;
  ballIMU.getCalibration(&system, &gyro, &acc, &mag);

  imu::Vector<3> line_acc = ballIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x_line = line_acc.x();
  y_line = line_acc.y();
  z_line = line_acc.z();

  imu::Vector<3> gyr = ballIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  x_gyro = gyr.x();
  y_gyro = gyr.y();
  z_gyro = gyr.z();

  imu::Vector<3> mg = ballIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  x_mag = mg.x();
  y_mag = mg.y();
  z_mag = mg.z();

  imu::Quaternion quat = ballIMU.getQuat();
  w = quat.w();
  x = quat.x();
  y = quat.y();
  z = quat.z();

  clocktime = millis();
  sensors_event_t event;
  ballIMU.getEvent(&event);

  Serial.print(clocktime);
  Serial.print(",");

  Serial.print(x_line);
  Serial.print(",");
  Serial.print(y_line);
  Serial.print(",");
  Serial.print(z_line);
  Serial.print(",");

  Serial.print(x_gyro);
  Serial.print(",");
  Serial.print(y_gyro);
  Serial.print(",");
  Serial.print(z_gyro);
  Serial.print(",");

  Serial.print(x_mag);
  Serial.print(",");
  Serial.print(y_mag);
  Serial.print(",");
  Serial.print(z_mag);
  Serial.print(",");

  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.print(quat.z());
  Serial.print(",");

  Serial.print(acc);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mag);
  Serial.print(",");
  Serial.println(system);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}