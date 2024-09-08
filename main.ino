#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
// #define SDA_PIN 4
// #define SCL_PIN 5

Adafruit_BNO055 ballIMU = Adafruit_BNO055();
unsigned long clocktime;

// accelerometer variables
float pitch_acc_measured;
float roll_acc_measured;
float pitch_acc_filter_old = 0;
float pitch_acc_filter_new;
float roll_acc_filter_old = 0;
float roll_acc_filter_new;
float x_line;
float y_line;
float z_line;

// gyroscope variables
float pitch_gyro = 0;
float roll_gyro = 0;

// magnetometer variables
float x_mag;
float y_mag;
float pitch_rad;
float roll_rad9;

// system variables
float w;
float x;
float y;
float z;
float pitch;
float roll;
float yaw;

// variables to deterimine tiime
float dt;
unsigned long t_old;

void setup(void)
{
  // Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  ballIMU.begin();
  delay(1000);
  ballIMU.setExtCrystalUse(true);
  t_old = millis();
}

void loop(void)
{
  uint8_t system, gyr, accel, mg = 0;
  ballIMU.getCalibration(&system, &gyr, &accel, &mg);
  imu::Vector<3> line_acc = ballIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x_line = line_acc.x();
  y_line = line_acc.y();
  z_line = line_acc.z();
  imu::Quaternion quat = ballIMU.getQuat();

  w = quat.w();
  x = quat.x();
  y = quat.y();
  z = quat.z();

  roll = (-atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))) * 180 / 3.14;
  pitch = (asin(2 * (w * y - z * x))) * 180 / 3.14;
  yaw = (-atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) * 180 / 3.14;

  clocktime = millis();
  sensors_event_t event;
  ballIMU.getEvent(&event);

  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.print(quat.z());
  Serial.print(",");
  Serial.print(x_line);
  Serial.print(",");
  Serial.print(y_line);
  Serial.print(",");
  Serial.print(z_line);
  Serial.print(",");
  Serial.print(clocktime);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}