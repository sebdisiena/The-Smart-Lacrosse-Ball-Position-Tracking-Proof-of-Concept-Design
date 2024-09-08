#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 ballIMU = Adafruit_BNO055();

float roll;
float pitch;
float yaw;

// variables for accelerometer orientation
float pitch_accel;
float roll_accel;
float pitch_accel_filter_old = 0;
float pitch_accel_filter_new;
float roll_accel_filter_old = 0;
float roll_accel_filter_new;

// variables for gyroscope orientation 
float pitch_gyro = 0;
float roll_gyro = 0;
float yaw_gyro = 0;

// variables for magnetometer orientation
float x_mag;
float y_mag;
float pitch_rad;
float roll_rad;
float yaw_mag;

// variables used for integration
float dt;
unsigned long t_old;
unsigned long clocktime;

void setup(void)
{
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

   // raw accelerometer in x, y and z axes
  imu::Vector<3> acc = ballIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // initial roll and pitch calculations based on theory in report
  roll_accel = atan2(acc.y(), acc.z()) / 2 / 3.141592654 * 360; 
  pitch_accel = atan2(-acc.x(), sqrt(acc.y()*acc.y() + acc.z()*acc.z())) / 2 / 3.141592654 * 360;

  // low-pass filter to remove noise from initial orientation calculations
  roll_accel_filter_new = 0.95*roll_accel_filter_old + 0.05*roll_accel;
  pitch_accel_filter_new = 0.95*pitch_accel_filter_old + 0.05*pitch_accel;

  // raw gyroscope values used for roll pitch and yaw calculations
  imu::Vector<3> gyro = ballIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  dt = (millis() - t_old) / 1000.;
  t_old = millis();

  // equations outline in report to determine orientation values
  pitch_gyro = pitch_gyro + gyro.y() * dt;
  roll_gyro = roll_gyro + gyro.x() * dt;
  yaw_gyro = yaw_gyro + gyro.z() * dt;

  // get yaw from magnetometer
  imu::Vector<3> mag = ballIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  x_mag = mag.x() * cos(pitch_gyro) + mag.y() * sin(pitch_gyro) * sin(roll_gyro) - mag.z() * cos(roll_gyro) * sin(pitch_gyro);
  y_mag = mag.y() * cos(roll_gyro) + mag.z() * sin(roll_gyro);
  yaw_mag = atan2(y_mag, x_mag) / (2 * 3.141592654) * 360;

  // complementary filters for orientation values
  pitch = pitch_gyro * 0.98 + pitch_accel_filter_new * 0.02;
  roll = roll_gyro * 0.98 + roll_accel_filter_new * 0.02;
  yaw = yaw_gyro * 0.98 - yaw_mag * 0.02;

  // update old orentation values for low-pass filter
  pitch_accel_filter_old = pitch_accel_filter_new;
  roll_accel_filter_old = roll_accel_filter_new;
  
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.println(yaw);

}