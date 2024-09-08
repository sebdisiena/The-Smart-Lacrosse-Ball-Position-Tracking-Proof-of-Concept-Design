#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define SDA_PIN 4
#define SCL_PIN 5

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

float sr;
float cr; 
float sp;
float cp; 
float sy; 
float cy; 

float R11;
float R12;
float R13;
float R21;
float R22;
float R23;
float R31;
float R32;
float R33;

// float R[3][3] = {{R11, R12, R13}, {R21, R22, R23}, {R31, R32, R33}};
// float V[3][1] = {{1}, {0}, {0}};
// float RV[3][1] = {{0}, {0}, {0}}; 

float xpos;
float ypos;
float zpos;

float acc[3];
float invT[3][3];

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
  float lineacc[3] = {x_line,y_line,z_line};
  imu::Quaternion quat = ballIMU.getQuat();

  w = quat.w();
  x = quat.x();
  y = quat.y();
  z = quat.z();

  roll = (-atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));// * 180 / 3.14;
  pitch = (asin(2 * (w * y - z * x)));// * 180 / 3.14;
  yaw = (-atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));// * 180 / 3.14;

  sr = sin(roll);
  cr = cos(roll);
  sp = sin(pitch);
  cp = cos(pitch);
  sy = sin(yaw);
  cy = cos(yaw);

  R11 = cy * cp;
  R12 = cy * sp * sr - sy * cr;
  R13 = cy * sp * cr + sy * sr;
  R21 = sy * cp;
  R22 = sy * sp * sr + cy * cr;
  R23 = sy * sp * cr - cy * sr;
  R31 = -sp;
  R32 = cp * sr;
  R33 = cp * cr;

  float R[3][3] = {{R11, R12, R13}, {R21, R22, R23}, {R31, R32, R33}};
  float V[3][1] = {{1}, {0}, {0}};
  float RV[3][1] = {{0}, {0}, {0}}; 

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        RV[i][j] += R[i][k] * V[k][j];
      }
    }
  }

  float Tinv[3][3];
  float det = RV[0][0] * (RV[1][1] * RV[2][2] - RV[2][1] * RV[1][2])- RV[0][1] * (RV[1][0] * RV[2][2] - RV[2][0] * RV[1][2])+ RV[0][2] * (RV[1][0] * RV[2][1] - RV[2][0] * RV[1][1]);

  Tinv[0][0] = (RV[1][1] * RV[2][2] - RV[2][1] * RV[1][2]) / det;
  Tinv[0][1] = -(RV[0][1] * RV[2][2] - RV[2][1] * RV[0][2]) / det;
  Tinv[0][2] = (RV[0][1] * RV[1][2] - RV[1][1] * RV[0][2]) / det;
  Tinv[1][0] = -(RV[1][0] * RV[2][2] - RV[2][0] * RV[1][2]) / det;
  Tinv[1][1] = (RV[0][0] * RV[2][2] - RV[2][0] * RV[0][2]) / det;
  Tinv[1][2] = -(RV[0][0] * RV[1][2] - RV[1][0] * RV[0][2]) / det;
  Tinv[2][0] = (RV[1][0] * RV[2][1] - RV[2][0] * RV[1][1]) / det;
  Tinv[2][1] = -(RV[0][0] * RV[2][1] - RV[2][0] * RV[0][1]) / det;
  Tinv[2][2] = (RV[0][0] * RV[1][1] - RV[1][0] * RV[0][1]) / det;  



  // Print the acceleration vector acc
  for (int i = 0; i < 3; i++) {
    Serial.print(acc[i]);
    Serial.print("\t");
  }  
  
  clocktime = millis();
  sensors_event_t event;
  ballIMU.getEvent(&event);
 
  // Serial.print(RV[0][0]);
  // Serial.print(",");
  // Serial.print(RV[1][0]);
  // Serial.print(",");
  // Serial.println(RV[2][0]);

  // Serial.print(",");  
  // Serial.print(roll);
  // Serial.print(",");
  // Serial.print(pitch);
  // Serial.print(",");
  // Serial.println(yaw);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}