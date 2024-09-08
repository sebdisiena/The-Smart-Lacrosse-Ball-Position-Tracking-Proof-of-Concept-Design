#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
 
#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 ballIMU= Adafruit_BNO055();
 
void setup() {
Serial.begin(115200);
ballIMU.begin();
delay(1000);
ballIMU.setExtCrystalUse(true);
}
 
void loop() {
uint8_t system, gyro, accel, mg = 0;
ballIMU.getCalibration(&system, &gyro, &accel, &mg); 
 
Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mg);
Serial.print(",");
Serial.println(system);
 
 
delay(BNO055_SAMPLERATE_DELAY_MS);
}