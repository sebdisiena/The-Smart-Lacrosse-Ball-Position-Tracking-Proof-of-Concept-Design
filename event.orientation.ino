#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
float roll, pitch, yaw

void setup() {
  Serial.begin(9600);
  Serial.println("Orientation Test");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  roll = event.orientation.x
  pitch = event.orientation.y
  yaw = event.orientation.z
  Serial.print("Orientation: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.println(event.orientation.z);

  delay(100);
}

