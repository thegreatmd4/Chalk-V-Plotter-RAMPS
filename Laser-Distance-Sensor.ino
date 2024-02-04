#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  Serial.println("VL53L0X-Sensor bereit!");
}

void loop() {
  int distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout! ");
  }

  Serial.print("Abstand: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(1000);
}
