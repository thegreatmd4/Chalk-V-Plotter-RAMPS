#include <Servo.h>

Servo myservo;  // Create a servo object to control a servo motor on pin 5

void setup() {
  myservo.attach(5);  // Attaches the servo on pin 5 to the servo object
}

void loop() {
  // Sweep the servo from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);  // Set the servo position
    delay(15);  // Wait for the servo to reach the position
  }

  // Sweep the servo from 180 to 0 degrees
  for (int pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);  // Set the servo position
    delay(15);  // Wait for the servo to reach the position
  }
}
