#include <AccelStepper.h>

int Speed_xy =     500;          //   < Speed_xy     <
int Speed_xy_Max = 500;          //   < Speed_xy_Max <
int Accl_xy =      500;          //   < Accl_xy      <

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

AccelStepper Stepper_X(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper Stepper_Y(1, Y_STEP_PIN, Y_DIR_PIN);

void setup() {
  pinMode(X_STEP_PIN  ,       OUTPUT);
  pinMode(X_DIR_PIN    ,      OUTPUT);
  pinMode(X_ENABLE_PIN    ,   OUTPUT);

  pinMode(Y_STEP_PIN  ,       OUTPUT);
  pinMode(Y_DIR_PIN    ,      OUTPUT);
  pinMode(Y_ENABLE_PIN    ,   OUTPUT);

  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);

  Stepper_X.setMaxSpeed(Speed_xy_Max);
  Stepper_X.setSpeed(Speed_xy);
  Stepper_X.setAcceleration(Accl_xy);

  Stepper_Y.setMaxSpeed(Speed_xy_Max);
  Stepper_Y.setSpeed(Speed_xy);
  Stepper_Y.setAcceleration(Accl_xy);

  Serial.begin(9600);
  
}

bool busy = false;

void loop() {
  
  if (!busy && Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int x = input.substring(0, input.indexOf(',')).toInt();
    int y = input.substring(input.indexOf(',') + 1).toInt();

    x = x;
    y = -y;

    Serial.println("Current Position:");
    Serial.println(x);
    Serial.println(y);
    busy = true;  // Set the flag to indicate motors are in movement
    
    Stepper_X.moveTo(x);        // ABSOLUTE position
    Stepper_Y.moveTo(y);
    while (Stepper_X.distanceToGo() != 0 || Stepper_Y.distanceToGo() != 0) {
      Stepper_X.run();
      Stepper_Y.run();
    }

    busy = false;  // Reset the flag when motors stop moving

    // Clear any remaining serial data
    while (Serial.available() > 0) {
      Serial.read();
      Serial.println("Busy! Try again later!");
    }
  }
  
}
