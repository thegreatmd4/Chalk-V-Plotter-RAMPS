#include <AccelStepper.h>
#include <Servo.h>
#include <SD.h>

int Width =     1560;                         // Width of working area in *mm* (Distance between motors) < Length of the Blackboard
float x0 = 800 + 50.0 + 125.50;               // Offset of the initial position
float y0 = 820 - 22.5 + 160.93;               // Offset of the initial position

float sl0 = sqrt(pow((x0 - 125.50) , 2) + pow((y0 - 160.93) , 2));          // Initial Length of SL
float sr0 = -sqrt(pow((Width - x0 - 54.50) , 2) + pow((y0 - 160.93) , 2));   // Initial Length of SR

float k = 0.0;                                // Compensates the lost steps while letting go

int Speed =     500;                          // 500  < Speed     < 2000
int Speed_Max = 500;                          // 1000 < Speed_Max < 2000
int Accl =      1000;                          // 500  < Accl      < 2000
int Micro_Step_Rate = 4;                      // 8 means 6400 pulse/rev. (3 Jumpers)
//                                            // 4 means 3200 pulse/rev. (2 Jumpers)
//                                            // 2 means 1600 pulse/rev. (1 Jumpers)
//                                            // 1 means 800  pulse/rev. (0 Jumpers)

int PulsePerRev = 800 * Micro_Step_Rate;      // How many pulses should be sent to rotate 1 Revolution
int Pulley_Diameter = 34;                     // Diameter of the pulleys in *mm*
float mmPerRev = Pulley_Diameter * 3.14159;   // How much is the movement in mm per Revolution
float PulsePermm = PulsePerRev / mmPerRev;    // How many pulses should be sent to motors to move 1 mm

float Last_Pose_L = 0;                        // Last position of motor left
float Delta_L = 0;                            // Last deriven distance of motor left
float Last_Pose_R = 0;                        // Last position of motor right
float Delta_R = 0;                            // Last deriven distance of motor right

#define L_STEP_PIN         54                 // Pins of motor left (x in ramps)
#define L_DIR_PIN          55
#define L_ENABLE_PIN       38
#define L_MIN_PIN           3
#define L_MAX_PIN           2

#define R_STEP_PIN         60                 // Pins of motor right (y in ramps)
#define R_DIR_PIN          61
#define R_ENABLE_PIN       56
#define R_MIN_PIN          14
#define R_MAX_PIN          15

#define Z_STEP_PIN         46                 // Pins of motor Chalk (z in ramps)
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

const int chipSelect = 53;                    // CS pin of SD Card reader

bool busy = false;

AccelStepper Stepper_L(1, L_STEP_PIN, L_DIR_PIN);     // Define the actuators
AccelStepper Stepper_R(1, R_STEP_PIN, R_DIR_PIN);
AccelStepper Stepper_Z(1, Z_STEP_PIN, Z_DIR_PIN);

void setup() {
  Serial.begin(9600);

  if (!SD.begin(chipSelect)) {                        // Check if SD card is available
    Serial.println("SD card initialization failed!");
    while (1);                                        // Stop the program if isn't available
  }

  pinMode(L_STEP_PIN  ,       OUTPUT);                // Define PinModes of the Steppers
  pinMode(L_DIR_PIN    ,      OUTPUT);
  pinMode(L_ENABLE_PIN    ,   OUTPUT);
  pinMode(R_STEP_PIN  ,       OUTPUT);
  pinMode(R_DIR_PIN    ,      OUTPUT);
  pinMode(R_ENABLE_PIN    ,   OUTPUT);
  pinMode(Z_STEP_PIN  ,       OUTPUT);
  pinMode(Z_DIR_PIN    ,      OUTPUT);
  pinMode(Z_ENABLE_PIN    ,   OUTPUT);
  digitalWrite(L_ENABLE_PIN    , LOW);
  digitalWrite(R_ENABLE_PIN    , LOW);
  digitalWrite(Z_ENABLE_PIN    , LOW);

  Stepper_L.setMaxSpeed(Speed_Max);                   // Initialize the Steppers
  Stepper_L.setSpeed(Speed);
  Stepper_L.setAcceleration(Accl);
  Stepper_R.setMaxSpeed(Speed_Max);
  Stepper_R.setSpeed(Speed);
  Stepper_R.setAcceleration(Accl);
  Stepper_Z.setMaxSpeed(Speed_Max);
  Stepper_Z.setSpeed(Speed);
  Stepper_Z.setAcceleration(Accl);

  Stepper_L.setCurrentPosition(sl0*PulsePermm);       // The Arduino is told that the initial position is NOT (0,0)
  Stepper_R.setCurrentPosition(sr0*PulsePermm);
  Last_Pose_L = Stepper_L.currentPosition()/PulsePermm;
  Last_Pose_R = Stepper_R.currentPosition()/PulsePermm;
  Serial.print("The Position initialized: (x,y)=("); Serial.print(x0); Serial.print(","); Serial.print(y0); Serial.println(")");
  Serial.print("The Position initialized: (sl,sr)=("); Serial.print(Last_Pose_L); Serial.print(","); Serial.print(Last_Pose_R); Serial.println(")");
  Serial.println("-------------------------------------");
  
}

void loop() {
  File gcodeFile = SD.open("K.txt");                                          // S = Straight Line

  if (gcodeFile) {
    while (gcodeFile.available()) {
      String command = gcodeFile.readStringUntil('\n');
      Serial.println(command);
      processGCode(command);
    }
    gcodeFile.close();
  } else {
    Serial.println("Error opening G-code file!");
  }

  while (1) {
  }                                                                           // Do nothing after G-code execution
}

void processGCode(String command) {
  char Type = command.charAt(0);                                              // Type is usually G
  command.remove(0, 1);                                                       // Remove "G" from the line

  String x_command = command;
  x_command.remove(0, 10);                                                    // Delete first 10 letters from command line (to delete extra spaces)
  int index_X = x_command.indexOf('X');                                       // Position of letter "X"
  int index_space = x_command.indexOf(' ');                                   // Position of space
  float Pos_X = x_command.substring(index_X + 1, index_space - 1).toFloat();  // Extract X coordinate

  String y_command = command;
  y_command.remove(0, 20);                                                    // Delete first 22 letters from command line (to delete extra spaces)
  int index_Y = y_command.indexOf('Y');                                       // Position of letter "Y"
  int index_space2 = y_command.indexOf(' ');                                  // Position of space
  float Pos_Y = y_command.substring(index_Y + 1, index_space2 - 1).toFloat(); // Extract Y coordinate
    
  Serial.print("Target Position:   (x,y) =("); Serial.print(Pos_X); Serial.print(","); Serial.print(Pos_Y); Serial.println(")");
  

  // *********** Convert (x,y) --> (sl,sr) **************//
  
  float Pos_L = sqrt(pow((Pos_X - 125.50) , 2) + pow((Pos_Y - 160.93) , 2));            // Length of Rope Left   WITH RESPECT to the position of the chalk inside the mechanism.
  float Pos_R = -sqrt(pow((Width - Pos_X - 54.50) , 2) + pow((Pos_Y - 160.93) , 2));     // Length of Rope Right

  //float Pos_L = sqrt(pow((Pos_X) , 2) + pow((Pos_Y) , 2));                                // Length of Rope Left   REGARDLESS of the position of the chalk inside the mechanism.
  //float Pos_R = sqrt(pow((Width - Pos_X) , 2) + pow((Pos_Y) , 2));                        // Length of Rope Right

  Serial.print("Target Position:  (sl,sr)=("); Serial.print(Pos_L); Serial.print(","); Serial.print(Pos_R); Serial.println(")");


  // ************* Calculate Speeds ********************//
  Delta_L = Pos_L - Last_Pose_L;                                              // Change of len. of left. (mm)
  Last_Pose_L = Pos_L;
  Delta_R = Pos_R - Last_Pose_R;                                              // Change of len. of right. (mm)
  Last_Pose_R = Pos_R;

  Serial.print("Change of Length: (dsl,dsr)=("); Serial.print(Delta_L); Serial.print(","); Serial.print(Delta_R); Serial.println(")");
  
  int Speed_L = Speed_Max;
  int Speed_R = Speed_Max;
  
  if (abs(Delta_L) > abs(Delta_R)) {                                          // If left moves more,
    Speed_L = Speed_Max;                                                      // high speed left
    Speed_R = abs(Delta_R / Delta_L) * Speed_Max;                             // low speed right
  } else {                                                                    // If right moves more,
    Speed_L = abs(Delta_L / Delta_R) * Speed_Max;                             // low speed left
    Speed_R = Speed_Max;                                                      // high speed right
  }

  Serial.print("Speed of motors: (Vsl,Vsr)=("); Serial.print(Speed_L); Serial.print(","); Serial.print(Speed_R); Serial.println(")");
  
  moveSteppers(Pos_L, Pos_R, Speed_L, Speed_R, Delta_L, Delta_R);             // Move the steppers regarding positions and speeds
  Serial.println("-------------------------------------");
}

void moveSteppers(float l, float r, int Speed_L, int Speed_R, float Delta_L, float Delta_R) {
  if (!busy) {
    busy = true;                                // Set the flag to indicate motors are in movement

    l = l * PulsePermm + max(Delta_L,0)*k;                         // Convert Position (mm) to Command (Pulses) + Compensation
    r = r * PulsePermm + max(-Delta_R,0)*k;

    //Serial.print("Compensated: (l,r)=("); Serial.print(l); Serial.print(","); Serial.print(r); Serial.println(")");

    Stepper_L.setSpeed(Speed_L);                // Assign the speed of this line
    Stepper_R.setSpeed(Speed_R);
    Stepper_L.setMaxSpeed(Speed_L);
    Stepper_R.setMaxSpeed(Speed_R);

    Stepper_L.moveTo(l);                        // ABSOLUTE position
    Stepper_R.moveTo(r);

    Serial.println("Start Moving...");
    
    while (Stepper_L.distanceToGo() != 0 || Stepper_R.distanceToGo() != 0) {    // Continue moving as long as the motors did not reach final position
      Stepper_L.run();
      Stepper_R.run();

    }

    Serial.println("Both Reached!");
    
    busy = false;                                                               // Reset the flag when motors stop moving

  }
}
