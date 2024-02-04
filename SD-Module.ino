#include <SD.h>

// Pin configuration for the SD card module on RAMPS
const int chipSelect = 53;

void setup() {
  Serial.begin(9600);

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  Serial.println("SD card initialized successfully!");

  // Read and print the G-code file
  
  readAndPrintGCode("G.txt");                                      // THIS HAS TO BE .GCODE NOT .TXT
}

void loop() {
  // Nothing to do in the loop for this example
}

void readAndPrintGCode(const char* filename) {
  // Open the G-code file
  File file = SD.open(filename);

  // Check if the file opened successfully
  if (file) {
    Serial.println("Printing contents of G-code file:");

    // Read and print the contents of the file
    while (file.available()) {
      Serial.write(file.read());
    }

    // Close the file
    file.close();
  } else {
    // If the file didn't open, print an error message
    Serial.println("Error opening G-code file!");
  }
}
