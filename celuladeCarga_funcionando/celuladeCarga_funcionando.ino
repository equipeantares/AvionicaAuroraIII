#include "HX711.h"
#include <SD.h>
#include <SPI.h>
#include <stdio.h>

#define calibration_factor 870 // Fator de calibração obtido previamente em outro código
#define LOADCELL_DOUT_PIN  A1
#define LOADCELL_SCK_PIN  A0

#define SD_CS_PIN 10 // Pin onde se conecta pino CS do leitor SD

char contagem = 0;
HX711 scale;
File dataFile;

void write_on_SD(float dado) {
  if (dataFile) {
    dataFile.print(dado);
    dataFile.print(" N");
    dataFile.print("\n");
    dataFile.flush(); // Ensure the data is written to the file
  } else {
    Serial.println("Error: Could not write to SD card.");
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 scale demo");

  // Initialize HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); // This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); // Assuming there is no weight on the scale at start up, reset the scale to 0
  Serial.begin(9600);
  Serial.println("HX711 scale demo");

  // Initialize HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); // This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); // Assuming there is no weight on the scale at start up, reset the scale to 0

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    //Serial.println("Error initializing SD card.");
    return;
  } else {
    Serial.println("SD card initialized successfully.");
  }

  // Open or create the file on the SD card
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening data.txt on SD card.");
    return;
  } else {
    Serial.println("data.txt opened successfully.");
  }

  Serial.println("Readings:");
}

void taragem() {
  if (Serial.available()) {
    char cara = Serial.read();
    if (cara == 't') {
      scale.tare();
      Serial.println("Scale tared.");
    }
  }
}

void loop() {
  float dado = scale.get_units();
  Serial.print("Reading: ");
  Serial.print(dado, 3); // scale.get_units() returns a float
  Serial.print(" N"); // You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();
  
  write_on_SD(dado);
  taragem();
}
