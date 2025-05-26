#include "smv_accel.h"

// -----------------------------------------------------------------------------------------------
// THIS IS A BASIC EXAMPLE .INO
// MODIFY THIS FILE TO CODE YOUR DEVICE
// -----------------------------------------------------------------------------------------------

// Define SPI Chip Select pin for RP2040
const int CS_PIN = 13;

ASM330LHH sensor(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Test!");
  sensor.begin();
}

void loop() {
    sensor.printSensorData();
    delay(250);
}

