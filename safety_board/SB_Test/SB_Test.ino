#include "SMVcanbus.h"

CANBUS can(Safety);
const int IGNITION_PIN = 25;
const int RP2040_ADC_PIN_ZERO = A0;
const int RP2040_ADC_PIN_ONE = A1;
const int RP2040_ADC_PIN_THREE = A3;

const double RP2040_ADC_PIN_ZERO_REGULAR_VOLTAGE = 3;
const double RP2040_ADC_PIN_ONE_REGULAR_VOLTAGE = 3;
const double RP2040_ADC_PIN_THREE_REGULAR_VOLTAGE = 3;
const double RP2040_ADC_PIN_ZERO_VOLTAGE_THRESHOLD = 2.9;
const double RP2040_ADC_PIN_ONE_VOLTAGE_THRESHOLD = 2.9;
const double RP2040_ADC_PIN_THREE_VOLTAGE_THRESHOLD = 2.9;

void setup(void) {
  Serial.begin(115200);
  can.begin();
  delay(400); // for printing
  pinMode(IGNITION_PIN, OUTPUT);
  digitalWrite(IGNITION_PIN, LOW);
  analogReadResolution(12);    //set for 12 bit
  

  Serial.println("setup2");
}

void loop() {
  // sets the ignition pin to high if we get a signal to start ignition from UI board
  can.looper();
  if(can.isThere()) {
    if(strcmp(can.getHardware(), "UI") == 0 && strcmp(can.getDataType(), "Motor") == 0) {
      if(can.getData() == 1) {
        digitalWrite(IGNITION_PIN, HIGH);
      } else if (can.getData() == 0) {
        digitalWrite(IGNITION_PIN, LOW);
      }
      // Serial.println(can.getHardware());
      // Serial.println(can.getDataType());
      // Serial.println(can.getData());
    }
  }

  double p0 = analogRead(RP2040_ADC_PIN_ZERO) * (3.3 / 4096.0);
  double p1 = analogRead(RP2040_ADC_PIN_ONE) * (3.3 / 4096.0);
  double p3 = analogRead(RP2040_ADC_PIN_THREE) * (3.3 / 4096.0);

  // Serial.println("P0: " + String(p0));
  // Serial.println("P1: " + String(p1));
  // Serial.println("P3: " + String(p3));

  // sets ignition pin to low and causes infinite loop if we detect a significant change in voltage signal from one of the three safety board connections
  if(p0 < 0.1 || p1 < 0.1 || p3 < 0.1) {
    digitalWrite(IGNITION_PIN, LOW);
    while(true);
  }
  delay(20);
}