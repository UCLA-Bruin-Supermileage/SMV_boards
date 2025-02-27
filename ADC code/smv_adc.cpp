#include "smv_adc.h"

ADS131M04::ADS131M04(int csPin) : _csPin(csPin) {}

void ADS131M04::begin() {
    SPI.begin();
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    reset();
    configure();
}

// -----------------------------------------------------------------------------------------------
// ADC RESET FUNCTION
// -----------------------------------------------------------------------------------------------
void ADS131M04::reset() {
  digitalWrite(_csPin, LOW);  // Select ADC

  SPI.transfer(0x06);         // 0x06 is the reset command

  digitalWrite(_csPin, HIGH); // Deselect ADC
  delay(1); // For timing purposes
}
// -----------------------------------------------------------------------------------------------
// END BLOCK
// -----------------------------------------------------------------------------------------------


// -----------------------------------------------------------------------------------------------
// ADC CONFIGURATION FUNCTION
// -----------------------------------------------------------------------------------------------
void ADS131M04::configure() {
    digitalWrite(_csPin, LOW);
    SPI.transfer(0x42); // WREG command for CONFIG1
    SPI.transfer(0x00); // Register address
    SPI.transfer(0x00);
    SPI.transfer(0x03);
    digitalWrite(_csPin, HIGH);
}
// -----------------------------------------------------------------------------------------------
// END BLOCK
// -----------------------------------------------------------------------------------------------


// -----------------------------------------------------------------------------------------------
// READ DATA ONE TIME (24 BITS)
// -----------------------------------------------------------------------------------------------
void ADS131M04::readData(uint32_t *data) {
    digitalWrite(_csPin, LOW);
    for (int i = 0; i < 4; i++) {
        uint8_t byte1 = SPI.transfer(0x00);
        uint8_t byte2 = SPI.transfer(0x00);
        uint8_t byte3 = SPI.transfer(0x00);
        data[i] = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;
    }
    digitalWrite(_csPin, HIGH);
}
// -----------------------------------------------------------------------------------------------
// END BLOCK
// -----------------------------------------------------------------------------------------------
