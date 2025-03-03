// Credit for library and example code goes to Stephen Zhang / Lucas Etchezuri: https://github.com/xzhang03/Pico-ADS131M04?tab=readme-ov-file

#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>

ADS131M04 adc;
adcOutput res;

void setup()
{
  Serial.begin(115200);
  // clk_pin, miso_pin, mosi_pin, cs_pin, drdy_pin, reset_pin
  adc.begin(14, 28, 27, 25, 20, 24);
  
  // Channel setup
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setOsr(OSR_1024);      // 32KSPS  only with 8MHZ clock

  delay(100);
}

void loop()
{
  adcOutput res;
  delay(100);

  // -----------------------------------------------------------
  // MODIFY BELOW WHILE LOOP TO USE AND MANIPULATE ADC DATA
  // -----------------------------------------------------------
  while (1)
  {
    // res.ch0 contains data in channel 0, res.ch1 contains data in channel 1, and so on until res.ch3
    res = adc.readADC();

    // Example: printing out ADC channel values for channels 0, 1, 2, and 3
    Serial.print("Status = ");
    Serial.println(res.status, BIN);
    Serial.print("CH0 = ");
    Serial.println(res.ch0);
    Serial.print("CH1 = ");
    Serial.println(res.ch1);
    Serial.print("CH2 = ");
    Serial.println(res.ch2);
    Serial.print("CH3 = ");
    Serial.println(res.ch3);
    Serial.println("");
    delay(500);

    // -----------------------------------------------------------
    // DIAGNOSTIC PRINTS
    // -----------------------------------------------------------
    // Serial.println(adc.readRegister(REG_CLOCK), BIN);
    // delay(500);
    // -----------------------------------------------------------
  }
}