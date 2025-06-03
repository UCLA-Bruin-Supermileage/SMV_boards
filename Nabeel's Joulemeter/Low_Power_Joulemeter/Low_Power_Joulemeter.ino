#include "SMVcanbus.h"
#include <string.h>

#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>
#include "RP2040_PWM.h"

ADS131M04 adc;
adcOutput res;
RP2040_PWM* PWM_Instance;
CANBUS can(Joule_L);


const int CLOCK_PIN = 26;

const double VOLTAGE_DIVIDING_FACTOR = 0.08928571429;
const int INA_GAIN = 20;
const double SCUFFED_VOLTAGE_DIVIDING_FACTOR = 0.1669449;
const double SHUNT_RESISTANCE = 0.01;

const double ERROR_MULTIPLIER = 1.33;

// ----------------------------------------------------------------------------------
// SETUP CODE
// ----------------------------------------------------------------------------------
void setup()
{
  // Serial.begin(115200);
  // can.begin() MUST be in this location -----------------------------------------------------------------
  can.begin();
  // can.begin() MUST be in this location -----------------------------------------------------------------
  // while (!Serial) delay(10);
  
  // Note: previous version used analogWriteFreq(8192000), analogWrite(CLOCK_PIN, 128) and it seemed to work
  // However, according to the Earle Philhower library, it shouldn't. The following code is its replacement.
  PWM_Instance = new RP2040_PWM(CLOCK_PIN, 8192000, 50);
  PWM_Instance->setPWM(CLOCK_PIN, 8192000, 50);
  
  
  delay(100); // Give the ADC time to recognize the clock
  

  // -----------------------------------------------------------------------------------------------------------
  // SHOULD MODIFY PINS TO MATCH YOUR BOARD - if you do not have DRDY, ignore it. DO NOT IGNORE RESET_PIN
  // -----------------------------------------------------------------------------------------------------------
  // ORDER: clk_pin, miso_pin, mosi_pin, cs_pin, drdy_pin, reset_pin
  adc.begin(14, 28, 27, 29, 20, 24);

  
  // Channel setup
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setOsr(OSR_1024);      // 32KSPS only with 8MHz clock

  // Serial.println("ADC initialized");
  delay(100);
}

// ----------------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------------

void loop()
{
  adcOutput res;
  delay(100);

  // -----------------------------------------------------------
  // ADC DATA ACQUISITION AND MANIPULATION
  // -----------------------------------------------------------
  while (1)
  {
    // res.ch0 contains data in channel 0, res.ch1 contains data in channel 1, and so on until res.ch3
    SPI.beginTransaction(SPISettings(SPIfreq, MSBFIRST, SPI_MODE1));
    res = adc.readADC();

    // ----------------------------------------------------------------------------------
    // MODIFY BELOW CODE TO MANIPULATE THE ADC'S READINGS AS YOU DESIRE
    // ----------------------------------------------------------------------------------

    double voltageShunt = (adc.convert(res.ch0) / SCUFFED_VOLTAGE_DIVIDING_FACTOR) / INA_GAIN;
    double voltageTwelve = adc.convert(res.ch1) / VOLTAGE_DIVIDING_FACTOR;
    double wattage = (voltageShunt/SHUNT_RESISTANCE)*voltageTwelve;

    // // Below would print wattage:
    // Serial.print("The instantaneous power consumption of the load is: ");
    // Serial.print(wattage);
    // Serial.println(" W");

    // Serial.println(adc.convert(res.ch3)); // printing test voltage

    // Below sends wattage through can bus
    double data = wattage * ERROR_MULTIPLIER;
    can.send(data, Power);


    // // Example: printing out ADC channel values for channels 0, 1, 2, and 3
    // Serial.print("Status = ");
    // Serial.println(res.status, BIN);
    // Serial.print("CH0 = ");
    // // adc.convert automatically converts the output into floating point voltage values
    // Serial.println(adc.convert(res.ch0));
    // Serial.print("CH1 = ");
    // Serial.println(adc.convert(res.ch1));
    // Serial.print("CH2 = ");
    // Serial.println(adc.convert(res.ch2));
    // Serial.print("CH3 = ");
    // Serial.println(adc.convert(res.ch3));
    // Serial.println("");
    // delay(500);
    // SPI.endTransaction();

    // double data = adc.convert(res.ch3);
    // delay(100);
    // can.send(data, Current);
    // delay(100);

    // -----------------------------------------------------------
    // DIAGNOSTIC PRINTS
    // -----------------------------------------------------------
    // Serial.println(adc.readRegister(REG_ID), BIN);
    delay(500);
    // -----------------------------------------------------------
  }
}