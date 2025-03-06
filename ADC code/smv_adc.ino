// Credit for library and example code goes to Stephen Zhang / Lucas Etchezuri: https://github.com/xzhang03/Pico-ADS131M04?tab=readme-ov-file

#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

ADS131M04 adc;
adcOutput res;

const int CLOCK_PIN = 26;


// Note: previous version used analogWriteFreq = 8192000, analogWrite(CLOCK_PIN, 128) and it seemed to work
// However, according to the Earle Philhower library, it shouldn't. The below clock code is its replacement.

// ----------------------------------------------------------------------------------
// DO NOT MODIFY THE BELOW CLOCK CODE
// ----------------------------------------------------------------------------------

// PIO program to generate a clock signal
const uint16_t clock_program_instructions[] = {
    0x80a0, //  0: pull   block          
    0x6021, //  1: out    pins, 1        
    0xa042, //  2: mov    x, osr         
    0x0043, //  3: jmp    x--, 3         
    0x6021, //  4: out    pins, 1        
    0xa042, //  5: mov    x, osr         
    0x0046, //  6: jmp    x--, 6         
    0x0000, //  7: jmp    0              
};

// Create a proper PIO program struct
const struct pio_program clock_program = {
    .instructions = clock_program_instructions,
    .length = 8,
    .origin = -1  // Allocate automatically
};

PIO pio;
uint sm;
uint offset;

// Function to initialize PIO for clock generation
void init_clock_pio(uint pin, float freq_mhz) {
    // Initialize PIO
    pio = pio0;
    
    // Load the program into PIO memory
    offset = pio_add_program(pio, &clock_program);
    
    // Initialize and start the state machine
    sm = pio_claim_unused_sm(pio, true);
    
    // Configure the state machine
    pio_sm_config c = pio_get_default_sm_config();
    
    // Set the OUT pins
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_gpio_init(pio, pin);
    
    // Configure OUT to use the specified pin
    sm_config_set_out_pins(&c, pin, 1);
    
    // Set the clock divider to achieve desired frequency
    // Each clock cycle in the PIO program takes 2 instructions
    // So we need to run at 2 * frequency to get the right output
    float div = clock_get_hz(clk_sys) / (2 * 2 * freq_mhz * 1000000);
    sm_config_set_clkdiv(&c, div);
    
    // Configure OUT shift settings
    sm_config_set_out_shift(&c, true, true, 32);
    
    // Initialize the state machine with the config
    pio_sm_init(pio, sm, offset, &c);
    
    // Set the X and Y registers to control the duty cycle (50%)
    pio_sm_put_blocking(pio, sm, 1);  // Half period count - 1
    
    // Start the state machine
    pio_sm_set_enabled(pio, sm, true);
}
// ----------------------------------------------------------------------------------
// CLOCK CODE END
// ----------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
// SETUP CODE
// ----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Generate 8.192MHz clock using PIO
  init_clock_pio(CLOCK_PIN, 8.192);
  
  delay(100); // Give the ADC time to recognize the clock
  

  // ----------------------------------------------------------------------------------
  // SHOULD MODIFY PINS TO MATCH YOUR BOARD
  // ----------------------------------------------------------------------------------
  // clk_pin, miso_pin, mosi_pin, cs_pin, drdy_pin, reset_pin
  adc.begin(14, 28, 27, 29, 20, 24);
  
  // Channel setup
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setOsr(OSR_1024);      // 32KSPS only with 8MHz clock

  Serial.println("ADC initialized");
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
    res = adc.readADC();

    // ----------------------------------------------------------------------------------
    // MODIFY BELOW CODE TO MANIPULATE THE ADC'S READINGS AS YOU DESIRE
    // ----------------------------------------------------------------------------------

    // Example: printing out ADC channel values for channels 0, 1, 2, and 3
    Serial.print("Status = ");
    Serial.println(res.status, BIN);
    Serial.print("CH0 = ");
    // adc.convert automatically converts the output into floating point voltage values
    Serial.println(adc.convert(res.ch0));
    Serial.print("CH1 = ");
    Serial.println(adc.convert(res.ch1));
    Serial.print("CH2 = ");
    Serial.println(adc.convert(res.ch2));
    Serial.print("CH3 = ");
    Serial.println(adc.convert(res.ch3));
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