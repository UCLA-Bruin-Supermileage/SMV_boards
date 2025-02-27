#ifndef SMV_ADC_H
#define SMV_ADC_H

#include <SPI.h>

class ADS131M04 {
public:
    ADS131M04(int csPin);
    void begin();
    void reset();
    void configure();
    void readData(uint32_t *data);

private:
    int _csPin;
};

#endif
