#include <stdint.h>
#include <stddef.h>


#ifndef HEALTHSENSOR_HPP
#define HEALTHSENSOR_HPP


class HealthSensor
{

    
private:
    void setup();
    void bitMask(uint8_t reg, uint8_t mask, uint8_t val);

public:
    HealthSensor();
    void Configure(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, uint16_t sampleRate, uint16_t pulseWidth, uint16_t adcRange);
    int ReadDataSanityCheck(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int ReadDataFifo(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int ReadData(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int WriteData(const uint8_t reg_addr, const uint8_t writeBuffer, size_t sizeWrite, int timeout);
    int ClearFifoPtrs();
    

};

#endif //HEALTHSENSOR_HPP