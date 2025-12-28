#include <stdint.h>
#include <stddef.h>


#ifndef HEALTHSENSOR_HPP
#define HEALTHSENSOR_HPP


class HealthSensor
{

    
private:
    void setup();
public:
    HealthSensor();
    void Configure();
    int ReadDataSanityCheck(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int ReadDataFifo(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int ReadData(uint8_t* readBuffer, size_t sizeRead, int timeout);
    int WriteData(const uint8_t reg_addr, const uint8_t writeBuffer, size_t sizeWrite, int timeout);
    int ClearFifoPtrs();


};

#endif //HEALTHSENSOR_HPP