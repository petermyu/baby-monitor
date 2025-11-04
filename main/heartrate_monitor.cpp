#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "heartrate_monitor.hpp"
#include "healthsensordriver.hpp"



constexpr uint16_t cSampleRatePulseOx = 1000; // Unit: ms
constexpr uint8_t cSampleRateHeartRate = 100; // Unit: ms

static volatile bool flagSampleHeartRate = false;
static volatile bool flagSamplePulseOx = false;

static HealthSensor sensorObj; 
void task(void *pvParameter);
void samplePulseOx();
void sampleHeartRate();

static void pulseOxTimerCallback( TimerHandle_t xTimer )
{
    flagSamplePulseOx = true;
}

static void heartRateTimerCallback( TimerHandle_t xTimer )
{
    flagSampleHeartRate = true;
}

namespace HeartRateMonitor 
{

void Init()
{
    printf("HeartRateMonitor::Init");
    sensorObj = HealthSensor();

    if(!xTimerCreate("hr_timer", pdMS_TO_TICKS(cSampleRateHeartRate), pdTRUE, NULL, heartRateTimerCallback))
    {
        //error message
    }
    if(!xTimerCreate("pulseox_timer", pdMS_TO_TICKS(cSampleRatePulseOx), pdTRUE, NULL, pulseOxTimerCallback))
    {
        //error message
    }
    xTaskCreate(&task, "hr_task", 2048, NULL, 5, NULL);
}


void samplePulseOx()
{
    // reads pulse ox from MAX30102
}

void sampleHeartRate()
{
    //reads HR from MAX30102
}


void task(void *pvParameter)
{
    while(pdTRUE)
    {
        
        printf("task heart rate");
        if(flagSampleHeartRate)
        {
        // sample heart rate every 150ms? might need to calculate this    }
            printf("sample heart rate");
        }

        if(flagSamplePulseOx)
        {
        // sample pulseox every 1sec?
            printf("sample pulse ox");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // test this
        uint8_t readBuffer[3];
        sensorObj.ReadData(readBuffer, 3, 5000);
        printf("readbuffer 0: %d", readBuffer[0]);
        printf("readbuffer 1: %d", readBuffer[1]);
        printf("readbuffer 2: %d", readBuffer[2]);

        //i2c_master_read_from_device(i2c_port_t::I2C_NUM_0, MAX30102_READ_ADDR, buffer, size_t read_size, TickType_t ticks_to_wait), uint8_t *data, size_t data_len, i2c_ack_type_t ack)
    }
}

}