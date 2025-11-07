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

TimerHandle_t xTimers[2];


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

    
    xTimers[0] = xTimerCreate("hr_timer", pdMS_TO_TICKS(cSampleRateHeartRate), pdTRUE, NULL, heartRateTimerCallback);
    xTimers[1] = xTimerCreate("pulseox_timer", pdMS_TO_TICKS(cSampleRatePulseOx), pdTRUE, NULL, pulseOxTimerCallback);
    xTaskCreate(&task, "hr_task", 2048, NULL, 5, NULL);

    xTimerStart(xTimers[0], 0);
    xTimerStart(xTimers[1], 0);

}


void samplePulseOx()
{
    // reads pulse ox from MAX30102
    flagSamplePulseOx = true;
}

void sampleHeartRate()
{
    flagSampleHeartRate = true;
    //reads HR from MAX30102
}


void task(void *pvParameter)
{
    while(pdTRUE)
    {
        
        printf("task heart rate\n");
        if(flagSampleHeartRate)
        {
        // sample heart rate every 150ms? might need to calculate this    }
            printf("sample heart rate");
            flagSampleHeartRate = false;
        }

        if(flagSamplePulseOx)
        {
        // sample pulseox every 1sec?
            printf("sample pulse ox");
            flagSamplePulseOx = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // test this
        uint8_t readBuffer = 0;
        sensorObj.ReadDataSanityCheck(&readBuffer, 1, 5000);
        printf("part id should be 0x15: %d\n", readBuffer);

        //i2c_master_read_from_device(i2c_port_t::I2C_NUM_0, MAX30102_READ_ADDR, buffer, size_t read_size, TickType_t ticks_to_wait), uint8_t *data, size_t data_len, i2c_ack_type_t ack)
    }
}

}