#include <cstdint>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "heartrate_monitor.hpp"


constexpr uint8_t cSampleRatePulseOx = 1000; // Unit: ms
constexpr uint8_t cSampleRateHeartRate = 100; // Unit: ms
static bool flagSampleHeartRate = false;
static bool flagSamplePulseOx = false;

void task(void *pvParameter);
void samplePulseOx();
void sampleHeartRate();
void heartRateTimerCallback( TimerHandle_t xTimer );
void pulseOxTimerCallback( TimerHandle_t xTimer );

namespace HeartRateMonitor 
{

void Init()
{
    if(!xTimerCreate("hr_timer", pdMS_TO_TICKS(cSampleRateHeartRate), pdTRUE, (void*) 0, heartRateTimerCallback))
    {
        //error message
    }
    if(!xTimerCreate("pulseox_timer", pdMS_TO_TICKS(flagSamplePulseOx), pdTRUE, (void*) 0, heartRateTimerCallback))
    {
        //error message
    }
    xTaskCreate(&task, "hr_task", 2048, NULL, 5, NULL);
}

void pulseOxTimerCallback( TimerHandle_t xTimer )
{
    flagSamplePulseOx = true;
}

void heartRateTimerCallback( TimerHandle_t xTimer )
{
    flagSampleHeartRate = true;
}

void samplePulseOx()
{
    // reads pulse ox from MAX30108
}

void sampleHeartRate()
{
    //reads HR from MAX30108 
}


void task(void *pvParameter)
{
    while(pdTRUE)
    {
        if(flagSampleHeartRate)
        {
        // sample heart rate every 150ms? might need to calculate this    }
        }

        if(flagSamplePulseOx)
        {
        // sample pulseox every 1sec?
        }
    }
}

}