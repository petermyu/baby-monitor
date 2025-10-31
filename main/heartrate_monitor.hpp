#include <cstdint>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "portmacro.h"

namespace HeartRateMonitor 
{
    void Init();
    void task(void *pvParameter);
    void samplePulseOx();
    void sampleHeartRate();
}