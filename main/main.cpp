#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "heartrate_monitor.hpp"

 extern "C" {
        void app_main();
    }


constexpr gpio_num_t blinkGPIO = gpio_num_t::GPIO_NUM_2;

void hello_task(void *pvParameter)
{
	while(1)
	{
	    printf("Hello world!\n");
	    vTaskDelay(100);
	}
}
void blinky(void *pvParameter)
{
    /* Set the GPIO as a push/pull output */
    gpio_reset_pin(blinkGPIO);
    gpio_set_direction(blinkGPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(blinkGPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(blinkGPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void app_main()
{
    xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);
    xTaskCreate(&blinky, "blinky", 2048,NULL,5,NULL );


    // Init modules
    HeartRateMonitor::Init();
    
}