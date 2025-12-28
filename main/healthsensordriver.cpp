#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "heartrate_monitor.hpp"
#include "driver/i2c_types.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <stdint.h>
#include "healthsensordriver.hpp"

#define MAX30102_DEVICE_ADDR 0x57
#define MAX30102_READ_ADDR 0xAF
#define MAX30102_WRITE_ADDR 0xAE
#define I2C_MASTER_FREQ_HZ 100000
#define PORT_NUMBER -1  //auto select the port

// registers
constexpr uint8_t  MAX30102_REG_FIFO = 0x07;
constexpr uint8_t  MAX30102_REG_FIFO_WR_PTR =0x04;
constexpr uint8_t  MAX30102_REG_FIFO_RD_PTR =0x06;
constexpr uint8_t  MAX30102_REG_PART_ID =0XFF;

constexpr uint8_t MAX30102_LED1_PULSEAMP = 	0x0C;
constexpr uint8_t MAX30102_LED2_PULSEAMP = 	0x0D;
constexpr uint8_t MAX30102_LED3_PULSEAMP = 	0x0E;
constexpr uint8_t MAX30102_LED_PROX_AMP = 	0x10;


constexpr gpio_num_t pinVin = gpio_num_t::GPIO_NUM_23;
constexpr gpio_num_t pinScl = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t pinSda = gpio_num_t::GPIO_NUM_21;
constexpr gpio_num_t pinVred = gpio_num_t::GPIO_NUM_19;
//constexpr gpio_num_t pinVird = gpio_num_t::GPIO_NUM_21;

i2c_master_bus_handle_t bus_handle = 0;
i2c_master_dev_handle_t dev_handle = 0;

HealthSensor::HealthSensor()
{
    setup();
}

void HealthSensor::setup()
{
    gpio_reset_pin(pinVin);
    gpio_set_direction(pinVin, GPIO_MODE_OUTPUT);
    gpio_set_level(pinVin, 1);
    
    const i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = PORT_NUMBER,
        .sda_io_num = pinSda,
        .scl_io_num = pinScl,
        .clk_source = i2c_clock_source_t::I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true, .allow_pd = false},
    };
    
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MAX30102_DEVICE_ADDR ,
    .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    .scl_wait_us = 0,
    .flags = {.disable_ack_check = 1},
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));




    // setup sampling rate
}

void HealthSensor::Configure()
{
    WriteData(MAX30102_LED1_PULSEAMP, 0xFF, 1, 1000);
    WriteData(MAX30102_LED2_PULSEAMP, 0xFF, 1, 1000);
    WriteData(MAX30102_LED3_PULSEAMP, 0xFF, 1, 1000);
}
int HealthSensor::ReadDataSanityCheck(uint8_t* readBuffer, size_t sizeRead, int timeout)
{
    const uint8_t writeBuf = MAX30102_REG_PART_ID;
    i2c_master_transmit(dev_handle, &writeBuf, 1, timeout);
    i2c_master_receive(dev_handle, readBuffer, sizeRead, timeout);
    return 0;
}

int HealthSensor::ReadDataFifo(uint8_t* readBuffer, size_t sizeRead, int timeout)
{
    const uint8_t writeBuf = MAX30102_REG_FIFO;
    i2c_master_transmit(dev_handle, &writeBuf, 1, timeout);
    i2c_master_receive(dev_handle, readBuffer, sizeRead, timeout);
    return 0;
}
int HealthSensor::WriteData(uint8_t reg_addr, uint8_t writeBuffer, size_t sizeWrite, int timeout)
{ 
        i2c_master_transmit_multi_buffer_info_t data[2] = {
        {.write_buffer = &reg_addr, .buffer_size = 1},
        {.write_buffer = &writeBuffer, .buffer_size = 1},
    };
    i2c_master_multi_buffer_transmit(dev_handle, data,  sizeof(data) / sizeof(i2c_master_transmit_multi_buffer_info_t), static_cast<int>(timeout));
    return 0;
}

int HealthSensor::ClearFifoPtrs()
{ 
    uint8_t fifo_wr_buf = MAX30102_REG_FIFO_WR_PTR;
    uint8_t fifo_rd_buf = MAX30102_REG_FIFO_RD_PTR;
    uint8_t fifo_ovf_buf = 0x05;
    uint8_t fifo_clr = 0;
    i2c_master_transmit_multi_buffer_info_t fifo_wr_clr[2] = {
        {.write_buffer = &fifo_wr_buf, .buffer_size = 1},
        {.write_buffer = &fifo_clr, .buffer_size = 1},
    };
    i2c_master_transmit_multi_buffer_info_t fifo_rd_clr[2] = {
        {.write_buffer = &fifo_rd_buf, .buffer_size = 1},
        {.write_buffer = &fifo_clr, .buffer_size = 1},
    };
    i2c_master_transmit_multi_buffer_info_t fifo_ovf_clr[2] = {
        {.write_buffer = &fifo_ovf_buf, .buffer_size = 1},
        {.write_buffer = &fifo_clr, .buffer_size = 1},
    };

    i2c_master_multi_buffer_transmit(dev_handle, fifo_wr_clr, sizeof(fifo_wr_clr) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1);
    i2c_master_multi_buffer_transmit(dev_handle, fifo_rd_clr, sizeof(fifo_rd_clr) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1);
    i2c_master_multi_buffer_transmit(dev_handle, fifo_ovf_clr, sizeof(fifo_ovf_clr) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1);
    return 0;
}

