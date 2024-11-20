/**
 * @file BNO08xSH2HAL.hpp
 * @author Myles Parfeniuk
 */
#pragma once

// hill-crest labs includes (apache 2.0 license, compatible with MIT)
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// esp-idf includes
#include <esp_log.h>
#include <esp_timer.h>

// forward dec to prevent compile errors
class BNO08x;

class BNO08xSH2HAL
{
    public:
        static void set_hal_imu(BNO08x* hal_imu);

        static int spi_open(sh2_Hal_t* self);
        static void spi_close(sh2_Hal_t* self);
        static int spi_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us);
        static int spi_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len);
        static uint32_t get_time_us(sh2_Hal_t* self);
        static void hal_cb(void* cookie, sh2_AsyncEvent_t* pEvent);
        static void sensor_report_cb(void* cookie, sh2_SensorEvent_t* event);

    private:
        static BNO08x* imu;
        static void hardware_reset();
        static bool spi_wait_for_int();
        static uint16_t spi_read_sh2_packet_header(uint8_t* pBuffer);
        static int spi_read_sh2_packet_body(uint8_t* pBuffer, uint16_t packet_sz);
};