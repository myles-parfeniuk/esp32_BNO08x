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
// in-house includes
#include "BNO08xPrivateTypes.hpp"

/**
 * @brief Clears the most significant byte of a 16-bit value.
 *
 * @param val_16bit The 16-bit value to modify.
 * @return The value with the MSB cleared.
 */
#define UINT16_CLR_MSB(val_16bit) ((val_16bit) & 0x00FFU)

/**
 * @brief Clears the least significant byte of a 16-bit value.
 *
 * @param val_16bit The 16-bit value to modify.
 * @return The value with the MSB cleared.
 */
#define UINT16_CLR_LSB(val_16bit) ((val_16bit) & 0xFF00U)

/**
 * @brief Clears a specified byte in a 32-bit value.
 *
 * @param val_32bit The 32-bit value to modify.
 * @param byte2clear The byte index to clear (0 = LSB, 3 = MSB).
 * @return The value with the specified byte cleared.
 */
#define UINT32_CLR_BYTE(val_32bit, byte2clear) ((val_32bit) & ~(0xFFUL << (byte2clear * 8UL)))

/**
 * @brief Masks a specified byte in a 32-bit value.
 *
 * @param val_32bit The 32-bit value to modify.
 * @param byte2mask The byte index to mask (0 = LSB, 3 = MSB).
 * @return The value with the specified byte masked.
 */
#define UINT32_MSK_BYTE(val_32bit, byte2mask) ((val_32bit) & (0xFFUL << (byte2mask * 8UL)))

// parsing universal to any packet

/**
 * @brief Parse length from SHTP packet header.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Length of SHTP packet.
 */
#define PARSE_PACKET_LENGTH(header)                                                                                              \
    (UINT16_CLR_LSB(static_cast<uint16_t>(header[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(header[0])))

// forward dec to prevent compile errors
class BNO08x;

/**
 * @class BNO08xSH2HAL
 *
 * @brief Fully static class containing callback implementations for sh2 HAL lib.
 * */
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
        static void sensor_event_cb(void* cookie, sh2_SensorEvent_t* event);

    private:
        static BNO08x* imu;
        static void hardware_reset();
        static bool spi_wait_for_int();
        static uint16_t spi_read_sh2_packet_header(uint8_t* pBuffer);
        static int spi_read_sh2_packet_body(uint8_t* pBuffer, uint16_t packet_sz);

        static const constexpr char* TAG = "BNO08xSH2HAL";
};