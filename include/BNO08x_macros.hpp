/**
 * @file BNO08x_macros.hpp
 * @author Myles Parfeniuk
 */
#pragma once

// standard library includes
#include <inttypes.h>

// esp-idf includes
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

// packet parsing macros

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
#define PARSE_PACKET_LENGTH(header) (UINT16_CLR_LSB(static_cast<uint16_t>(header[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(header[0])))
