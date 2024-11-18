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

/**
 * @brief Clears the most significant byte of a 16-bit value.
 *
 * @param evt_grp_task_flow Task flow event group handle.
 * @param running_bit EVT_GRP_TSK_FLW_RUNNING_BIT
 *
 * @return The value with the MSB cleared.
 */
#define CHECK_TASKS_RUNNING(evt_grp_task_flow, running_bit) ((xEventGroupGetBits(evt_grp_task_flow) & (running_bit)) != 0)

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
#define PARSE_PACKET_LENGTH(packet_ptr)                                                                                                              \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet_ptr->header[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet_ptr->header[0])))

/**
 * @brief Parse timestamp from SHTP packet.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Packet timestamp.
 */
#define PARSE_PACKET_TIMESTAMP(packet_ptr)                                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[4]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[3]) << 16UL, 2UL) |   \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[2]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[1]), 0UL))

// product id report parsing

/**
 * @brief Parse reset reason from SHTP packet containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Reset reason.
 */
#define PARSE_PRODUCT_ID_REPORT_RESET_REASON(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[1]), 0UL)

/**
 * @brief Parse sw part number from SHTP packet containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sw part number.
 */
#define PARSE_PRODUCT_ID_REPORT_SW_PART_NO(packet_ptr)                                                                                               \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[7]) << 24UL, 3UL) |                                                                      \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[6]) << 16UL, 2UL) |                                                               \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[5]) << 8UL, 1UL) |                                                                \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[4]), 0UL))

/**
 * @brief Parse sw build number from SHTP packet containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sw build number.
 */
#define PARSE_PRODUCT_ID_REPORT_SW_BUILD_NO(packet_ptr)                                                                                              \
    UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[11]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[10]) << 16UL, 2UL) |  \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[9]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[8]), 0UL)

/**
 * @brief Parse sw version patch from SHTP packet containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sw version patch.
 */
#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_PATCH(packet_ptr)                                                                                         \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[13]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[12]), 0UL))

/**
 * @brief Parse product ID SHTP packet containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Product ID.
 */
#define PARSE_PRODUCT_ID_REPORT_PRODUCT_ID(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[0]), 0UL)

/**
 * @brief Parse product sw version major containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sw version major.
 */
#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_MAJOR(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[2]), 0UL)

/**
 * @brief Parse product sw version minor containing product ID report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sw version minor.
 */
#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_MINOR(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[3]), 0UL)

// gyro report parsing

/**
 * @brief Parse quat I data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Quat I data.
 */
#define PARSE_GYRO_REPORT_RAW_QUAT_I(packet)                                                                                                         \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[0])))

/**
 * @brief Parse quat J data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Quat J data.
 */
#define PARSE_GYRO_REPORT_RAW_QUAT_J(packet)                                                                                                         \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[3]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[2])))

/**
 * @brief Parse quat K data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Quat K data.
 */
#define PARSE_GYRO_REPORT_RAW_QUAT_K(packet)                                                                                                         \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[4])))

/**
 * @brief Parse quat real data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Quat real data.
 */
#define PARSE_GYRO_REPORT_RAW_QUAT_REAL(packet)                                                                                                      \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[7]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[6])))

/**
 * @brief Parse x axis velocity data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return x velocity data.
 */
#define PARSE_GYRO_REPORT_RAW_GYRO_VEL_X(packet)                                                                                                     \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[9]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[8])))

/**
 * @brief Parse y axis velocity data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return y velocity data.
 */
#define PARSE_GYRO_REPORT_RAW_GYRO_VEL_Y(packet)                                                                                                     \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[11]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[10])))

/**
 * @brief Parse z axis velocity data from integrated gyro rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return z velocity data.
 */
#define PARSE_GYRO_REPORT_RAW_GYRO_VEL_Z(packet)                                                                                                     \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[12])))

// input report parsing

/**
 * @brief Parse status bits from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Input report status bits.
 */
#define PARSE_INPUT_REPORT_STATUS_BITS(packet) (packet->body[5 + 2] & 0x03U)

/**
 * @brief Parse report ID from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Report ID.
 */
#define PARSE_INPUT_REPORT_REPORT_ID(packet) UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5]))

/**
 * @brief Parse first data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return First data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_1(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 5]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 4])))

/**
 * @brief Parse second data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return Second data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_2(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 7]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 6])))

/**
 * @brief Parse third data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return third data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_3(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 9]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 8])))

/**
 * @brief Parse fourth data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return fourth data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_4(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 11]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 10])))

/**
 * @brief Parse fifth data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return fifth data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_5(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 12])))

/**
 * @brief Parse sixth data block from input report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return sixth data block of input report.
 */
#define PARSE_INPUT_REPORT_DATA_6(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 15]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 14])))

/**
 * @brief Checks if packet containing input report is a rotation vector report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return True if contained input report is rotation vector report.
 */
#define IS_ROTATION_VECTOR_REPORT(packet)                                                                                                            \
    ((packet)->body[5] == SENSOR_REPORT_ID_ROTATION_VECTOR || (packet)->body[5] == SENSOR_REPORT_ID_GAME_ROTATION_VECTOR ||                          \
            (packet)->body[5] == SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR ||                                                                 \
            (packet)->body[5] == SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR)

// frs read response report parsing

/**
 * @brief Parse FRS record ID from FRS read response report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return FRS record ID.
 */
#define PARSE_FRS_READ_RESPONSE_REPORT_RECORD_ID(packet_body)                                                                                        \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet_body[13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet_body[12])))

/**
 * @brief Parse data block 1 from FRS read response report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return FRS read response data block 1.
 */
#define PARSE_FRS_READ_RESPONSE_REPORT_DATA_1(packet_body)                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[7]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[6]) << 16UL, 2UL) |     \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[5]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[4]), 0UL))

/**
 * @brief Parse data block 2 from FRS read response report.
 *
 * @param packet Pointer to bno08x_rx_packet_t containing data.
 * @return FRS read response data block 2.
 */
#define PARSE_FRS_READ_RESPONSE_REPORT_DATA_2(packet_body)                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[11]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[10]) << 16UL, 2UL) |   \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[9]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[8]), 0UL))
