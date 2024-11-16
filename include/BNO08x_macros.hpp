#pragma once

// standard library includes
#include <inttypes.h>

// esp-idf includes
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define CHECK_TASKS_RUNNING(evt_grp_task_flow, running_bit) ((xEventGroupGetBits(evt_grp_task_flow) & (running_bit)) != 0)

// packet parsing macros
#define UINT16_CLR_MSB(val_16bit) ((val_16bit) & 0x00FFU)
#define UINT16_CLR_LSB(val_16bit) ((val_16bit) & 0xFF00U)
#define UINT32_CLR_BYTE(val_32bit, byte2clear) ((val_32bit) & ~(0xFFUL << (byte2clear * 8UL)))
#define UINT32_MSK_BYTE(val_32bit, byte2mask) ((val_32bit) & (0xFFUL << (byte2mask * 8UL)))

// parsing universal to any packet
#define PARSE_PACKET_LENGTH(packet)                                                                                                                  \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->header[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->header[0])))

#define PARSE_PACKET_DATA_LENGTH(packet_ptr)                                                                                                         \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->header[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->header[0])))

#define PARSE_PACKET_TIMESTAMP(packet_ptr)                                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[4]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[3]) << 16UL, 2UL) |    \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[2]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[1]), 0UL))

// product id report parsing
#define PARSE_PRODUCT_ID_REPORT_RESET_REASON(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[1]), 0UL)

#define PARSE_PRODUCT_ID_REPORT_SW_PART_NO(packet_ptr)                                                                                               \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[7]) << 24UL, 3UL) |                                                                       \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[6]) << 16UL, 2UL) |                                                               \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[5]) << 8UL, 1UL) |                                                                \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_ptr->body[4]), 0UL))

#define PARSE_PRODUCT_ID_REPORT_SW_BUILD_NO(packet_ptr)                                                                                              \
    UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[11]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[10]) << 16UL, 2UL) |  \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[9]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[8]), 0UL)

#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_PATCH(packet_ptr)                                                                                         \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[13]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[12]), 0UL))

#define PARSE_PRODUCT_ID_REPORT_PRODUCT_ID(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[0]), 0UL)

#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_MAJOR(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[2]), 0UL)

#define PARSE_PRODUCT_ID_REPORT_SW_VERSION_MINOR(packet_ptr) UINT32_MSK_BYTE(static_cast<uint32_t>(packet->body[3]), 0UL)

// input report parsing
#define PARSE_INPUT_REPORT_RAW_QUAT_I(packet)                                                                                                        \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[1]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[0])))
#define PARSE_INPUT_REPORT_RAW_QUAT_J(packet)                                                                                                        \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[3]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[2])))
#define PARSE_INPUT_REPORT_RAW_QUAT_K(packet)                                                                                                        \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[4])))
#define PARSE_INPUT_REPORT_RAW_QUAT_REAL(packet)                                                                                                     \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[7]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[6])))

#define PARSE_INPUT_REPORT_RAW_GYRO_VEL_X(packet)                                                                                                    \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[9]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[8])))
#define PARSE_INPUT_REPORT_RAW_GYRO_VEL_Y(packet)                                                                                                    \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[11]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[10])))
#define PARSE_INPUT_REPORT_RAW_GYRO_VEL_Z(packet)                                                                                                    \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[12])))

#define PARSE_INPUT_REPORT_STATUS_BITS(packet) (packet->body[5 + 2] & 0x03U)

#define PARSE_INPUT_REPORT_DATA_1(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 5]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 4])))
#define PARSE_INPUT_REPORT_DATA_2(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 7]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 6])))
#define PARSE_INPUT_REPORT_DATA_3(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 9]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 8])))
#define PARSE_INPUT_REPORT_DATA_4(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 11]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 10])))
#define PARSE_INPUT_REPORT_DATA_5(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 12])))
#define PARSE_INPUT_REPORT_DATA_6(packet)                                                                                                            \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet->body[5 + 15]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet->body[5 + 14])))

#define PARSE_FRS_READ_RESPONSE_REPORT_RECORD_ID(packet_body)                                                                                        \
    (UINT16_CLR_LSB(static_cast<uint16_t>(packet_body[13]) << 8U) | UINT16_CLR_MSB(static_cast<uint16_t>(packet_body[12])))
#define PARSE_FRS_READ_RESPONSE_REPORT_DATA_1(packet_body)                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[7]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[6]) << 16UL, 2UL) |     \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[5]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[4]), 0UL))
#define PARSE_FRS_READ_RESPONSE_REPORT_DATA_2(packet_body)                                                                                           \
    (UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[11]) << 24UL, 3UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[10]) << 16UL, 2UL) |   \
            UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[9]) << 8UL, 1UL) | UINT32_MSK_BYTE(static_cast<uint32_t>(packet_body[8]), 0UL))