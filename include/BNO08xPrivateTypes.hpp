/**
 * @file BNO08xPrivateTypes.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// etl includes
#include <etl/vector.h>
#include <etl/variant.h>
// esp-idf includes
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
// in-house includes
#include "BNO08xGlobalTypes.hpp"
#include "BNO08xCbParamRptID.hpp"
#include "BNO08xCbParamVoid.hpp"

namespace BNO08xPrivateTypes
{
    using bno08x_cb_list_t = etl::vector<etl::variant<BNO08xCbParamVoid, BNO08xCbParamRptID>,
            CONFIG_ESP32_BNO08X_CB_MAX>; ///< Alias for vector type to contain both cb flavors.

    /// @brief Holds info about which functionality has been successfully initialized (used by deconstructor during cleanup).
    typedef struct bno08x_init_status_t
    {
            bool gpio_outputs;         ///< True if GPIO outputs have been initialized.
            bool gpio_inputs;          ///< True if GPIO inputs have been initialized.
            bool isr_service;          ///< True if global ISR service has been initialized.
            bool isr_handler;          ///< True if HINT ISR handler has been initialized.
            bool spi_bus;              ///< True if spi_bus_initialize() has been called successfully.
            bool spi_device;           ///< True if spi_bus_add_device() has been called successfully.
            bool sh2_HAL;              ///< True if sh2_open() has been called successfully.
            bool data_proc_task;       ///< True if xTaskCreate has been called successfully for data_proc_task.
            bool sh2_HAL_service_task; ///< True if xTaskCreate has been called successfully for sh2_HAL_service_task.
            bool cb_task;              ///< True if xTaskCreate has been called successfully for cb_task.

            bno08x_init_status_t()
                : gpio_outputs(false)
                , gpio_inputs(false)
                , isr_service(false)
                , isr_handler(false)
                , spi_bus(false)
                , spi_device(false)
                , data_proc_task(false)
                , sh2_HAL_service_task(false)
                , cb_task(false)
            {
            }
    } bno08x_init_status_t;

    /// @brief Holds info used to initialize report modules.
    typedef struct bno08x_report_info_t
    {
            uint8_t ID;
            EventBits_t rpt_bit;
            SemaphoreHandle_t* _sh2_HAL_lock;
            SemaphoreHandle_t* _data_lock;
            EventGroupHandle_t* _evt_grp_rpt_en;
            EventGroupHandle_t* _evt_grp_rpt_data_available;
            EventGroupHandle_t* _evt_grp_bno08x_task;
            etl::vector<uint8_t, TOTAL_RPT_COUNT>* _en_report_ids;
            bno08x_cb_list_t* _cb_list;

            bno08x_report_info_t(uint8_t ID, EventBits_t rpt_bit, SemaphoreHandle_t* _sh2_HAL_lock, SemaphoreHandle_t* _data_lock,
                    EventGroupHandle_t* _evt_grp_rpt_en, EventGroupHandle_t* _evt_grp_rpt_data_available, EventGroupHandle_t* _evt_grp_bno08x_task,
                    etl::vector<uint8_t, TOTAL_RPT_COUNT>* _en_report_ids, bno08x_cb_list_t* _cb_list)
                : ID(ID)
                , rpt_bit(rpt_bit)
                , _sh2_HAL_lock(_sh2_HAL_lock)
                , _data_lock(_data_lock)
                , _evt_grp_rpt_en(_evt_grp_rpt_en)
                , _evt_grp_rpt_data_available(_evt_grp_rpt_data_available)
                , _evt_grp_bno08x_task(_evt_grp_bno08x_task)
                , _en_report_ids(_en_report_ids)
                , _cb_list(_cb_list)
            {
            }
    } bno08x_report_info_t;

    inline static sh2_SensorConfig default_sensor_cfg = {.changeSensitivityEnabled = false, ///<TODO
            .changeSensitivityRelative = false,
            .wakeupEnabled = false,
            .alwaysOnEnabled = false,
            .changeSensitivity = 0,
            .reportInterval_us = 0,
            .batchInterval_us = 0,
            .sensorSpecific = 0};

    /// @brief Bits for evt_grp_rpt_en & evt_grp_rpt_data_available
    enum bno08x_rpt_bit_t : EventBits_t
    {
        EVT_GRP_RPT_RV_BIT = (1UL << 0U),                    ///< When set, rotation vector reports are active.
        EVT_GRP_RPT_RV_GAME_BIT = (1UL << 1U),               ///< When set, game rotation vector reports are active.
        EVT_GRP_RPT_RV_ARVR_S_BIT = (1UL << 2U),             ///< When set, ARVR stabilized rotation vector reports are active.
        EVT_GRP_RPT_RV_ARVR_S_GAME_BIT = (1UL << 3U),        ///< When set, ARVR stabilized game rotation vector reports are active.
        EVT_GRP_RPT_GYRO_INTEGRATED_RV_BIT = (1UL << 4U),    ///< When set, gyro integrator rotation vector reports are active.
        EVT_GRP_RPT_GEOMAG_RV_BIT = (1UL << 5U),             ///< When set, geomagnetic rotation vector reports are active.
        EVT_GRP_RPT_ACCELEROMETER_BIT = (1UL << 6U),         ///< When set, accelerometer reports are active.
        EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT = (1UL << 7U),  ///< When set, linear accelerometer reports are active.
        EVT_GRP_RPT_GRAVITY_BIT = (1UL << 8U),               ///< When set, gravity reports are active.
        EVT_GRP_RPT_CAL_GYRO_BIT = (1UL << 9U),              ///< When set, calibrated gyro reports are active.
        EVT_GRP_RPT_UNCAL_GYRO_BIT = (1UL << 10U),           ///< When set, uncalibrated gyro reports are active.
        EVT_GRP_RPT_CAL_MAGNETOMETER_BIT = (1UL << 11U),     ///< When set, calibrated magnetometer reports are active.
        EVT_GRP_RPT_UNCAL_MAGNETOMETER_BIT = (1UL << 12U),   ///< When set, uncalibrated magnetometer reports are active.
        EVT_GRP_RPT_TAP_DETECTOR_BIT = (1UL << 13U),         ///< When set, tap detector reports are active.
        EVT_GRP_RPT_STEP_COUNTER_BIT = (1UL << 14U),         ///< When set, step counter reports are active.
        EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT = (1UL << 15U), ///< When set, stability classifier reports are active.
        EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT = (1UL << 16U),  ///< When set, activity classifier reports are active.
        EVT_GRP_RPT_SHAKE_DETECTOR_BIT = (1UL << 17U),       ///< When set, shake detector reports are active.
        EVT_GRP_RPT_RAW_ACCELEROMETER_BIT = (1UL << 18U),    ///< When set, raw accelerometer reports are active.
        EVT_GRP_RPT_RAW_GYRO_BIT = (1UL << 19U),             ///< When set, raw gyro reports are active.
        EVT_GRP_RPT_RAW_MAGNETOMETER_BIT = (1UL << 20U),     ///< When set, raw magnetometer reports are active.

        EVT_GRP_RPT_ALL = EVT_GRP_RPT_RV_BIT | EVT_GRP_RPT_RV_GAME_BIT | EVT_GRP_RPT_RV_ARVR_S_BIT | EVT_GRP_RPT_RV_ARVR_S_GAME_BIT |
                          EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT | EVT_GRP_RPT_GRAVITY_BIT | EVT_GRP_RPT_CAL_GYRO_BIT | EVT_GRP_RPT_UNCAL_GYRO_BIT |
                          EVT_GRP_RPT_CAL_MAGNETOMETER_BIT | EVT_GRP_RPT_TAP_DETECTOR_BIT | EVT_GRP_RPT_STEP_COUNTER_BIT |
                          EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT | EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT | EVT_GRP_RPT_RAW_ACCELEROMETER_BIT |
                          EVT_GRP_RPT_RAW_GYRO_BIT | EVT_GRP_RPT_RAW_MAGNETOMETER_BIT | EVT_GRP_RPT_UNCAL_MAGNETOMETER_BIT |
                          EVT_GRP_RPT_SHAKE_DETECTOR_BIT | EVT_GRP_RPT_ACCELEROMETER_BIT | EVT_GRP_RPT_GEOMAG_RV_BIT |
                          EVT_GRP_RPT_GYRO_INTEGRATED_RV_BIT
    };

    /// @brief Bits for evt_grp_bno08x_task
    enum bno08x_tsk_bit_t : EventBits_t
    {
        EVT_GRP_BNO08x_TASKS_RUNNING =
                (1UL << 0U), ///< When this bit is set it indicates the BNO08x tasks are running, it is always set to 1 for the duration of the BNO08x driver object. Cleared in the destructor for safe task deletion.
        EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT =
                (1UL << 1U), ///< When this bit is set it indicates the BNO08x has asserted its host interrupt pin, thus an SPI transaction should commence.
        EVT_GRP_BNO08x_TASK_RESET_OCCURRED =
                (1UL << 2U), ///< When this bit is set it indicates the SH2 HAL lib has reset the IMU, any reports enabled by the user must be re-enabled.
        EVT_GRP_BNO08x_TASK_DATA_AVAILABLE =
                (1UL << 3U) ///< When this bit is set it indicates a report has been received for the user to read, cleared in data_available() set/cleared in handle_sensor_report().
    };
};