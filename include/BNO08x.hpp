/**
 * @file BNO08x.hpp
 * @author Myles Parfeniuk
 */
#pragma once
// standard library includes
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <functional>
#include <vector>

// hill-crest labs includes (apache 2.0 license, compatible with MIT)
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// esp-idf includes
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <rom/ets_sys.h>

// in-house includes
#include "BNO08x_global_types.hpp"

/**
 * @class BNO08x
 *
 * @brief BNO08x IMU driver class.
 * */
class BNO08x
{
    public:
        BNO08x(bno08x_config_t imu_config = bno08x_config_t());
        ~BNO08x();
        bool initialize();

        bool hard_reset();

        void register_cb(std::function<void()> cb_fxn);

    private:
        /// @brief Holds data that is sent and received over spi.
        typedef struct sh2_packet_t
        {
                uint8_t header[4]; ///< Header of SHTP packet.
                uint8_t body[300]; /// Body of SHTP packet.
                uint16_t length;   ///< Packet length in bytes.
        } sh2_packet_t;

        /// @brief Holds info about which functionality has been successfully initialized (used by deconstructor during cleanup).
        typedef struct bno08x_init_status_t
        {
                bool gpio_outputs;   ///< True if GPIO outputs have been initialized.
                bool gpio_inputs;    ///< True if GPIO inputs have been initialized.
                bool isr_service;    ///< True if global ISR service has been initialized.
                bool isr_handler;    ///< True if HINT ISR handler has been initialized.
                uint8_t task_count;  ///< How many successfully initialized tasks (max of TSK_CNT)
                bool data_proc_task; ///< True if xTaskCreate has been called successfully for data_proc_task.
                bool spi_task;       ///< True if xTaskCreate has been called successfully for spi_task.
                bool spi_bus;        ///< True if spi_bus_initialize() has been called successfully.
                bool spi_device;     ///< True if spi_bus_add_device() has been called successfully.

                bno08x_init_status_t()
                    : gpio_outputs(false)
                    , gpio_inputs(false)
                    , isr_service(false)
                    , isr_handler(false)
                    , task_count(0)
                    , data_proc_task(false)
                    , spi_task(false)
                    , spi_bus(false)
                    , spi_device(false)
                {
                }
        } bno08x_init_status_t;

        esp_err_t init_config_args();
        esp_err_t init_gpio();
        esp_err_t init_gpio_inputs();
        esp_err_t init_gpio_outputs();
        esp_err_t init_hint_isr();
        esp_err_t init_spi();

        esp_err_t deinit_gpio();
        esp_err_t deinit_gpio_inputs();
        esp_err_t deinit_gpio_outputs();
        esp_err_t deinit_hint_isr();
        esp_err_t deinit_spi();

        esp_err_t transmit_packet();
        esp_err_t transmit_packet_header(sh2_packet_t* rx_packet, sh2_packet_t* tx_packet);
        esp_err_t transmit_packet_body(sh2_packet_t* rx_packet, sh2_packet_t* tx_packet);

        // for debug
        void print_header(sh2_packet_t* packet);
        void print_packet(sh2_packet_t* packet);
        bool first_boot = true; ///< true only for first execution of hard_reset(), used to suppress the printing of product ID report.

        // spi task
        TaskHandle_t spi_task_hdl; ///<spi_task() handle
        static void spi_task_trampoline(void* arg);
        void spi_task();

        // data processing task
        TaskHandle_t data_proc_task_hdl; ///<data_proc_task() task handle
        static void data_proc_task_trampoline(void* arg);
        void data_proc_task();

        // task control
        SemaphoreHandle_t sem_kill_tasks; ///<semaphore to count amount of killed tasks
        esp_err_t launch_tasks();
        esp_err_t kill_all_tasks();

        EventGroupHandle_t
                evt_grp_spi; ///<Event group for indicating when bno08x hint pin has triggered and when new data has been processed. Used by calls to sending or receiving functions.
        EventGroupHandle_t evt_grp_report_en; ///<Event group for indicating which reports are currently enabled.
        EventGroupHandle_t evt_grp_task_flow; ///<Event group for indicating when tasks should complete and self-delete (on deconstructor call)

        QueueHandle_t queue_rx_data; ///<Packet queue used to send data received from bno08x from spi_task to data_proc_task.
        QueueHandle_t queue_tx_data; ///<Packet queue used to send data to be sent over SPI from sending functions to spi_task.

        std::vector<std::function<void()>> cb_list; // Vector for storing any call-back functions added with register_cb()

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle
        bno08x_init_status_t
                init_status; ///<Initialization status of various functionality, used by deconstructor during cleanup, set during initialization.

        static void IRAM_ATTR hint_handler(void* arg);

        static const constexpr uint8_t TASK_CNT = 2U; ///<Total amount of tasks utilized by BNO08x driver library

        static const constexpr uint16_t RX_DATA_LENGTH = 300U; ///<length buffer containing data received over spi

        static const constexpr TickType_t HOST_INT_TIMEOUT_DEFAULT_MS =
                3000UL /
                portTICK_PERIOD_MS; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in miliseconds), when no reports are enabled (ie during reset)

        static const constexpr TickType_t HARD_RESET_DELAY_MS =
                100UL /
                portTICK_PERIOD_MS; ///<How long RST pin is held low during hard reset (min 10ns according to datasheet, but should be longer for stable operation)

        static const constexpr uint32_t SCLK_MAX_SPEED = 3000000UL; ///<Max SPI SCLK speed BNO08x is capable of

        // evt_grp_spi bits
        static const constexpr EventBits_t EVT_GRP_SPI_RX_DONE_BIT =
                (1U << 0U); ///<When this bit is set it indicates a receive procedure has completed.
        static const constexpr EventBits_t EVT_GRP_SPI_RX_VALID_PACKET_BIT =
                (1U << 1U); ///< When this bit is set, it indicates a valid packet has been received and processed.
        static const constexpr EventBits_t EVT_GRP_SPI_RX_INVALID_PACKET_BIT =
                (1U << 2U); ///<When this bit is set, it indicates an invalid packet has been received.
        static const constexpr EventBits_t EVT_GRP_SPI_TX_DONE_BIT = (1 << 3); ///<When this bit is set, it indicates a queued packet has been sent.

        // evt_grp_report_en bits
        static const constexpr EventBits_t EVT_GRP_RPT_ROTATION_VECTOR_BIT_EN = (1 << 0);      ///< When set, rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT_EN = (1 << 1); ///< When set, game rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT_EN =
                (1U << 2U); ///< When set, ARVR stabilized rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT_EN =
                (1U << 3U); ///< When set, ARVR stabilized game rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT_EN =
                (1U << 4U); ///< When set, gyro integrator rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ACCELEROMETER_BIT_EN = (1U << 5U); ///< When set, accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT_EN =
                (1U << 6U);                                                                   ///< When set, linear accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GRAVITY_BIT_EN = (1U << 7U);           ///< When set, gravity reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_BIT_EN = (1U << 8U);              ///< When set, gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT_EN = (1U << 9U); ///< When set, uncalibrated gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_MAGNETOMETER_BIT_EN = (1U << 10U);     ///< When set, magnetometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_TAP_DETECTOR_BIT_EN = (1U << 11U);     ///< When set, tap detector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STEP_COUNTER_BIT_EN = (1U << 12U);     ///< When set, step counter reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT_EN =
                (1U << 13U); ///< When set, stability classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT_EN =
                (1U << 14U);                                                                   ///< When set, activity classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_ACCELEROMETER_BIT_EN = (1U << 15U); ///< When set, raw accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_GYRO_BIT_EN = (1U << 16U);          ///< When set, raw gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_MAGNETOMETER_BIT_EN = (1U << 17U);  ///< When set, raw magnetometer reports are active.

        static const constexpr EventBits_t EVT_GRP_RPT_ALL_BITS =
                EVT_GRP_RPT_ROTATION_VECTOR_BIT_EN | EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT_EN | EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT_EN |
                EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT_EN | EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT_EN | EVT_GRP_RPT_ACCELEROMETER_BIT_EN |
                EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT_EN | EVT_GRP_RPT_GRAVITY_BIT_EN | EVT_GRP_RPT_GYRO_BIT_EN |
                EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT_EN | EVT_GRP_RPT_MAGNETOMETER_BIT_EN | EVT_GRP_RPT_TAP_DETECTOR_BIT_EN |
                EVT_GRP_RPT_STEP_COUNTER_BIT_EN | EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT_EN | EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT_EN |
                EVT_GRP_RPT_RAW_ACCELEROMETER_BIT_EN | EVT_GRP_RPT_RAW_GYRO_BIT_EN | EVT_GRP_RPT_RAW_MAGNETOMETER_BIT_EN;

        // evt_grp_task_flow bits
        static const constexpr EventBits_t EVT_GRP_TSK_FLW_RUNNING_BIT =
                (1U << 0U); ///< When set, data_proc_task and spi_task are active, when 0 they are pending deletion or deleted.

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements
};