/**
 * @file BNO08x.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// etl includes
#include <etl/vector.h>
#include <etl/variant.h>
#include <etl/map.h>

// esp-idf includes
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
// in-house includes
#include "BNO08xGlobalTypes.hpp"
#include "BNO08xPrivateTypes.hpp"
#include "BNO08xSH2HAL.hpp"
#include "BNO08xReports.hpp"

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
        bool soft_reset();
        BNO08xResetReason get_reset_reason();

        bool on();
        bool sleep();

        bool get_frs(uint16_t frs_ID, uint32_t (&data)[16], uint16_t& rx_data_sz);

        bool data_available();
        bool register_cb(std::function<void(void)> cb_fxn);
        bool register_cb(std::function<void(uint8_t report_ID)> cb_fxn);

        void print_product_ids();

        // enum helper fxns
        static const char* activity_to_str(BNO08xActivity activity);
        static const char* stability_to_str(BNO08xStability stability);
        static const char* accuracy_to_str(BNO08xAccuracy accuracy);

        BNO08xRptAcceleration accelerometer;
        BNO08xRptLinearAcceleration linear_accelerometer;
        BNO08xRptGravity gravity;
        BNO08xRptCalMagnetometer cal_magnetometer;
        BNO08xRptUncalMagnetometer uncal_magnetometer;
        BNO08xRptCalGyro cal_gyro;
        BNO08xRptUncalGyro uncal_gyro;
        BNO08xRptRV rv;
        BNO08xRptGameRV rv_game;
        BNO08xRptARVRStabilizedRV rv_ARVR_stabilized;
        BNO08xRptARVRStabilizedGameRV rv_ARVR_stabilized_game;
        BNO08xRptIGyroRV rv_gyro_integrated;
        BNO08xRptRVGeomag rv_geomagnetic;
        BNO08xRptRawMEMSGyro raw_gyro;
        BNO08xRptRawMEMSAccelerometer raw_accelerometer;
        BNO08xRptRawMEMSMagnetometer raw_magnetometer;
        BNO08xRptStepCounter step_counter;
        BNO08xRptActivityClassifier activity_classifier;
        BNO08xStabilityClassifier stability_classifier;
        BNO08xShakeDetector shake_detector;
        BNO08xTapDetector tap_detector;

    private:
        // data processing task
        static const constexpr configSTACK_DEPTH_TYPE DATA_PROC_TASK_SZ =
                CONFIG_ESP32_BNO08X_DATA_PROC_TASK_SZ; ///< Size of data_proc_task() stack in bytes
        TaskHandle_t data_proc_task_hdl;               ///<data_proc_task() task handle
        static void data_proc_task_trampoline(void* arg);
        void data_proc_task();

        // sh2 service task
        static const constexpr configSTACK_DEPTH_TYPE SH2_HAL_SERVICE_TASK_SZ =
                CONFIG_ESP32_BNO08X_SH2_HAL_SERVICE_TASK_SZ; ///< Size of sh2_HAL_service_task() stack in bytes
        TaskHandle_t sh2_HAL_service_task_hdl;               ///<sh2_HAL_service_task() task handle
        static void sh2_HAL_service_task_trampoline(void* arg);
        void sh2_HAL_service_task();

        // callback task
        static const constexpr configSTACK_DEPTH_TYPE CB_TASK_SZ = CONFIG_ESP32_BNO08X_CB_TASK_SZ; ///< Size of sh2_HAL_service_task() stack in bytes
        TaskHandle_t cb_task_hdl;                                                                  ///<sh2_HAL_service_task() task handle
        static void cb_task_trampoline(void* arg);
        void cb_task();

        SemaphoreHandle_t sh2_HAL_lock;   ///<Mutex to prevent sh2 HAL lib functions from being accessed at same time.
        SemaphoreHandle_t data_lock;      ///<Mutex to prevent user from reading data while data_proc_task() updates it, and vice versa.
        SemaphoreHandle_t sem_kill_tasks; ///<Counting Semaphore to count amount of killed tasks.

        void lock_sh2_HAL();
        void unlock_sh2_HAL();
        void lock_user_data();
        void unlock_user_data();

        void handle_sensor_report(sh2_SensorValue_t* sensor_val);
        void handle_cb(uint8_t rpt_ID, BNO08xCbGeneric* cb_entry);

        esp_err_t init_config_args();
        esp_err_t init_gpio();
        esp_err_t init_gpio_inputs();
        esp_err_t init_gpio_outputs();
        esp_err_t init_hint_isr();
        esp_err_t init_spi();
        esp_err_t init_tasks();
        esp_err_t init_sh2_HAL();

        esp_err_t deinit_gpio();
        esp_err_t deinit_gpio_inputs();
        esp_err_t deinit_gpio_outputs();
        esp_err_t deinit_hint_isr();
        esp_err_t deinit_spi();
        esp_err_t deinit_tasks();
        esp_err_t deinit_sh2_HAL();

        esp_err_t wait_for_hint();
        esp_err_t wait_for_reset();

        void toggle_reset();

        esp_err_t re_enable_reports();

        sh2_Hal_t sh2_HAL; ///< sh2 hardware abstraction layer struct for use with sh2 HAL lib.

        EventGroupHandle_t evt_grp_bno08x_task; ///<Event group for indicating various BNO08x related events between tasks.
        EventGroupHandle_t evt_grp_report_en;   ///<Event group for indicating which reports are currently enabled.
        EventGroupHandle_t
                evt_grp_report_data_available; ///< Event group for indicating to BNO08xRpt::has_new_data() that a module received a new report since the last time it was called (note this group is unaffected by data read through callbacks).

        QueueHandle_t
                queue_rx_sensor_event; ///< Queue to send sensor events from sh2 HAL sensor event callback (BNO08xSH2HAL::sensor_event_cb()) to data_proc_task()

        QueueHandle_t queue_cb_report_id; ///< Queue to send report ID of most recent report to cb_task()

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle
        BNO08xPrivateTypes::bno08x_init_status_t
                init_status; ///<Initialization status of various functionality, used by deconstructor during cleanup, set during initialization.

        sh2_ProductIds_t product_IDs; ///< Product ID info returned IMU at initialization, can be viewed with print_product_ids()

        BNO08xPrivateTypes::bno08x_cb_list_t cb_list; ///< Vector to contain registered callbacks.

        etl::vector<uint8_t, TOTAL_RPT_COUNT> en_report_ids; ///< Vector to contain IDs of currently enabled reports

        // clang-format off
        etl::map<uint8_t, BNO08xRpt*, TOTAL_RPT_COUNT, etl::less<uint8_t>> usr_reports = 
        {
                {SH2_ACCELEROMETER, &accelerometer},
                {SH2_LINEAR_ACCELERATION, &linear_accelerometer}, 
                {SH2_GRAVITY, &gravity}, 
                {SH2_MAGNETIC_FIELD_CALIBRATED, &cal_magnetometer},
                {SH2_MAGNETIC_FIELD_UNCALIBRATED, &uncal_magnetometer}, 
                {SH2_GYROSCOPE_CALIBRATED, &cal_gyro},
                {SH2_GYROSCOPE_UNCALIBRATED, &uncal_gyro}, 
                {SH2_ROTATION_VECTOR, &rv}, 
                {SH2_GAME_ROTATION_VECTOR, &rv_game},
                {SH2_ARVR_STABILIZED_RV, &rv_ARVR_stabilized}, 
                {SH2_ARVR_STABILIZED_GRV, &rv_ARVR_stabilized_game},
                {SH2_GYRO_INTEGRATED_RV, &rv_gyro_integrated}, 
                {SH2_GEOMAGNETIC_ROTATION_VECTOR, &rv_geomagnetic}, 
                {SH2_RAW_GYROSCOPE, &raw_gyro},
                {SH2_RAW_ACCELEROMETER, &raw_accelerometer}, 
                {SH2_RAW_MAGNETOMETER, &raw_magnetometer}, 
                {SH2_STEP_COUNTER, &step_counter},
                {SH2_PERSONAL_ACTIVITY_CLASSIFIER, &activity_classifier}, 
                {SH2_STABILITY_CLASSIFIER, &stability_classifier},
                {SH2_SHAKE_DETECTOR, &shake_detector}, 
                {SH2_TAP_DETECTOR, &tap_detector},
                
                // not implemented, see include/report for existing implementations to add your own
                {SH2_PRESSURE, nullptr}, // requires auxilary i2c sensor
                {SH2_AMBIENT_LIGHT, nullptr},  // requires auxilary i2c sensor
                {SH2_HUMIDITY, nullptr},  // requires auxilary i2c sensor
                {SH2_PROXIMITY, nullptr},  // requires auxilary i2c sensor
                {SH2_TEMPERATURE, nullptr},  // requires auxilary i2c sensor
                {SH2_HEART_RATE_MONITOR, nullptr},  // requires auxilary i2c sensor
                {SH2_STEP_DETECTOR, nullptr},
                {SH2_SIGNIFICANT_MOTION, nullptr},
                {SH2_FLIP_DETECTOR, nullptr},
                {SH2_PICKUP_DETECTOR, nullptr},
                {SH2_STABILITY_DETECTOR, nullptr},
                {SH2_SLEEP_DETECTOR, nullptr},
                {SH2_TILT_DETECTOR, nullptr},
                {SH2_POCKET_DETECTOR, nullptr},
                {SH2_CIRCLE_DETECTOR, nullptr},
                {SH2_IZRO_MOTION_REQUEST, nullptr}
        };
        // clang-format on

        static void IRAM_ATTR hint_handler(void* arg);

        static const constexpr uint16_t RX_DATA_LENGTH = 300U; ///<length buffer containing data received over spi

        static const constexpr TickType_t HOST_INT_TIMEOUT_DEFAULT_MS =
                CONFIG_ESP32_BNO08X_HINT_TIMEOUT_MS /
                portTICK_PERIOD_MS; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in miliseconds).

        static const constexpr TickType_t DATA_AVAILABLE_TIMEOUT_MS =
                CONFIG_ESP32_BNO08X_DATA_AVAILABLE_TIMEOUT_MS /
                portTICK_PERIOD_MS; ///<Max wait between data_available() being called and no new data/report being detected.

        static const constexpr TickType_t HARD_RESET_DELAY_MS =
                CONFIG_ESP32_BNO08X_HARD_RESET_DELAY_MS /
                portTICK_PERIOD_MS; ///<How long RST pin is held low during hard reset (min 10ns according to datasheet, but should be longer for stable operation)

        static const constexpr uint32_t SCLK_MAX_SPEED = 3000000UL; ///<Max SPI SCLK speed BNO08x is capable of.

      

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements

        friend class BNO08xSH2HAL;
        friend class BNO08xTestHelper;
};