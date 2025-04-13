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
        bool disable_all_reports();
        BNO08xResetReason get_reset_reason();

        bool on();
        bool sleep();

        // bool calibration_turntable_start(uint32_t period_us);
        // bool calibration_turntable_end(sh2_CalStatus_t& status);

        bool dynamic_calibration_enable(BNO08xCalSel sensor);
        bool dynamic_calibration_disable(BNO08xCalSel sensor);
        bool dynamic_calibration_autosave_enable();
        bool dynamic_calibration_autosave_disable();
        bool dynamic_calibration_save();
        bool dynamic_calibration_clear();
        bool dynamic_calibration_run_routine(); 

        bool delete_calibration_data();

        constexpr static float SQRT2OVER2 = 0.7071067811865476f; // sqrt(2)/2, used for setting system orientation
        bool set_system_orientation(float w, float x, float y, float z);
        bool get_system_orientation(float &w, float &x, float &y, float &z);

        bool get_frs(BNO08xFrsID frs_ID, uint32_t (&data)[16], uint16_t& rx_data_sz);
        bool write_frs(BNO08xFrsID frs_ID, uint32_t *data, const uint16_t tx_data_sz);
        sh2_ProductIds_t get_product_IDs();

        bool data_available();
        bool register_cb(std::function<void(void)> cb_fxn);
        bool register_cb(std::function<void(uint8_t report_ID)> cb_fxn);

        void print_product_ids();
        void print_system_orientation();

        // enum helper fxns
        static const char* activity_to_str(BNO08xActivity activity);
        static const char* stability_to_str(BNO08xStability stability);
        static const char* accuracy_to_str(BNO08xAccuracy accuracy);

        /// @brief Contains report implementations.
        typedef struct bno08x_reports_t
        {
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
                BNO08xRptStabilityClassifier stability_classifier;
                BNO08xRptShakeDetector shake_detector;
                BNO08xRptTapDetector tap_detector;

                bno08x_reports_t(BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
                    : accelerometer(SH2_ACCELEROMETER, BNO08xPrivateTypes::EVT_GRP_RPT_ACCELEROMETER_BIT, sync_ctx)
                    , linear_accelerometer(
                              SH2_LINEAR_ACCELERATION, BNO08xPrivateTypes::EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT, sync_ctx)
                    , gravity(SH2_GRAVITY, BNO08xPrivateTypes::EVT_GRP_RPT_GRAVITY_BIT, sync_ctx)
                    , cal_magnetometer(
                              SH2_MAGNETIC_FIELD_CALIBRATED, BNO08xPrivateTypes::EVT_GRP_RPT_CAL_MAGNETOMETER_BIT, sync_ctx)
                    , uncal_magnetometer(
                              SH2_MAGNETIC_FIELD_UNCALIBRATED, BNO08xPrivateTypes::EVT_GRP_RPT_UNCAL_MAGNETOMETER_BIT, sync_ctx)
                    , cal_gyro(SH2_GYROSCOPE_CALIBRATED, BNO08xPrivateTypes::EVT_GRP_RPT_CAL_GYRO_BIT, sync_ctx)
                    , uncal_gyro(SH2_GYROSCOPE_UNCALIBRATED, BNO08xPrivateTypes::EVT_GRP_RPT_UNCAL_GYRO_BIT, sync_ctx)
                    , rv(SH2_ROTATION_VECTOR, BNO08xPrivateTypes::EVT_GRP_RPT_RV_BIT, sync_ctx)
                    , rv_game(SH2_GAME_ROTATION_VECTOR, BNO08xPrivateTypes::EVT_GRP_RPT_RV_GAME_BIT, sync_ctx)
                    , rv_ARVR_stabilized(SH2_ARVR_STABILIZED_RV, BNO08xPrivateTypes::EVT_GRP_RPT_RV_ARVR_S_BIT, sync_ctx)
                    , rv_ARVR_stabilized_game(
                              SH2_ARVR_STABILIZED_GRV, BNO08xPrivateTypes::EVT_GRP_RPT_RV_ARVR_S_GAME_BIT, sync_ctx)
                    , rv_gyro_integrated(SH2_GYRO_INTEGRATED_RV, BNO08xPrivateTypes::EVT_GRP_RPT_GYRO_INTEGRATED_RV_BIT, sync_ctx)
                    , rv_geomagnetic(SH2_GEOMAGNETIC_ROTATION_VECTOR, BNO08xPrivateTypes::EVT_GRP_RPT_GEOMAG_RV_BIT, sync_ctx)
                    , raw_gyro(SH2_RAW_GYROSCOPE, BNO08xPrivateTypes::EVT_GRP_RPT_RAW_GYRO_BIT, sync_ctx)
                    , raw_accelerometer(SH2_RAW_ACCELEROMETER, BNO08xPrivateTypes::EVT_GRP_RPT_RAW_ACCELEROMETER_BIT, sync_ctx)
                    , raw_magnetometer(SH2_RAW_MAGNETOMETER, BNO08xPrivateTypes::EVT_GRP_RPT_RAW_MAGNETOMETER_BIT, sync_ctx)
                    , step_counter(SH2_STEP_COUNTER, BNO08xPrivateTypes::EVT_GRP_RPT_STEP_COUNTER_BIT, sync_ctx)
                    , activity_classifier(
                              SH2_PERSONAL_ACTIVITY_CLASSIFIER, BNO08xPrivateTypes::EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT, sync_ctx)
                    , stability_classifier(
                              SH2_STABILITY_CLASSIFIER, BNO08xPrivateTypes::EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT, sync_ctx)
                    , shake_detector(SH2_SHAKE_DETECTOR, BNO08xPrivateTypes::EVT_GRP_RPT_SHAKE_DETECTOR_BIT, sync_ctx)
                    , tap_detector(SH2_TAP_DETECTOR, BNO08xPrivateTypes::EVT_GRP_RPT_TAP_DETECTOR_BIT, sync_ctx)
                {
                }
        } bno08x_reports_t;

        bno08x_reports_t rpt;

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
        static const constexpr configSTACK_DEPTH_TYPE CB_TASK_SZ =
                CONFIG_ESP32_BNO08X_CB_TASK_SZ; ///< Size of sh2_HAL_service_task() stack in bytes
        TaskHandle_t cb_task_hdl;               ///<sh2_HAL_service_task() task handle
        static void cb_task_trampoline(void* arg);
        void cb_task();

        static const constexpr BaseType_t CB_TASK_AFFINITY = 
                CONFIG_ESP32_BNO08X_CB_TASK_AFFINITY < 0 ? tskNO_AFFINITY : CONFIG_ESP32_BNO08X_CB_TASK_AFFINITY ;  /// tskNO_AFFINITY if not pinned to a core, 0 or 1
        static const constexpr UBaseType_t CB_TASK_PRIORITY = CONFIG_ESP32_BNO08X_CB_TASK_PRIORITY; /// 5 per default, Priority of the callback task, 0-25, 0 is lowest priority, 25 is highest priority

        static const constexpr BaseType_t DATA_PROC_TASK_AFFINITY = 
                CONFIG_ESP32_BNO08X_DATA_PROC_TASK_AFFINITY < 0 ? tskNO_AFFINITY : CONFIG_ESP32_BNO08X_DATA_PROC_TASK_AFFINITY; /// tskNO_AFFINITY if not pinned to a core, 0 or 1
        static const constexpr UBaseType_t DATA_PROC_TASK_PRIORITY = CONFIG_ESP32_BNO08X_DATA_PROC_TASK_PRIORITY; /// 6 per default, Priority of the data processing task, 0-25, 0 is lowest priority, 25 is highest priority
        
        static const constexpr BaseType_t SH2_HAL_SERVICE_TASK_AFFINITY = 
                CONFIG_ESP32_BNO08X_SH2_HAL_SERVICE_TASK_AFFINITY < 0 ? tskNO_AFFINITY : CONFIG_ESP32_BNO08X_SH2_HAL_SERVICE_TASK_AFFINITY; /// tskNO_AFFINITY if not pinned to a core, 0 or 1
        static const constexpr UBaseType_t SH2_HAL_SERVICE_TASK_PRIORITY = CONFIG_ESP32_BNO08X_SH2_HAL_SERVICE_TASK_PRIORITY; /// 7 per default, Priority of the sh2 HAL service task, 0-25, 0 is lowest priority, 25 is highest priority


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
        BNO08xPrivateTypes::bno08x_sync_ctx_t sync_ctx; ///< Holds context used to synchronize tasks and callback execution.
        sh2_ProductIds_t product_IDs; ///< Product ID info returned IMU at initialization, can be viewed with print_product_ids()

        // clang-format off
        etl::map<uint8_t, BNO08xRpt*, TOTAL_RPT_COUNT, etl::less<uint8_t>> usr_reports = 
        {
                {SH2_ACCELEROMETER, &rpt.accelerometer},
                {SH2_LINEAR_ACCELERATION, &rpt.linear_accelerometer}, 
                {SH2_GRAVITY, &rpt.gravity}, 
                {SH2_MAGNETIC_FIELD_CALIBRATED, &rpt.cal_magnetometer},
                {SH2_MAGNETIC_FIELD_UNCALIBRATED, &rpt.uncal_magnetometer}, 
                {SH2_GYROSCOPE_CALIBRATED, &rpt.cal_gyro},
                {SH2_GYROSCOPE_UNCALIBRATED, &rpt.uncal_gyro}, 
                {SH2_ROTATION_VECTOR, &rpt.rv}, 
                {SH2_GAME_ROTATION_VECTOR, &rpt.rv_game},
                {SH2_ARVR_STABILIZED_RV, &rpt.rv_ARVR_stabilized}, 
                {SH2_ARVR_STABILIZED_GRV, &rpt.rv_ARVR_stabilized_game},
                {SH2_GYRO_INTEGRATED_RV, &rpt.rv_gyro_integrated}, 
                {SH2_GEOMAGNETIC_ROTATION_VECTOR, &rpt.rv_geomagnetic}, 
                {SH2_RAW_GYROSCOPE, &rpt.raw_gyro},
                {SH2_RAW_ACCELEROMETER, &rpt.raw_accelerometer}, 
                {SH2_RAW_MAGNETOMETER, &rpt.raw_magnetometer}, 
                {SH2_STEP_COUNTER, &rpt.step_counter},
                {SH2_PERSONAL_ACTIVITY_CLASSIFIER, &rpt.activity_classifier}, 
                {SH2_STABILITY_CLASSIFIER, &rpt.stability_classifier},
                {SH2_SHAKE_DETECTOR, &rpt.shake_detector}, 
                {SH2_TAP_DETECTOR, &rpt.tap_detector},
                
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