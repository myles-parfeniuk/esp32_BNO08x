/**
 * @file BNO08x.hpp
 * @author Myles Parfeniuk
 */
#pragma once
// standard library includes
#include <inttypes.h>
#include <stdio.h>
#include <cstring>
#include <functional>
#include <vector>

// esp-idf includes
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <rom/ets_sys.h>

// in-house includes
#include "BNO08x_global_types.hpp"
#include "BNO08xSH2HAL.hpp"

/**
 * @class BNO08x
 *
 * @brief BNO08x IMU driver class.
 * */
class BNO08x
{
    public:
        inline static sh2_SensorConfig default_sensor_cfg = {.changeSensitivityEnabled = false, ///<TODO
                .changeSensitivityRelative = false,
                .wakeupEnabled = false,
                .alwaysOnEnabled = false,
                .changeSensitivity = 0,
                .reportInterval_us = 0,
                .batchInterval_us = 0,
                .sensorSpecific = 0};

        BNO08x(bno08x_config_t imu_config = bno08x_config_t());
        ~BNO08x();
        bool initialize();

        void hard_reset();

        bool enable_rotation_vector(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_game_rotation_vector(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_ARVR_stabilized_rotation_vector(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_ARVR_stabilized_game_rotation_vector(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_gravity(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_linear_accelerometer(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_accelerometer(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_calibrated_magnetometer(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_step_counter(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_activity_classifier(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_calibrated_gyro(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_raw_mems_gyro(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_raw_mems_accelerometer(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool enable_raw_mems_magnetometer(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);

        bno08x_quat_t get_rotation_vector_quat();
        bno08x_euler_angle_t get_rotation_vector_euler(bool in_degrees = true);
        bno08x_quat_t get_game_rotation_vector_quat();
        bno08x_euler_angle_t get_game_rotation_vector_euler(bool in_degrees = true);
        bno08x_quat_t get_ARVR_stabilized_rotation_vector_quat();
        bno08x_euler_angle_t get_ARVR_stabilized_rotation_vector_euler(bool in_degrees = true);
        bno08x_quat_t get_ARVR_stabilized_game_rotation_vector_quat();
        bno08x_euler_angle_t get_ARVR_stabilized_game_rotation_vector_euler(bool in_degrees = true);

        bno08x_accel_data_t get_gravity();
        bno08x_accel_data_t get_linear_accel();
        bno08x_accel_data_t get_accel();
        bno08x_magf_data_t get_calibrated_magnetometer();
        bno08x_gyro_data_t get_calibrated_gyro();
        bno08x_raw_gyro_data_t get_raw_mems_gyro();
        bno08x_raw_accel_data_t get_raw_mems_accelerometer();
        bno08x_raw_magf_data_t get_raw_mems_magnetometer();
        bno08x_step_counter_data_t get_step_counter();

        void register_cb(std::function<void()> cb_fxn);
        void print_product_ids();

    private:
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
                uint8_t task_init_cnt;     ///< Amount of tasks that have been successfully initialized.

                bno08x_init_status_t()
                    : gpio_outputs(false)
                    , gpio_inputs(false)
                    , isr_service(false)
                    , isr_handler(false)
                    , spi_bus(false)
                    , spi_device(false)
                    , data_proc_task(false)
                    , sh2_HAL_service_task(false)
                    , task_init_cnt(0)
                {
                }
        } bno08x_init_status_t;

        /// @brief Holds data returned from sensor reports.
        typedef struct bno08x_data_t
        {
                bno08x_accel_data_t gravity;
                bno08x_accel_data_t linear_acceleration;
                bno08x_accel_data_t acceleration;
                bno08x_magf_data_t cal_magnetometer;
                bno08x_step_counter_data_t step_counter;
                bno08x_activity_classifier_data_t activity_classifier;
                bno08x_gyro_data_t cal_gyro;
                bno08x_raw_gyro_data_t mems_raw_gyro;
                bno08x_raw_accel_data_t mems_raw_accel;
                bno08x_raw_magf_data_t mems_raw_magnetometer;
                bno08x_quat_t rotation_vector;
                bno08x_quat_t game_rotation_vector;
                bno08x_quat_t arvr_s_rotation_vector;
                bno08x_quat_t arvr_s_game_rotation_vector;

                bno08x_data_t()
                    : gravity({0.0f, 0.0f, 0.0f})
                    , linear_acceleration({0.0f, 0.0f, 0.0f})
                    , acceleration({0.0f, 0.0f, 0.0f})
                    , cal_magnetometer({0.0f, 0.0f, 0.0f})
                    , step_counter({0UL, 0U})
                    , activity_classifier(bno08x_activity_classifier_data_t())
                    , cal_gyro({0.0f, 0.0f, 0.0f})
                    , mems_raw_gyro({0, 0, 0, 0, 0UL})
                    , mems_raw_accel({0, 0, 0, 0UL})
                    , mems_raw_magnetometer({0, 0, 0, 0UL})
                    , rotation_vector(bno08x_quat_t())
                    , game_rotation_vector(bno08x_quat_t())

                {
                }

        } bno08x_data_t;

        typedef struct bno08x_usr_report_periods_t
        {
                uint32_t gravity;
                uint32_t linear_accelerometer;
                uint32_t accelerometer;
                uint32_t cal_magnetometer;
                uint32_t step_counter;
                uint32_t activity_classifier;
                uint32_t cal_gyro;
                uint32_t raw_mems_gyro;
                uint32_t raw_mems_accelerometer;
                uint32_t raw_mems_magnetometer;
                uint32_t rotation_vector;
                uint32_t game_rotation_vector;
                uint32_t arvr_s_rotation_vector;
                uint32_t arvr_s_game_rotation_vector;

                bno08x_usr_report_periods_t()
                    : gravity(0UL)
                    , linear_accelerometer(0UL)
                    , accelerometer(0UL)
                    , cal_magnetometer(0UL)
                    , step_counter(0UL)
                    , activity_classifier(0UL)
                    , cal_gyro(0UL)
                    , raw_mems_gyro(0UL)
                    , raw_mems_accelerometer(0UL)
                    , raw_mems_magnetometer(0UL)
                    , rotation_vector(0UL)
                    , game_rotation_vector(0UL)
                    , arvr_s_rotation_vector(0UL)
                    , arvr_s_game_rotation_vector(0UL)
                {
                }
        } bno08x_usr_report_periods_t;

        bno08x_data_t data;                              ///< Holds all data returned from enabled reports.
        bno08x_usr_report_periods_t user_report_periods; ///< Holds periods for reports enabled by user (0 == disabled report)

        // data processing task
        TaskHandle_t data_proc_task_hdl; ///<data_proc_task() task handle
        static void data_proc_task_trampoline(void* arg);
        void data_proc_task();

        // sh2 service task
        TaskHandle_t sh2_HAL_service_task_hdl; ///<sh2_HAL_service_task() task handle
        static void sh2_HAL_service_task_trampoline(void* arg);
        void sh2_HAL_service_task();

        SemaphoreHandle_t sh2_HAL_lock;   ///<Mutex to prevent sh2 HAL lib functions from being accessed at same time.
        SemaphoreHandle_t data_lock;      ///<Mutex to prevent user from reading data while data_proc_task() updates it, and vice versa.
        SemaphoreHandle_t sem_kill_tasks; ///<Counting Semaphore to count amount of killed tasks.

        void lock_sh2_HAL();
        void unlock_sh2_HAL();
        void lock_user_data();
        void unlock_user_data();

        void handle_sensor_report(sh2_SensorValue_t* sensor_val);
        void update_rotation_vector_data(sh2_SensorValue_t* sensor_val);
        void update_game_rotation_vector_data(sh2_SensorValue_t* sensor_val);
        void update_arvr_s_rotation_vector_data(sh2_SensorValue_t* sensor_val);
        void update_arvr_s_game_rotation_vector_data(sh2_SensorValue_t* sensor_val);
        void update_gravity_data(sh2_SensorValue_t* sensor_val);
        void update_linear_accelerometer_data(sh2_SensorValue_t* sensor_val);
        void update_accelerometer_data(sh2_SensorValue_t* sensor_val);
        void update_calibrated_magnetometer_data(sh2_SensorValue_t* sensor_val);
        void update_step_counter_data(sh2_SensorValue_t* sensor_val);
        void update_activity_classifier_data(sh2_SensorValue_t* sensor_val);
        void update_calibrated_gyro_data(sh2_SensorValue_t* sensor_val);
        void update_raw_mems_gyro_data(sh2_SensorValue_t* sensor_val);
        void update_raw_mems_accel_data(sh2_SensorValue_t* sensor_val);
        void update_raw_mems_magnetometer_data(sh2_SensorValue_t* sensor_val);

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

        esp_err_t enable_report(sh2_SensorId_t sensor_ID, uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg);
        esp_err_t re_enable_reports();

        sh2_Hal_t sh2_HAL; ///< SH2 hardware abstraction layer struct for use with SH2 lib.

        EventGroupHandle_t evt_grp_bno08x_task; ///<Event group for indicating various BNO08x related events between tasks.
        EventGroupHandle_t evt_grp_report_en;   ///<Event group for indicating which reports are currently enabled.

        QueueHandle_t queue_rx_sensor_event; ///< TODO

        std::vector<std::function<void()>> cb_list; // Vector for storing any call-back functions added with register_cb()

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle
        bno08x_init_status_t
                init_status; ///<Initialization status of various functionality, used by deconstructor during cleanup, set during initialization.

        sh2_ProductIds_t product_IDs; ///< TODO

        static void IRAM_ATTR hint_handler(void* arg);

        static const constexpr uint16_t RX_DATA_LENGTH = 300U; ///<length buffer containing data received over spi

        static const constexpr TickType_t HOST_INT_TIMEOUT_DEFAULT_MS =
                500UL /
                portTICK_PERIOD_MS; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in miliseconds), when no reports are enabled (ie during reset)

        static const constexpr TickType_t HARD_RESET_DELAY_MS =
                100UL /
                portTICK_PERIOD_MS; ///<How long RST pin is held low during hard reset (min 10ns according to datasheet, but should be longer for stable operation)

        static const constexpr uint32_t SCLK_MAX_SPEED = 3000000UL; ///<Max SPI SCLK speed BNO08x is capable of

        static const constexpr float RAD_2_DEG =
                (180.0f / M_PI); ///< Constant for radian to degree conversions, sed in quaternion to euler function conversions.

        // evt_grp_bno08x_task bits
        static const constexpr EventBits_t EVT_GRP_BNO08x_TASKS_RUNNING =
                (1U << 0U); ///<When this bit is set it indicates the BNO08x tasks are running, it is always set to 1 for the duration BNO08x driver object. Cleared in deconstructor for safe task deletion.
        static const constexpr EventBits_t EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT =
                (1U << 1U); ///<When this bit is set it indicates the BNO08x has asserted its host interrupt pin, thus an SPI transaction should commence.
        static const constexpr EventBits_t EVT_GRP_BNO08x_TASK_RESET_OCCURRED =
                (1U << 2U); ///<When this bit is set it indicates the SH2 HAL lib has reset the IMU, any reports enabled by the user must be re-enabled.

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
        static const constexpr EventBits_t EVT_GRP_RPT_CAL_GYRO_BIT_EN = (1U << 8U);          ///< When set, gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT_EN = (1U << 9U); ///< When set, uncalibrated gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_CAL_MAGNETOMETER_BIT_EN = (1U << 10U); ///< When set, magnetometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_TAP_DETECTOR_BIT_EN = (1U << 11U);     ///< When set, tap detector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STEP_COUNTER_BIT_EN = (1U << 12U);     ///< When set, step counter reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT_EN =
                (1U << 13U); ///< When set, stability classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT_EN =
                (1U << 14U);                                                                   ///< When set, activity classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_ACCELEROMETER_BIT_EN = (1U << 15U); ///< When set, raw accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_GYRO_BIT_EN = (1U << 16U);          ///< When set, raw gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_MAGNETOMETER_BIT_EN = (1U << 17U);  ///< When set, raw magnetometer reports are active.

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements

        friend class BNO08xSH2HAL;
};