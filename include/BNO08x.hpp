#pragma once
// esp-idf includes
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <rom/ets_sys.h>

// standard library includes
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <functional>
#include <vector>

// macros
#define CHECK_TASKS_RUNNING(evt_grp_task_flow, running_bit) ((xEventGroupGetBits(evt_grp_task_flow) & (running_bit)) != 0)

/// @brief SHTP protocol channels
enum channels_t
{
    CHANNEL_COMMAND,
    CHANNEL_EXECUTABLE,
    CHANNEL_CONTROL,
    CHANNEL_REPORTS,
    CHANNEL_WAKE_REPORTS,
    CHANNEL_GYRO
};

/// @brief Sensor accuracy returned during sensor calibration
enum class IMUAccuracy
{
    LOW = 1,
    MED,
    HIGH
};

/// @brief Reason for previous IMU reset (returned by get_reset_reason())
enum class IMUResetReason
{
    UNDEFINED, ///< Undefined reset reason, this should never occur and is an error.
    POR,       ///< Previous reset was due to power on reset.
    INT_RST,   ///< Previous reset was due to internal reset.
    WTD,       ///< Previous reset was due to watchdog timer.
    EXT_RST,   ///< Previous reset was due to external reset.
    OTHER      ///< Previous reset was due to power other reason.
};

/// @brief IMU configuration settings passed into constructor
typedef struct bno08x_config_t
{
        spi_host_device_t spi_peripheral; ///<SPI peripheral to be used
        gpio_num_t io_mosi;               ///<MOSI GPIO pin (connects to BNO08x DI pin)
        gpio_num_t io_miso;               ///<MISO GPIO pin (connects to BNO08x SDA pin)
        gpio_num_t io_sclk;               ///<SCLK pin (connects to BNO08x SCL pin)
        gpio_num_t io_cs;                 /// Chip select pin (connects to BNO08x CS pin)
        gpio_num_t io_int;                /// Host interrupt pin (connects to BNO08x INT pin)
        gpio_num_t io_rst;                /// Reset pin (connects to BNO08x RST pin)
        gpio_num_t io_wake;               ///<Wake pin (optional, connects to BNO08x P0)
        uint32_t sclk_speed;              ///<Desired SPI SCLK speed in Hz (max 3MHz)
        bool install_isr_service; ///<Indicates whether the ISR service for the HINT should be installed at IMU initialization, (if gpio_install_isr_service() is called before initialize() set this to false)

        /// @brief Default IMU configuration settings constructor.
        /// To modify default GPIO pins, run "idf.py menuconfig" esp32_BNO08x->GPIO Configuration.
        /// Alternatively, edit the default values in "Kconfig.projbuild"
        bno08x_config_t(bool install_isr_service = true)
            : spi_peripheral((spi_host_device_t) CONFIG_ESP32_BNO08x_SPI_HOST)
            , io_mosi(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_DI))       // default: 23
            , io_miso(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_SDA))      // default: 19
            , io_sclk(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_SCL))      // default: 18
            , io_cs(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_CS))         // default: 33
            , io_int(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_HINT))      // default: 26
            , io_rst(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_RST))       // default: 32
            , io_wake(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_WAKE))     // default: -1 (unused)
            , sclk_speed(static_cast<uint32_t>(CONFIG_ESP32_BNO08X_SCL_SPEED_HZ)) // default: 2MHz
            , install_isr_service(install_isr_service)                            // default: true

        {
        }

        /// @brief Overloaded IMU configuration settings constructor for custom pin settings
        bno08x_config_t(spi_host_device_t spi_peripheral, gpio_num_t io_mosi, gpio_num_t io_miso, gpio_num_t io_sclk, gpio_num_t io_cs,
                gpio_num_t io_int, gpio_num_t io_rst, gpio_num_t io_wake, uint32_t sclk_speed, bool install_isr_service = true)
            : spi_peripheral(spi_peripheral)
            , io_mosi(io_mosi)
            , io_miso(io_miso)
            , io_sclk(io_sclk)
            , io_cs(io_cs)
            , io_int(io_int)
            , io_rst(io_rst)
            , io_wake(io_wake)
            , sclk_speed(sclk_speed)
            , install_isr_service(install_isr_service)
        {
        }
} bno08x_config_t;

class BNO08x
{
    public:
        BNO08x(bno08x_config_t imu_config = bno08x_config_t());
        ~BNO08x();
        bool initialize();

        bool hard_reset();
        bool soft_reset();
        IMUResetReason get_reset_reason();

        bool mode_sleep();
        bool mode_on();
        float q_to_float(int16_t fixed_point_value, uint8_t q_point);

        bool run_full_calibration_routine();
        void calibrate_all();
        void calibrate_accelerometer();
        void calibrate_gyro();
        void calibrate_magnetometer();
        void calibrate_planar_accelerometer();
        void request_calibration_status();
        bool calibration_complete();
        void end_calibration();
        void save_calibration();

        void enable_rotation_vector(uint32_t time_between_reports);
        void enable_game_rotation_vector(uint32_t time_between_reports);
        void enable_ARVR_stabilized_rotation_vector(uint32_t time_between_reports);
        void enable_ARVR_stabilized_game_rotation_vector(uint32_t time_between_reports);
        void enable_gyro_integrated_rotation_vector(uint32_t time_between_reports);
        void enable_accelerometer(uint32_t time_between_reports);
        void enable_linear_accelerometer(uint32_t time_between_reports);
        void enable_gravity(uint32_t time_between_reports);
        void enable_gyro(uint32_t time_between_reports);
        void enable_uncalibrated_gyro(uint32_t time_between_reports);
        void enable_magnetometer(uint32_t time_between_reports);
        void enable_tap_detector(uint32_t time_between_reports);
        void enable_step_counter(uint32_t time_between_reports);
        void enable_stability_classifier(uint32_t time_between_reports);
        void enable_activity_classifier(uint32_t time_between_reports, uint32_t activities_to_enable, uint8_t (&activity_confidence_vals)[9]);
        void enable_raw_accelerometer(uint32_t time_between_reports);
        void enable_raw_gyro(uint32_t time_between_reports);
        void enable_raw_magnetometer(uint32_t time_between_reports);

        void disable_rotation_vector();
        void disable_game_rotation_vector();
        void disable_ARVR_stabilized_rotation_vector();
        void disable_ARVR_stabilized_game_rotation_vector();
        void disable_gyro_integrated_rotation_vector();
        void disable_accelerometer();
        void disable_linear_accelerometer();
        void disable_gravity();
        void disable_gyro();
        void disable_uncalibrated_gyro();
        void disable_magnetometer();
        void disable_tap_detector();
        void disable_step_counter();
        void disable_stability_classifier();
        void disable_activity_classifier();
        void disable_raw_accelerometer();
        void disable_raw_gyro();
        void disable_raw_magnetometer();

        void tare_now(uint8_t axis_sel = TARE_AXIS_ALL, uint8_t rotation_vector_basis = TARE_ROTATION_VECTOR);
        void save_tare();
        void clear_tare();

        bool data_available();
        void register_cb(std::function<void()> cb_fxn);

        uint32_t get_time_stamp();

        void get_magf(float& x, float& y, float& z, uint8_t& accuracy);
        float get_magf_X();
        float get_magf_Y();
        float get_magf_Z();
        uint8_t get_magf_accuracy();

        void get_gravity(float& x, float& y, float& z, uint8_t& accuracy);
        float get_gravity_X();
        float get_gravity_Y();
        float get_gravity_Z();
        uint8_t get_gravity_accuracy();

        float get_roll();
        float get_pitch();
        float get_yaw();

        float get_roll_deg();
        float get_pitch_deg();
        float get_yaw_deg();

        void get_quat(float& i, float& j, float& k, float& real, float& rad_accuracy, uint8_t& accuracy);
        float get_quat_I();
        float get_quat_J();
        float get_quat_K();
        float get_quat_real();
        float get_quat_radian_accuracy();
        uint8_t get_quat_accuracy();

        void get_accel(float& x, float& y, float& z, uint8_t& accuracy);
        float get_accel_X();
        float get_accel_Y();
        float get_accel_Z();
        uint8_t get_accel_accuracy();

        void get_linear_accel(float& x, float& y, float& z, uint8_t& accuracy);
        float get_linear_accel_X();
        float get_linear_accel_Y();
        float get_linear_accel_Z();
        uint8_t get_linear_accel_accuracy();

        int16_t get_raw_accel_X();
        int16_t get_raw_accel_Y();
        int16_t get_raw_accel_Z();

        int16_t get_raw_gyro_X();
        int16_t get_raw_gyro_Y();
        int16_t get_raw_gyro_Z();

        int16_t get_raw_magf_X();
        int16_t get_raw_magf_Y();
        int16_t get_raw_magf_Z();

        void get_gyro_calibrated_velocity(float& x, float& y, float& z, uint8_t& accuracy);
        float get_gyro_calibrated_velocity_X();
        float get_gyro_calibrated_velocity_Y();
        float get_gyro_calibrated_velocity_Z();
        uint8_t get_gyro_accuracy();

        void get_uncalibrated_gyro(float& x, float& y, float& z, float& bx, float& by, float& bz, uint8_t& accuracy);
        float get_uncalibrated_gyro_X();
        float get_uncalibrated_gyro_Y();
        float get_uncalibrated_gyro_Z();
        float get_uncalibrated_gyro_bias_X();
        float get_uncalibrated_gyro_bias_Y();
        float get_uncalibrated_gyro_bias_Z();
        uint8_t get_uncalibrated_gyro_accuracy();

        void get_gyro_velocity(float& x, float& y, float& z);
        float get_gyro_velocity_X();
        float get_gyro_velocity_Y();
        float get_gyro_velocity_Z();

        uint8_t get_tap_detector();
        uint16_t get_step_count();
        int8_t get_stability_classifier();
        uint8_t get_activity_classifier();

        // Metadata functions
        int16_t get_Q1(uint16_t record_ID);
        int16_t get_Q2(uint16_t record_ID);
        int16_t get_Q3(uint16_t record_ID);
        float get_resolution(uint16_t record_ID);
        float get_range(uint16_t record_ID);
        uint32_t FRS_read_word(uint16_t record_ID, uint8_t word_number);
        bool FRS_read_request(uint16_t record_ID, uint16_t read_offset, uint16_t block_size);
        bool FRS_read_data(uint16_t record_ID, uint8_t start_location, uint8_t words_to_read);

        // Record IDs from figure 29, page 29 reference manual
        // These are used to read the metadata for each sensor type
        static const constexpr uint16_t FRS_RECORD_ID_ACCELEROMETER =
                0xE302U; ///< Accelerometer record ID, to be passed in metadata functions like get_Q1()
        static const constexpr uint16_t FRS_RECORD_ID_GYROSCOPE_CALIBRATED =
                0xE306U; ///< Calirated gyroscope record ID, to be passed in metadata functions like get_Q1()
        static const constexpr uint16_t FRS_RECORD_ID_MAGNETIC_FIELD_CALIBRATED =
                0xE309U; ///< Calibrated magnetometer record ID, to be passed in metadata functions like get_Q1()
        static const constexpr uint16_t FRS_RECORD_ID_ROTATION_VECTOR =
                0xE30BU; ///< Rotation vector record ID, to be passed in metadata functions like get_Q1()

        // Activity classifier bits
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_UNKNOWN_EN = (1U << 0U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_IN_VEHICLE_EN = (1U << 1U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_ON_BICYCLE_EN = (1U << 2U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_ON_FOOT_EN = (1U << 3U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_STILL_EN = (1U << 4U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_TILTING_EN = (1U << 5U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_WALKING_EN = (1U << 6U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_RUNNING_EN = (1U << 7U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_ON_STAIRS_EN = (1U << 8U);
        static const constexpr uint16_t ACTIVITY_CLASSIFIER_ALL_EN = 0x1FU;

        static const constexpr uint8_t TARE_AXIS_ALL = 0x07U; ///< Tare all axes (used with tare now command)
        static const constexpr uint8_t TARE_AXIS_Z = 0x04U;   ///< Tar yaw axis only (used with tare now command)

        // Which rotation vector to tare, BNO08x saves them seperately
        static const constexpr uint8_t TARE_ROTATION_VECTOR = 0U;                      ///<Tare rotation vector
        static const constexpr uint8_t TARE_GAME_ROTATION_VECTOR = 1U;                 ///<Tare game rotation vector
        static const constexpr uint8_t TARE_GEOMAGNETIC_ROTATION_VECTOR = 2U;          ///< tare geomagnetic rotation vector
        static const constexpr uint8_t TARE_GYRO_INTEGRATED_ROTATION_VECTOR = 3U;      ///<Tare gyro integrated rotation vector
        static const constexpr uint8_t TARE_ARVR_STABILIZED_ROTATION_VECTOR = 4U;      ///< Tare ARVR stabilized rotation vector
        static const constexpr uint8_t TARE_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 5U; ///<Tare ARVR stabilized game rotation vector

        static const constexpr int16_t ROTATION_VECTOR_Q1 = 14;          ///< Rotation vector Q point (See SH-2 Ref. Manual 6.5.18)
        static const constexpr int16_t ROTATION_VECTOR_ACCURACY_Q1 = 12; ///< Rotation vector accuracy estimate Q point (See SH-2 Ref. Manual 6.5.18)
        static const constexpr int16_t ACCELEROMETER_Q1 = 8;             ///< Acceleration Q point (See SH-2 Ref. Manual 6.5.9)
        static const constexpr int16_t LINEAR_ACCELEROMETER_Q1 = 8;      ///< Linear acceleration Q point (See SH-2 Ref. Manual 6.5.10)
        static const constexpr int16_t GYRO_Q1 = 9;                      ///< Gyro Q point (See SH-2 Ref. Manual 6.5.13)
        static const constexpr int16_t MAGNETOMETER_Q1 = 4;              ///< Magnetometer Q point (See SH-2 Ref. Manual 6.5.16)
        static const constexpr int16_t ANGULAR_VELOCITY_Q1 = 10;         ///< Angular velocity Q point (See SH-2 Ref. Manual 6.5.44)
        static const constexpr int16_t GRAVITY_Q1 = 8;                   ///< Gravity Q point (See SH-2 Ref. Manual 6.5.11)

    private:
        /// @brief Holds data that is received over spi.
        typedef struct bno08x_rx_packet_t
        {
                uint8_t header[4]; ///< Header of SHTP packet.
                uint8_t body[300]; /// Body of SHTP packet.
                uint16_t length;   ///< Packet length in bytes.
        } bno08x_rx_packet_t;

        /// @brief Holds data that is sent over spi.
        typedef struct bno08x_tx_packet_t
        {
                uint8_t body[50]; ///< Body of SHTP the packet (header + body)
                uint16_t length;  ///< Packet length in bytes.
        } bno08x_tx_packet_t;

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

        bool wait_for_rx_done();
        bool wait_for_tx_done();
        bool wait_for_data();
        bool receive_packet();
        void send_packet(bno08x_tx_packet_t* packet);
        void flush_rx_packets(uint8_t flush_count, TickType_t delay);
        void enable_report(uint8_t report_ID, uint32_t time_between_reports, const EventBits_t report_evt_grp_bit, uint32_t special_config = 0);
        void disable_report(uint8_t report_ID, const EventBits_t report_evt_grp_bit);
        void queue_packet(uint8_t channel_number, uint8_t data_length, uint8_t* commands);
        void queue_command(uint8_t command, uint8_t* commands);
        void queue_feature_command(uint8_t report_ID, uint32_t time_between_reports, uint32_t specific_config = 0);
        void queue_calibrate_command(uint8_t _to_calibrate);
        void queue_tare_command(uint8_t command, uint8_t axis = TARE_AXIS_ALL, uint8_t rotation_vector_basis = TARE_ROTATION_VECTOR);
        void queue_request_product_id_command();

        uint16_t parse_packet(bno08x_rx_packet_t* packet);
        uint16_t parse_product_id_report(bno08x_rx_packet_t* packet);
        uint16_t parse_frs_read_response_report(bno08x_rx_packet_t* packet);
        uint16_t parse_input_report(bno08x_rx_packet_t* packet);
        uint16_t parse_command_report(bno08x_rx_packet_t* packet);

        // for debug
        void print_header(bno08x_rx_packet_t* packet);
        void print_packet(bno08x_rx_packet_t* packet);

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

        QueueHandle_t queue_rx_data;       ///<Packet queue used to send data received from bno08x from spi_task to data_proc_task.
        QueueHandle_t queue_tx_data;       ///<Packet queue used to send data to be sent over SPI from sending functions to spi_task.
        QueueHandle_t queue_frs_read_data; ///<Queue used to send packet body from data_proc_task to frs read functions.
        QueueHandle_t queue_reset_reason;  ///<Queue used to send reset reason from product id report to reset_reason() function

        std::vector<std::function<void()>> cb_list; // Vector for storing any call-back functions added with register_cb()

        uint32_t meta_data[9]; ///<First 9 bytes of meta data returned from FRS read operation (we don't really need the rest) (See Ref. Manual 5.1)

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle
        bno08x_init_status_t
                init_status; ///<Initialization status of various functionality, used by deconstructor during cleanup, set during initialization.

        // These are the raw sensor values (without Q applied) pulled from the user requested Input Report
        uint32_t time_stamp; ///<Report timestamp (see datasheet 1.3.5.3)
        uint16_t raw_accel_X, raw_accel_Y, raw_accel_Z,
                accel_accuracy; ///<Raw acceleration readings (See SH-2 Ref. Manual 6.5.8)
        uint16_t raw_lin_accel_X, raw_lin_accel_Y, raw_lin_accel_Z,
                accel_lin_accuracy;                                 ///<Raw linear acceleration (See SH-2 Ref. Manual 6.5.10)
        uint16_t raw_gyro_X, raw_gyro_Y, raw_gyro_Z, gyro_accuracy; ///<Raw gyro reading (See SH-2 Ref. Manual 6.5.13)
        uint16_t raw_quat_I, raw_quat_J, raw_quat_K, raw_quat_real, raw_quat_radian_accuracy,
                quat_accuracy; ///<Raw quaternion reading (See SH-2 Ref. Manual 6.5.44)
        uint16_t raw_velocity_gyro_X, raw_velocity_gyro_Y,
                raw_velocity_gyro_Z; ///<Raw gyro angular velocity reading (See SH-2 Ref. Manual 6.5.44)
        uint16_t gravity_X, gravity_Y, gravity_Z,
                gravity_accuracy; ///<Gravity reading in m/s^2 (See SH-2 Ref. Manual 6.5.11)
        uint16_t raw_uncalib_gyro_X, raw_uncalib_gyro_Y, raw_uncalib_gyro_Z, raw_bias_X, raw_bias_Y, raw_bias_Z,
                uncalib_gyro_accuracy; ///<Uncalibrated gyro reading (See SH-2 Ref. Manual 6.5.14)
        uint16_t raw_magf_X, raw_magf_Y, raw_magf_Z,
                magf_accuracy;         ///<Calibrated magnetic field reading in uTesla (See SH-2 Ref. Manual 6.5.16)
        uint8_t tap_detector;          ///<Tap detector reading (See SH-2 Ref. Manual 6.5.27)
        uint16_t step_count;           ///<Step counter reading (See SH-2 Ref. Manual 6.5.29)
        uint8_t stability_classifier;  ///<Stability status reading (See SH-2 Ref. Manual 6.5.31)
        uint8_t activity_classifier;   ///<Activity status reading (See SH-2 Ref. Manual 6.5.36)
        uint8_t* activity_confidences; ///<Confidence of read activities (See SH-2 Ref. Manual 6.5.36)
        uint8_t calibration_status;    ///<Calibration status of device (See SH-2 Ref. Manual 6.4.7.1 & 6.4.7.2)
        uint16_t mems_raw_accel_X, mems_raw_accel_Y,
                mems_raw_accel_Z; ///<Raw accelerometer readings from MEMS sensor (See SH2 Ref. Manual 6.5.8)
        uint16_t mems_raw_gyro_X, mems_raw_gyro_Y,
                mems_raw_gyro_Z; ///<Raw gyro readings from MEMS sensor (See SH-2 Ref. Manual 6.5.12)
        uint16_t mems_raw_magf_X, mems_raw_magf_Y,
                mems_raw_magf_Z; ///<Raw magnetometer (compass) readings from MEMS sensor (See SH-2 Ref. Manual 6.5.15)

        static void IRAM_ATTR hint_handler(void* arg);

        static const constexpr uint8_t TASK_CNT = 2U; ///<Total amount of tasks utilized by BNO08x driver library

        static const constexpr uint16_t RX_DATA_LENGTH = 300U;    ///<length buffer containing data received over spi
        static const constexpr uint16_t MAX_METADATA_LENGTH = 9U; ///<max length of metadata used in frs read operations

        static const constexpr TickType_t HOST_INT_TIMEOUT_MS =
                300UL /
                portTICK_PERIOD_MS; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in miliseconds)

        static const constexpr TickType_t HARD_RESET_DELAY_MS =
                200UL /
                portTICK_PERIOD_MS; ///<How long RST pin is held low during hard reset (min 10ns according to datasheet, but should be longer for stable operation)

        static const constexpr TickType_t CMD_EXECUTION_DELAY_MS = 10UL / portTICK_PERIOD_MS; ///<How long to delay after queueing command to allow it to execute (for ex. after sending command to enable report).

        static const constexpr TickType_t FLUSH_PKT_DELAY_MS = 20UL / portTICK_PERIOD_MS; ///<How long to delay between wait_for_rx_done() calls when flush_rx_packets() is called.

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
        static const constexpr EventBits_t EVT_GRP_RPT_ROTATION_VECTOR_BIT = (1 << 0);      ///< When set, rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT = (1 << 1); ///< When set, game rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT =
                (1U << 2U); ///< When set, ARVR stabilized rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT =
                (1U << 3U); ///< When set, ARVR stabilized game rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT =
                (1U << 4U); ///< When set, gyro integrator rotation vector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ACCELEROMETER_BIT = (1U << 5U);         ///< When set, accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT = (1U << 6U);  ///< When set, linear accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GRAVITY_BIT = (1U << 7U);               ///< When set, gravity reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_BIT = (1U << 8U);                  ///< When set, gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT = (1U << 9U);     ///< When set, uncalibrated gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_MAGNETOMETER_BIT = (1U << 10U);         ///< When set, magnetometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_TAP_DETECTOR_BIT = (1U << 11U);         ///< When set, tap detector reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STEP_COUNTER_BIT = (1U << 12U);         ///< When set, step counter reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT = (1U << 13U); ///< When set, stability classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT = (1U << 14U);  ///< When set, activity classifier reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_ACCELEROMETER_BIT = (1U << 15U);    ///< When set, raw accelerometer reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_GYRO_BIT = (1U << 16U);             ///< When set, raw gyro reports are active.
        static const constexpr EventBits_t EVT_GRP_RPT_RAW_MAGNETOMETER_BIT = (1U << 17U);     ///< When set, raw magnetometer reports are active.

        // evt_grp_task_flow bits
        static const constexpr EventBits_t EVT_GRP_TSK_FLW_RUNNING_BIT =
                (1U << 0U); ///< When set, data_proc_task and spi_task are active, when 0 they are pending deletion or deleted.

        static const constexpr EventBits_t EVT_GRP_RPT_ALL_BITS =
                EVT_GRP_RPT_ROTATION_VECTOR_BIT | EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT | EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT |
                EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT | EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT | EVT_GRP_RPT_ACCELEROMETER_BIT |
                EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT | EVT_GRP_RPT_GRAVITY_BIT | EVT_GRP_RPT_GYRO_BIT | EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT |
                EVT_GRP_RPT_MAGNETOMETER_BIT | EVT_GRP_RPT_TAP_DETECTOR_BIT | EVT_GRP_RPT_STEP_COUNTER_BIT | EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT |
                EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT | EVT_GRP_RPT_RAW_ACCELEROMETER_BIT | EVT_GRP_RPT_RAW_GYRO_BIT | EVT_GRP_RPT_RAW_MAGNETOMETER_BIT;

        // Higher level calibration commands, used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_ACCEL = 0U;        ///<Calibrate accelerometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_GYRO = 1U;         ///<Calibrate gyro command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_MAG = 2U;          ///<Calibrate magnetometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_PLANAR_ACCEL = 3U; ///<Calibrate planar acceleration command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_ACCEL_GYRO_MAG =
                4U;                                         ///<Calibrate accelerometer, gyro, & magnetometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_STOP = 5U; ///<Stop calibration command used by queue_calibrate_command

        // Command IDs (see Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_ERRORS = 1U;
        static const constexpr uint8_t COMMAND_COUNTER = 2U;
        static const constexpr uint8_t COMMAND_TARE = 3U;            ///<Command and response to tare command (See Sh2 Ref. Manual 6.4.4)
        static const constexpr uint8_t COMMAND_INITIALIZE = 4U;      ///<Reinitialize sensor hub components See (SH2 Ref. Manual 6.4.5)
        static const constexpr uint8_t COMMAND_DCD = 6U;             ///<Save DCD command (See SH2 Ref. Manual 6.4.7)
        static const constexpr uint8_t COMMAND_ME_CALIBRATE = 7U;    ///<Command and response to configure ME calibration (See SH2 Ref. Manual 6.4.7)
        static const constexpr uint8_t COMMAND_DCD_PERIOD_SAVE = 9U; ///<Configure DCD periodic saving (See SH2 Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_OSCILLATOR = 10U;     ///<Retrieve oscillator type command (See SH2 Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_CLEAR_DCD = 11U;      ///<Clear DCD & Reset command (See SH2 Ref. Manual 6.4)

        // SHTP channel 2 control report IDs, used in communication with sensor (See Ref. Manual 6.2)
        static const constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1U;    ///< See SH2 Ref. Manual 6.3.9
        static const constexpr uint8_t SHTP_REPORT_COMMAND_REQUEST = 0xF2U;     ///< See SH2 Ref. Manual 6.3.8
        static const constexpr uint8_t SHTP_REPORT_FRS_READ_RESPONSE = 0xF3U;   ///< See SH2 Ref. Manual 6.3.7
        static const constexpr uint8_t SHTP_REPORT_FRS_READ_REQUEST = 0xF4U;    ///< See SH2 Ref. Manual 6.3.6
        static const constexpr uint8_t SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8U; ///< See SH2 Ref. Manual 6.3.2
        static const constexpr uint8_t SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9U;  ///< See SH2 Ref. Manual 6.3.1
        static const constexpr uint8_t SHTP_REPORT_BASE_TIMESTAMP = 0xFBU;      ///< See SH2 Ref. Manual 7.2.1
        static const constexpr uint8_t SHTP_REPORT_SET_FEATURE_COMMAND = 0xFDU; ///< See SH2 Ref. Manual 6.5.4

        // Sensor report IDs, used when enabling and reading BNO08x reports
        static const constexpr uint8_t SENSOR_REPORT_ID_ACCELEROMETER = 0x01U;                        ///< See SH2 Ref. Manual 6.5.9
        static const constexpr uint8_t SENSOR_REPORT_ID_GYROSCOPE = 0x02U;                            ///< See SH2 Ref. Manual 6.5.13
        static const constexpr uint8_t SENSOR_REPORT_ID_MAGNETIC_FIELD = 0x03U;                       ///< See SH2 Ref. Manual 6.5.16
        static const constexpr uint8_t SENSOR_REPORT_ID_LINEAR_ACCELERATION = 0x04U;                  ///< See SH2 Ref. Manual 6.5.10
        static const constexpr uint8_t SENSOR_REPORT_ID_ROTATION_VECTOR = 0x05U;                      ///< See SH2 Ref. Manual 6.5.18
        static const constexpr uint8_t SENSOR_REPORT_ID_GRAVITY = 0x06U;                              ///< See SH2 Ref. Manual 6.5.11
        static const constexpr uint8_t SENSOR_REPORT_ID_UNCALIBRATED_GYRO = 0x07U;                    ///< See SH2 Ref. Manual 6.5.14
        static const constexpr uint8_t SENSOR_REPORT_ID_GAME_ROTATION_VECTOR = 0x08U;                 ///< See SH2 Ref. Manual 6.5.19
        static const constexpr uint8_t SENSOR_REPORT_ID_GEOMAGNETIC_ROTATION_VECTOR = 0x09U;          ///< See SH2 Ref. Manual 6.5.20
        static const constexpr uint8_t SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2AU;      ///< See SH2 Ref. Manual 6.5.44
        static const constexpr uint8_t SENSOR_REPORT_ID_TAP_DETECTOR = 0x10U;                         ///< See SH2 Ref. Manual 6.5.27
        static const constexpr uint8_t SENSOR_REPORT_ID_STEP_COUNTER = 0x11U;                         ///< See SH2 Ref. Manual 6.5.29
        static const constexpr uint8_t SENSOR_REPORT_ID_STABILITY_CLASSIFIER = 0x13U;                 ///< See SH2 Ref. Manual 6.5.31
        static const constexpr uint8_t SENSOR_REPORT_ID_RAW_ACCELEROMETER = 0x14U;                    ///< See SH2 Ref. Manual 6.5.8
        static const constexpr uint8_t SENSOR_REPORT_ID_RAW_GYROSCOPE = 0x15U;                        ///< See SH2 Ref. Manual 6.5.12
        static const constexpr uint8_t SENSOR_REPORT_ID_RAW_MAGNETOMETER = 0x16U;                     ///< See SH2 Ref. Manual 6.5.15
        static const constexpr uint8_t SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1EU;         ///< See SH2 Ref. Manual 6.5.36
        static const constexpr uint8_t SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR = 0x28U;      ///< See SH2 Ref. Manual 6.5.42
        static const constexpr uint8_t SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29U; ///< See SH2 Ref. Manual 6.5.43

        // Tare commands used by queue_tare_command
        static const constexpr uint8_t TARE_NOW = 0U;               ///< See SH2 Ref. Manual 6.4.4.1
        static const constexpr uint8_t TARE_PERSIST = 1U;           ///< See SH2 Ref. Manual 6.4.4.2
        static const constexpr uint8_t TARE_SET_REORIENTATION = 2U; ///< See SH2 Ref. Manual 6.4.4.3

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements

        friend class BNO08xTestHelper; //allow test helper to access private members for unit tests
};