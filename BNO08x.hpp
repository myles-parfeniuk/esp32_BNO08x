#pragma once
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <cstring>

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
        uint64_t sclk_speed;              ///<Desired SPI SCLK speed in Hz (max 3MHz)
        bool debug_en;                    ///<Whether or not debugging print statements are enabled

#ifdef ESP32C3_IMU_CONFIG
        /// @brief Default IMU configuration settings constructor for ESP32-C3, add
        /// add_compile_definitions("ESP32C3_IMU_CONFIG") to CMakeList to use
        bno08x_config_t()
            : spi_peripheral(SPI2_HOST)
            , io_mosi(GPIO_NUM_4)
            , io_miso(GPIO_NUM_19)
            , io_sclk(GPIO_NUM_18)
            , io_cs(GPIO_NUM_5)
            , io_int(GPIO_NUM_6)
            , io_rst(GPIO_NUM_7)
            , io_wake(GPIO_NUM_NC)
            , sclk_speed(2000000UL) // 2MHz SCLK speed
            , debug_en(false)
        {
        }
#else
        /// @brief Default IMU configuration settings constructor for ESP32
        bno08x_config_t()
            : spi_peripheral(SPI3_HOST)
            , io_mosi(GPIO_NUM_23)
            , io_miso(GPIO_NUM_19)
            , io_sclk(GPIO_NUM_18)
            , io_cs(GPIO_NUM_33)
            , io_int(GPIO_NUM_26)
            , io_rst(GPIO_NUM_32)
            , io_wake(GPIO_NUM_NC)
            , sclk_speed(2000000UL) // 2MHz SCLK speed
            // , sclk_speed(10000U), //clock slowed to see on AD2
            , debug_en(false)

        {
        }
#endif
        /// @brief Overloaded IMU configuration settings constructor for custom pin settings
        bno08x_config_t(spi_host_device_t spi_peripheral, gpio_num_t io_mosi, gpio_num_t io_miso, gpio_num_t io_sclk, gpio_num_t io_cs,
                gpio_num_t io_int, gpio_num_t io_rst, gpio_num_t io_wake, uint64_t sclk_speed, bool debug)
            : spi_peripheral(spi_peripheral)
            , io_mosi(io_mosi)
            , io_miso(io_miso)
            , io_sclk(io_sclk)
            , io_cs(io_cs)
            , io_int(io_int)
            , io_rst(io_rst)
            , io_wake(io_wake)
            , sclk_speed(sclk_speed)
            , debug_en(false)

        {
        }
} bno08x_config_t;

class BNO08x
{
    public:
        BNO08x(bno08x_config_t imu_config = default_imu_config);
        bool initialize();

        bool hard_reset();
        bool soft_reset();
        uint8_t get_reset_reason();

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

        void enable_rotation_vector(uint16_t time_between_reports);
        void enable_game_rotation_vector(uint16_t time_between_reports);
        void enable_ARVR_stabilized_rotation_vector(uint16_t time_between_reports);
        void enable_ARVR_stabilized_game_rotation_vector(uint16_t time_between_reports);
        void enable_gyro_integrated_rotation_vector(uint16_t timeBetweenReports);
        void enable_accelerometer(uint16_t time_between_reports);
        void enable_linear_accelerometer(uint16_t time_between_reports);
        void enable_gravity(uint16_t time_between_reports);
        void enable_gyro(uint16_t time_between_reports);
        void enable_uncalibrated_gyro(uint16_t time_between_reports);
        void enable_magnetometer(uint16_t time_between_reports);
        void enable_tap_detector(uint16_t time_between_reports);
        void enable_step_counter(uint16_t time_between_reports);
        void enable_stability_classifier(uint16_t time_between_reports);
        void enable_activity_classifier(uint16_t time_between_reports, uint32_t activities_to_enable, uint8_t (&activity_confidence_vals)[9]);
        void enable_raw_accelerometer(uint16_t time_between_reports);
        void enable_raw_gyro(uint16_t time_between_reports);
        void enable_raw_magnetometer(uint16_t time_between_reports);

        void tare_now(uint8_t axis_sel = TARE_AXIS_ALL, uint8_t rotation_vector_basis = TARE_ROTATION_VECTOR);
        void save_tare();
        void clear_tare();

        bool data_available();
        uint16_t parse_input_report();
        uint16_t parse_command_report();
        uint16_t get_readings();

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

        void print_header();
        void print_packet();

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
        static const constexpr uint16_t FRS_RECORDID_ACCELEROMETER = 0xE302;
        static const constexpr uint16_t FRS_RECORDID_GYROSCOPE_CALIBRATED = 0xE306;
        static const constexpr uint16_t FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED = 0xE309;
        static const constexpr uint16_t FRS_RECORDID_ROTATION_VECTOR = 0xE30B;

        static const constexpr uint8_t TARE_AXIS_ALL = 0x07; ///< Tare all axes (used with tare now command)
        static const constexpr uint8_t TARE_AXIS_Z = 0x04;   ///< Tar yaw axis only (used with tare now command)

        // Which rotation vector to tare, BNO08x saves them seperately
        static const constexpr uint8_t TARE_ROTATION_VECTOR = 0;                       ///<Tare rotation vector
        static const constexpr uint8_t TARE_GAME_ROTATION_VECTOR = 1;                  ///<Tare game rotation vector
        static const constexpr uint8_t TARE_GEOMAGNETIC_ROTATION_VECTOR = 2;           ///< tare geomagnetic rotation vector
        static const constexpr uint8_t TARE_GYRO_INTEGRATED_ROTATION_VECTOR = 3;       ///<Tare gyro integrated rotation vector
        static const constexpr uint8_t TARE_AR_VR_STABILIZED_ROTATION_VECTOR = 4;      ///< Tare ARVR stabilized rotation vector
        static const constexpr uint8_t TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR = 5; ///<Tare ARVR stabilized game rotation vector

        static const constexpr int16_t ROTATION_VECTOR_Q1 = 14;          ///< Rotation vector Q point (See SH-2 Ref. Manual 6.5.18)
        static const constexpr int16_t ROTATION_VECTOR_ACCURACY_Q1 = 12; ///< Rotation vector accuracy estimate Q point (See SH-2 Ref. Manual 6.5.18)
        static const constexpr int16_t ACCELEROMETER_Q1 = 8;             ///< Acceleration Q point (See SH-2 Ref. Manual 6.5.9)
        static const constexpr int16_t LINEAR_ACCELEROMETER_Q1 = 8;      ///< Linear acceleration Q point (See SH-2 Ref. Manual 6.5.10)
        static const constexpr int16_t GYRO_Q1 = 9;                      ///< Gyro Q point (See SH-2 Ref. Manual 6.5.13)
        static const constexpr int16_t MAGNETOMETER_Q1 = 4;              ///< Magnetometer Q point (See SH-2 Ref. Manual 6.5.16)
        static const constexpr int16_t ANGULAR_VELOCITY_Q1 = 10;         ///< Angular velocity Q point (See SH-2 Ref. Manual 6.5.44)
        static const constexpr int16_t GRAVITY_Q1 = 8;                   ///< Gravity Q point (See SH-2 Ref. Manual 6.5.11)

    private:
        bool wait_for_device_int();
        bool receive_packet();
        void send_packet();
        void queue_packet(uint8_t channel_number, uint8_t data_length);
        void queue_command(uint8_t command);
        void queue_feature_command(uint8_t report_ID, uint16_t time_between_reports);
        void queue_feature_command(uint8_t report_ID, uint16_t time_between_reports, uint32_t specific_config);
        void queue_calibrate_command(uint8_t _to_calibrate);
        void queue_tare_command(uint8_t command, uint8_t axis = TARE_AXIS_ALL, uint8_t rotation_vector_basis = TARE_ROTATION_VECTOR);
        void queue_request_product_id_command();

        static bno08x_config_t default_imu_config; ///< default imu config settings

        volatile uint8_t
                tx_packet_queued; ///<Whether or not a packet is currently waiting to be sent, a queued packet is sent on assertion of BNO08x HINT pin)
        SemaphoreHandle_t tx_semaphore; ///<Mutex semaphore used to prevent sending or receiving of packets if packet is currently being queued
        uint8_t rx_buffer[300];         ///<buffer used to receive packet with receive_packet()
        uint8_t tx_buffer[50];          ///<buffer used for sending packet with send_packet()
        uint8_t packet_header_rx[4];    ///<SHTP header received with receive_packet()
        uint8_t commands[20];           ///<Command to be sent with send_packet()
        uint8_t sequence_number[6];     ///<Sequence num of each com channel, 6 in total
        uint32_t meta_data[9]; ///<First 9 bytes of meta data returned from FRS read operation (we don't really need the rest) (See Ref. Manual 5.1)
        uint8_t command_sequence_number = 0; ///<Sequence num of command, sent within command packet.
        uint16_t packet_length_tx = 0;       ///<Packet length to be sent with send_packet()
        uint16_t packet_length_rx = 0;       ///<Packet length received (calculated from packet_header_rx)

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle

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

        // spi task
        TaskHandle_t spi_task_hdl; ///<SPI task handle
        static void spi_task_trampoline(void* arg);
        void spi_task();

        volatile bool int_asserted; ///<Interrupt asserted flag, sets true after hint_handler ISR launches SPI task and it has run to completion
        static void IRAM_ATTR hint_handler(void* arg);
        static bool
                isr_service_installed; ///<true of the isr service has been installed, only has to be done once regardless of how many devices are used

        static const constexpr uint64_t HOST_INT_TIMEOUT_US =
                150000ULL; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in microseconds)

        // Higher level calibration commands, used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_ACCEL = 0;        ///<Calibrate accelerometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_GYRO = 1;         ///<Calibrate gyro command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_MAG = 2;          ///<Calibrate magnetometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_PLANAR_ACCEL = 3; ///<Calibrate planar acceleration command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_ACCEL_GYRO_MAG =
                4;                                         ///<Calibrate accelerometer, gyro, & magnetometer command used by queue_calibrate_command
        static const constexpr uint8_t CALIBRATE_STOP = 5; ///<Stop calibration command used by queue_calibrate_command

        // Command IDs (see Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_ERRORS = 1;
        static const constexpr uint8_t COMMAND_COUNTER = 2;
        static const constexpr uint8_t COMMAND_TARE = 3;            ///<Command and response to tare command (See Sh2 Ref. Manual 6.4.4)
        static const constexpr uint8_t COMMAND_INITIALIZE = 4;      ///<Reinitialize sensor hub components See (SH2 Ref. Manual 6.4.5)
        static const constexpr uint8_t COMMAND_DCD = 6;             ///<Save DCD command (See SH2 Ref. Manual 6.4.7)
        static const constexpr uint8_t COMMAND_ME_CALIBRATE = 7;    ///<Command and response to configure ME calibration (See SH2 Ref. Manual 6.4.7)
        static const constexpr uint8_t COMMAND_DCD_PERIOD_SAVE = 9; ///<Configure DCD periodic saving (See SH2 Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_OSCILLATOR = 10;     ///<Retrieve oscillator type command (See SH2 Ref. Manual 6.4)
        static const constexpr uint8_t COMMAND_CLEAR_DCD = 11;      ///<Clear DCD & Reset command (See SH2 Ref. Manual 6.4)

        // SHTP channel 2 control report IDs, used in communication with sensor (See Ref. Manual 6.2)
        static const constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1;    ///< See SH2 Ref. Manual 6.3.9
        static const constexpr uint8_t SHTP_REPORT_COMMAND_REQUEST = 0xF2;     ///< See SH2 Ref. Manual 6.3.8
        static const constexpr uint8_t SHTP_REPORT_FRS_READ_RESPONSE = 0xF3;   ///< See SH2 Ref. Manual 6.3.7
        static const constexpr uint8_t SHTP_REPORT_FRS_READ_REQUEST = 0xF4;    ///< See SH2 Ref. Manual 6.3.6
        static const constexpr uint8_t SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8; ///< See SH2 Ref. Manual 6.3.2
        static const constexpr uint8_t SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9;  ///< See SH2 Ref. Manual 6.3.1
        static const constexpr uint8_t SHTP_REPORT_BASE_TIMESTAMP = 0xFB;      ///< See SH2 Ref. Manual 7.2.1
        static const constexpr uint8_t SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD; ///< See SH2 Ref. Manual 6.5.4

        // Sensor report IDs, used when enabling and reading BNO08x reports
        static const constexpr uint8_t SENSOR_REPORTID_ACCELEROMETER = 0x01;                         ///< See SH2 Ref. Manual 6.5.9
        static const constexpr uint8_t SENSOR_REPORTID_GYROSCOPE = 0x02;                             ///< See SH2 Ref. Manual 6.5.13
        static const constexpr uint8_t SENSOR_REPORTID_MAGNETIC_FIELD = 0x03;                        ///< See SH2 Ref. Manual 6.5.16
        static const constexpr uint8_t SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04;                   ///< See SH2 Ref. Manual 6.5.10
        static const constexpr uint8_t SENSOR_REPORTID_ROTATION_VECTOR = 0x05;                       ///< See SH2 Ref. Manual 6.5.18
        static const constexpr uint8_t SENSOR_REPORTID_GRAVITY = 0x06;                               ///< See SH2 Ref. Manual 6.5.11
        static const constexpr uint8_t SENSOR_REPORTID_UNCALIBRATED_GYRO = 0x07;                     ///< See SH2 Ref. Manual 6.5.14
        static const constexpr uint8_t SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08;                  ///< See SH2 Ref. Manual 6.5.19
        static const constexpr uint8_t SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = 0x09;           ///< See SH2 Ref. Manual 6.5.20
        static const constexpr uint8_t SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A;       ///< See SH2 Ref. Manual 6.5.44
        static const constexpr uint8_t SENSOR_REPORTID_TAP_DETECTOR = 0x10;                          ///< See SH2 Ref. Manual 6.5.27
        static const constexpr uint8_t SENSOR_REPORTID_STEP_COUNTER = 0x11;                          ///< See SH2 Ref. Manual 6.5.29
        static const constexpr uint8_t SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x13;                  ///< See SH2 Ref. Manual 6.5.31
        static const constexpr uint8_t SENSOR_REPORTID_RAW_ACCELEROMETER = 0x14;                     ///< See SH2 Ref. Manual 6.5.8
        static const constexpr uint8_t SENSOR_REPORTID_RAW_GYROSCOPE = 0x15;                         ///< See SH2 Ref. Manual 6.5.12
        static const constexpr uint8_t SENSOR_REPORTID_RAW_MAGNETOMETER = 0x16;                      ///< See SH2 Ref. Manual 6.5.15
        static const constexpr uint8_t SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E;          ///< See SH2 Ref. Manual 6.5.36
        static const constexpr uint8_t SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR = 0x28;      ///< See SH2 Ref. Manual 6.5.42
        static const constexpr uint8_t SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR = 0x29; ///< See SH2 Ref. Manual 6.5.43

        // Tare commands used by queue_tare_command
        static const constexpr uint8_t TARE_NOW = 0;               ///< See SH2 Ref. Manual 6.4.4.1
        static const constexpr uint8_t TARE_PERSIST = 1;           ///< See SH2 Ref. Manual 6.4.4.2
        static const constexpr uint8_t TARE_SET_REORIENTATION = 2; ///< See SH2 Ref. Manual 6.4.4.3

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements
};