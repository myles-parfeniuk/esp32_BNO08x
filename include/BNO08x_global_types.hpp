/**
 * @file BNO08x_global_types.hpp
 * @author Myles Parfeniuk
 */
#pragma once

#include <math.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "sh2_SensorValue.h"

// macros for bno08x_tap_detector_t
#define TAP_DETECTED_X_AXIS(tap) ((tap) & (1U << 0U) ? 1 : 0)
#define TAP_DETECTED_X_AXIS_POSITIVE(tap) ((tap) & (1U << 1U) ? 1 : 0)
#define TAP_DETECTED_Y_AXIS(tap) ((tap) & (1U << 2U) ? 1 : 0)
#define TAP_DETECTED_Y_AXIS_POSITIVE(tap) ((tap) & (1U << 3U) ? 1 : 0)
#define TAP_DETECTED_Z_AXIS(tap) ((tap) & (1U << 4U) ? 1 : 0)
#define TAP_DETECTED_Z_AXIS_POSITIVE(tap) ((tap) & (1U << 5U) ? 1 : 0)
#define TAP_DETECTED_DOUBLE(tap) ((tap) & (1U << 6U) ? 1 : 0)

// macros for bno08x_shake_detector_t
#define SHAKE_DETECTED_X(tap) ((tap) & (1U << 0U) ? 1 : 0)
#define SHAKE_DETECTED_Y(tap) ((tap) & (1U << 1U) ? 1 : 0)
#define SHAKE_DETECTED_Z(tap) ((tap) & (1U << 2U) ? 1 : 0)

/// @brief BNO08xActivity Classifier enable bits passed to enable_activity_classifier()
enum class BNO08xActivityEnable
{
    UNKNOWN = (1U << 0U),
    IN_VEHICLE = (1U << 1U),
    ON_BICYCLE = (1U << 2U),
    ON_FOOT = (1U << 3U),
    STILL = (1U << 4U),
    TILTING = (1U << 5U),
    WALKING = (1U << 6U),
    RUNNING = (1U << 7U),
    ON_STAIRS = (1U << 8U),
    ALL = 0x1FU
};

/// @brief BNO08xActivity states returned from BNO08x::activity_classifier.get()
enum class BNO08xActivity
{
    UNKNOWN = 0,    // 0 = unknown
    IN_VEHICLE = 1, // 1 = in vehicle
    ON_BICYCLE = 2, // 2 = on bicycle
    ON_FOOT = 3,    // 3 = on foot
    STILL = 4,      // 4 = still
    TILTING = 5,    // 5 = tilting
    WALKING = 6,    // 6 = walking
    RUNNING = 7,    // 7 = running
    ON_STAIRS = 8,  // 8 = on stairs
    UNDEFINED = 9   // used for unit tests
};

/// @brief BNO08xStability states returned from BNO08x::stability_classifier.get()
enum class BNO08xStability
{
    UNKNOWN = 0,    // 0 = unknown
    ON_TABLE = 1,   // 1 = on table
    STATIONARY = 2, // 2 = stationary
    STABLE = 3,     // 3 = stable
    MOTION = 4,     // 4 = in motion
    RESERVED = 5,   // 5 = reserved (not used)
    UNDEFINED = 6   // used for unit tests

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

/// @brief Struct to represent unit quaternion.
typedef struct bno08x_quat_t
{
        float real;
        float i;
        float j;
        float k;
        float accuracy;

        bno08x_quat_t()
            : real(0.0f)
            , i(0.0f)
            , j(0.0f)
            , k(0.0f)
            , accuracy(0.0f)
        {
        }

        // overloaded assignment operator to handle RV with accuracy
        bno08x_quat_t& operator=(const sh2_RotationVectorWAcc_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->accuracy = source.accuracy;
            return *this;
        }

        // overloaded assignment operator to handle RV with w/o accuracy
        bno08x_quat_t& operator=(const sh2_RotationVector_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->accuracy = 0.0f;
            return *this;
        }

        // overloaded assignment operator to handle IRV report

        // overloaded assignment operator to handle RV with w/o accuracy
        bno08x_quat_t& operator=(const sh2_GyroIntegratedRV_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->accuracy = 0.0f;
            return *this;
        }

} bno08x_quat_t;

/// @brief Struct to represent euler angle (units in degrees or rads)
typedef struct bno08x_euler_angle_t
{
        float x;
        float y;
        float z;
        float accuracy;

        bno08x_euler_angle_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , accuracy(0.0f)
        {
        }

        // overloaded = operator for quat to euler conversion
        bno08x_euler_angle_t& operator=(const bno08x_quat_t& source)
        {
            this->x = atan2(2.0f * (source.real * source.i + source.j * source.k), 1.0f - 2.0f * (source.i * source.i + source.j * source.j));
            this->y = asin(2.0f * (source.real * source.j - source.k * source.i));
            this->z = atan2(2.0f * (source.real * source.k + source.i * source.j), 1.0f - 2.0f * (source.j * source.j + source.k * source.k));
            this->accuracy = source.accuracy;
            return *this;
        }

        // overloaded *= operator for rad2deg conversions
        template <typename T>
        bno08x_euler_angle_t& operator*=(T value)
        {
            x *= static_cast<float>(value);
            y *= static_cast<float>(value);
            z *= static_cast<float>(value);
            accuracy *= static_cast<float>(value);
            return *this;
        }

} bno08x_euler_angle_t;

/// @brief Struct to represent angular velocity (units in rad/s)
typedef struct bno08x_ang_vel_t
{
        float x;
        float y;
        float z;

        bno08x_ang_vel_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }

        // overloaded *= operator for rad2deg conversions
        template <typename T>
        bno08x_ang_vel_t& operator*=(T value)
        {
            x *= static_cast<float>(value);
            y *= static_cast<float>(value);
            z *= static_cast<float>(value);
            return *this;
        }

        // strip sh2_GyroIntegratedRV_t of velocity data for IRV reports
        bno08x_ang_vel_t& operator=(const sh2_GyroIntegratedRV_t& source)
        {
            this->x = source.angVelX;
            this->y = source.angVelY;
            this->z = source.angVelZ;
            return *this;
        }
} bno08x_ang_vel_t;

/// @brief Struct to represent magnetic field data (units in uTesla)
typedef struct bno08x_magf_t
{
        float x;
        float y;
        float z;

        bno08x_magf_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }

        // overloaded = operator for sh2_MagneticField_t conversion
        bno08x_magf_t& operator=(const sh2_MagneticField_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            return *this;
        }

        // overloaded = operator for sh2_MagneticFieldUncalibrated_t conversion
        bno08x_magf_t& operator=(const sh2_MagneticFieldUncalibrated_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            return *this;
        }

} bno08x_magf_t;

/// @brief Struct to represent magnetic field bias data (units in uTesla)
typedef struct bno08x_magf_bias_t
{
        float x;
        float y;
        float z;

        bno08x_magf_bias_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }

        // overloaded = operator for sh2_MagneticFieldUncalibrated_t conversion
        bno08x_magf_bias_t& operator=(const sh2_MagneticFieldUncalibrated_t& source)
        {
            this->x = source.biasX;
            this->y = source.biasY;
            this->z = source.biasZ;
            return *this;
        }

} bno08x_magf_bias_t;

/// @brief Struct to represent gyro data (units in rad/s)
typedef struct bno08x_gyro_t
{
        float x;
        float y;
        float z;

        bno08x_gyro_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }

        // overloaded = operator for sh2_Gyroscope_t conversion
        bno08x_gyro_t& operator=(const sh2_Gyroscope_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            return *this;
        }

        // overloaded = operator for sh2_GyroscopeUncalibrated conversion
        bno08x_gyro_t& operator=(const sh2_GyroscopeUncalibrated& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            return *this;
        }

} bno08x_gyro_t;

/// @brief Struct to represent gyro bias data (units in rad/s)
typedef struct bno08x_gyro_bias_t
{
        float x;
        float y;
        float z;

        bno08x_gyro_bias_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
        {
        }

        // overloaded = operator for sh2_GyroscopeUncalibrated conversion
        bno08x_gyro_bias_t& operator=(const sh2_GyroscopeUncalibrated& source)
        {
            this->x = source.biasX;
            this->y = source.biasY;
            this->z = source.biasZ;
            return *this;
        }

} bno08x_gyro_bias_t;

/// @brief Struct to represent activity classifier data.
typedef struct bno08x_activity_classifier_t
{
        uint8_t page;
        bool lastPage;
        BNO08xActivity mostLikelyState;
        uint8_t confidence[10];

        bno08x_activity_classifier_t()
            : page(0U)
            , lastPage(false)
            , mostLikelyState(BNO08xActivity::UNDEFINED)
            , confidence({})
        {
        }

        // conversion from sh2_PersonalActivityClassifier_t
        bno08x_activity_classifier_t& operator=(const sh2_PersonalActivityClassifier_t& source)
        {
            this->page = source.page;
            this->lastPage = source.lastPage;
            this->mostLikelyState = static_cast<BNO08xActivity>(source.mostLikelyState);

            for (int i = 0; i < 10; ++i)
                this->confidence[i] = source.confidence[i];

            return *this;
        }
} bno08x_activity_classifier_t;

/// @brief Struct to represent tap detector data (flag meaning: 0 = no tap, 1 = positive tap on axis, -1 = negative tap on axis)
typedef struct bno08x_tap_detector_t
{
        int8_t x_flag;
        int8_t y_flag;
        int8_t z_flag;
        bool double_tap;

        bno08x_tap_detector_t()
            : x_flag(0)
            , y_flag(0)
            , z_flag(0)
            , double_tap(false)
        {
        }

        // overloaded = operator for sh2_GyroscopeUncalibrated conversion
        bno08x_tap_detector_t& operator=(const sh2_TapDetector_t& source)
        {
            if (TAP_DETECTED_X_AXIS(source.flags))
                this->x_flag = -1;
            else
                this->x_flag = 0;

            if (TAP_DETECTED_X_AXIS_POSITIVE(source.flags))
                this->x_flag = 1;

            if (TAP_DETECTED_Y_AXIS(source.flags))
                this->y_flag = -1;
            else
                this->y_flag = 0;

            if (TAP_DETECTED_Y_AXIS_POSITIVE(source.flags))
                this->y_flag = 1;

            if (TAP_DETECTED_Z_AXIS(source.flags))
                this->z_flag = -1;
            else
                this->z_flag = 0;

            if (TAP_DETECTED_Z_AXIS_POSITIVE(source.flags))
                this->z_flag = 1;

            if (TAP_DETECTED_DOUBLE(source.flags))
                this->double_tap = true;
            else
                this->double_tap = false;

            return *this;
        }

} bno08x_tap_detector_t;

/// @brief Struct to represent shake detector data (flag meaning: 0 = no shake 1 = shake detected)
typedef struct bno08x_shake_detector_t
{
        uint8_t x_flag;
        uint8_t y_flag;
        uint8_t z_flag;

        bno08x_shake_detector_t()
            : x_flag(0U)
            , y_flag(0U)
            , z_flag(0U)
        {
        }

        // overloaded = operator for sh2_GyroscopeUncalibrated conversion
        bno08x_shake_detector_t& operator=(const sh2_ShakeDetector_t& source)
        {
            if (SHAKE_DETECTED_X(source.shake))
                this->x_flag = 1U;
            else
                this->x_flag = 0U;

            if (SHAKE_DETECTED_Y(source.shake))
                this->y_flag = 1U;
            else
                this->y_flag = 0U;

            if (SHAKE_DETECTED_Z(source.shake))
                this->z_flag = 1U;
            else
                this->z_flag = 0U;

            return *this;
        }

} bno08x_shake_detector_t;

typedef sh2_Accelerometer_t bno08x_accel_t; ///< Acceleration data.
typedef sh2_StepCounter bno08x_step_counter_t;
typedef sh2_RawGyroscope_t bno08x_raw_gyro_t;
typedef sh2_RawAccelerometer bno08x_raw_accel_t;
typedef sh2_RawMagnetometer_t bno08x_raw_magf_t;

typedef bno08x_config_t imu_config_t; // legacy version compatibility