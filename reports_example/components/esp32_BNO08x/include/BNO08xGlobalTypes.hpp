/**
 * @file BNO08xGlobalTypes.hpp
 * @author Myles Parfeniuk
 */

#pragma once

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

// standard library includes
#include <math.h>
#include <inttypes.h>
#include <stdint.h>
#include <cstring>

// esp-idf includes
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

// third-party includes
#include "sh2_SensorValue.h"

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
            , sclk_speed(static_cast<uint32_t>(CONFIG_ESP32_BNO08X_SCL_SPEED_HZ)) // default: 2MHz
            , install_isr_service(install_isr_service)                            // default: true

        {
        }

        /// @brief Overloaded IMU configuration settings constructor for custom pin settings
        bno08x_config_t(spi_host_device_t spi_peripheral, gpio_num_t io_mosi, gpio_num_t io_miso, gpio_num_t io_sclk,
                gpio_num_t io_cs, gpio_num_t io_int, gpio_num_t io_rst, uint32_t sclk_speed, bool install_isr_service = true)
            : spi_peripheral(spi_peripheral)
            , io_mosi(io_mosi)
            , io_miso(io_miso)
            , io_sclk(io_sclk)
            , io_cs(io_cs)
            , io_int(io_int)
            , io_rst(io_rst)
            , sclk_speed(sclk_speed)
            , install_isr_service(install_isr_service)
        {
        }
} bno08x_config_t;
typedef bno08x_config_t imu_config_t; // legacy version compatibility

enum class BNO08xCalSel
{
    accelerometer = SH2_CAL_ACCEL,
    gyro = SH2_CAL_GYRO,
    magnetometer = SH2_CAL_MAG,
    planar_accelerometer = SH2_CAL_PLANAR,
    all = (SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG | SH2_CAL_PLANAR)
};

/// @brief Reason for previous IMU reset (returned by get_reset_reason())
enum class BNO08xResetReason
{
    UNDEFINED, ///< Undefined reset reason, this should never occur and is an error.
    POR,       ///< Previous reset was due to power on reset.
    INT_RST,   ///< Previous reset was due to internal reset.
    WTD,       ///< Previous reset was due to watchdog timer.
    EXT_RST,   ///< Previous reset was due to external reset.
    OTHER      ///< Previous reset was due to power other reason.
};

/// @brief Sensor accuracy returned from input reports, corresponds to status bits (see ref.
/// manual 6.5.1)
enum class BNO08xAccuracy
{
    UNRELIABLE,
    LOW,
    MED,
    HIGH,
    UNDEFINED
};
using IMUAccuracy = BNO08xAccuracy; // legacy version compatibility

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

/// @brief Struct to represent unit quaternion.
typedef struct bno08x_quat_t
{
        float real;
        float i;
        float j;
        float k;
        BNO08xAccuracy accuracy;
        float rad_accuracy;

        bno08x_quat_t()
            : real(0.0f)
            , i(0.0f)
            , j(0.0f)
            , k(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
            , rad_accuracy(0.0f)
        {
        }

        // overloaded assignment operator to handle RV with rad accuracy
        bno08x_quat_t& operator=(const sh2_RotationVectorWAcc_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->rad_accuracy = source.accuracy;
            return *this;
        }

        // overloaded assignment operator to handle RV with w/o rad accuracy
        bno08x_quat_t& operator=(const sh2_RotationVector_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->rad_accuracy = 0.0f;
            return *this;
        }

        // overloaded assignment operator to handle IRV report
        bno08x_quat_t& operator=(const sh2_GyroIntegratedRV_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->rad_accuracy = 0.0f;
            return *this;
        }

} bno08x_quat_t;

/// @brief Struct to represent euler angle (units in degrees or rads)
typedef struct bno08x_euler_angle_t
{
        float x;
        float y;
        float z;
        BNO08xAccuracy accuracy;
        float rad_accuracy;

        bno08x_euler_angle_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
            , rad_accuracy(0.0f)
        {
        }

        // overloaded = operator for quat to euler conversion
        bno08x_euler_angle_t& operator=(const bno08x_quat_t& source)
        {
            this->x = atan2(2.0f * (source.real * source.i + source.j * source.k),
                    1.0f - 2.0f * (source.i * source.i + source.j * source.j));
            this->y = asin(2.0f * (source.real * source.j - source.k * source.i));
            this->z = atan2(2.0f * (source.real * source.k + source.i * source.j),
                    1.0f - 2.0f * (source.j * source.j + source.k * source.k));
            this->rad_accuracy = source.rad_accuracy;
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
            rad_accuracy *= static_cast<float>(value);
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
        BNO08xAccuracy accuracy;

        bno08x_magf_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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
        BNO08xAccuracy accuracy;

        bno08x_gyro_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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
        BNO08xAccuracy accuracy;

        bno08x_activity_classifier_t()
            : page(0U)
            , lastPage(false)
            , mostLikelyState(BNO08xActivity::UNDEFINED)
            , confidence({})
            , accuracy(BNO08xAccuracy::UNDEFINED)
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

/// @brief Struct to represent tap detector data (flag meaning: 0 = no tap, 1 = positive tap on
/// axis, -1 = negative tap on axis)
typedef struct bno08x_tap_detector_t
{
        int8_t x_flag;
        int8_t y_flag;
        int8_t z_flag;
        bool double_tap;
        BNO08xAccuracy accuracy;

        bno08x_tap_detector_t()
            : x_flag(0)
            , y_flag(0)
            , z_flag(0)
            , double_tap(false)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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
        BNO08xAccuracy accuracy;

        bno08x_shake_detector_t()
            : x_flag(0U)
            , y_flag(0U)
            , z_flag(0U)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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

/// @brief Struct to represent acceleration data from acceleration, linear acceleration, and gravity
/// reports.
typedef struct bno08x_accel_t
{
        float x;
        float y;
        float z;
        BNO08xAccuracy accuracy;

        bno08x_accel_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_Accelerometer_t
        bno08x_accel_t& operator=(const sh2_Accelerometer_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            return *this;
        }
} bno08x_accel_t;

/// @brief Struct to represent step counter data from step counter reports.
typedef struct bno08x_step_counter_t
{
        uint32_t latency;
        uint16_t steps;
        BNO08xAccuracy accuracy;

        bno08x_step_counter_t()
            : latency(0UL)
            , steps(0U)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_StepCounter_t
        bno08x_step_counter_t& operator=(const sh2_StepCounter_t& source)
        {
            this->latency = source.latency;
            this->steps = source.steps;
            return *this;
        }
} bno08x_step_counter_t;

/// @brief Struct to represent raw mems gyro data from raw gyro reports (units in ADC counts).
typedef struct bno08x_raw_gyro_t
{
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t temperature;
        uint32_t timestamp_us;
        BNO08xAccuracy accuracy;

        bno08x_raw_gyro_t()
            : x(0U)
            , y(0U)
            , z(0U)
            , temperature(0U)
            , timestamp_us(0UL)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_RawGyroscope_t
        bno08x_raw_gyro_t& operator=(const sh2_RawGyroscope_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            this->temperature = source.temperature;
            this->timestamp_us = source.timestamp;
            return *this;
        }
} bno08x_raw_gyro_t;

/// @brief Struct to represent raw mems accelerometer data from raw accelerometer reports (units in
/// ADC counts).
typedef struct bno08x_raw_accel_t
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint32_t timestamp_us;
        BNO08xAccuracy accuracy;

        bno08x_raw_accel_t()
            : x(0U)
            , y(0U)
            , z(0U)
            , timestamp_us(0UL)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_RawAccelerometer_t
        bno08x_raw_accel_t& operator=(const sh2_RawAccelerometer_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            this->timestamp_us = source.timestamp;
            return *this;
        }
} bno08x_raw_accel_t;

/// @brief Struct to represent raw mems magnetometer data from raw magnetometer reports (units in
/// ADC counts).
typedef struct bno08x_raw_magf_t
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint32_t timestamp_us;
        BNO08xAccuracy accuracy;

        bno08x_raw_magf_t()
            : x(0U)
            , y(0U)
            , z(0U)
            , timestamp_us(0UL)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_RawMagnetometer_t
        bno08x_raw_magf_t& operator=(const sh2_RawMagnetometer_t& source)
        {
            this->x = source.x;
            this->y = source.y;
            this->z = source.z;
            this->timestamp_us = source.timestamp;
            return *this;
        }
} bno08x_raw_magf_t;

/// @brief Struct to represent stability classifier data from stability classifier reports.
typedef struct bno08x_stability_classifier_t
{
        BNO08xStability stability;
        BNO08xAccuracy accuracy;

        bno08x_stability_classifier_t()
            : stability(BNO08xStability::UNDEFINED)
            , accuracy(BNO08xAccuracy::UNDEFINED)
        {
        }

        // conversion from sh2_StabilityClassifier_t
        bno08x_stability_classifier_t& operator=(const sh2_StabilityClassifier_t& source)
        {
            this->stability = static_cast<BNO08xStability>(source.classification);
            return *this;
        }

} bno08x_stability_classifier_t;

/// @brief Struct to represent sample counts, returned from BNO08xRpt::get_sample_counts()
typedef struct bno08x_sample_counts_t
{
        uint32_t offered;  ///< Number of samples produced by underlying data source.
        uint32_t on;       ///< Number of "offered" samples while this sensor was requested by host.
        uint32_t accepted; ///< Number of "on" samples that passed decimation filter.
        uint32_t
                attempted; ///< Number of "accepted" samples that passed threshold requirements and had transmission to the host attempted.

        bno08x_sample_counts_t()
            : offered(0UL)
            , on(0UL)
            , accepted(0UL)
            , attempted(0UL)
        {
        }

        // conversion from sh2_PersonalActivityClassifier_t
        bno08x_sample_counts_t& operator=(const sh2_Counts_t& source)
        {
            this->offered = source.offered;
            this->on = source.on;
            this->accepted = source.accepted;
            this->attempted = source.attempted;

            return *this;
        }
} bno08x_sample_counts_t;

/// @brief Struct to represent sensor/report meta data, returned from BNO08xRpt::get_meta_data()
typedef struct bno08x_meta_data_t
{
        uint8_t me_version;           ///< Motion Engine Version
        uint8_t mh_version;           ///< Motion Hub Version
        uint8_t sh_version;           ///< SensorHub Version
        uint32_t range;               ///< Same units as sensor reports
        uint32_t resolution;          ///< Same units as sensor reports
        uint16_t revision;            ///< Metadata record format revision
        uint16_t power_mA;            ///< [mA] Fixed point 16Q10 format
        uint32_t min_period_us;       ///< [uS] min period to use with enable_report
        uint32_t max_period_us;       ///< [uS] max period to use with enable_report
        uint32_t fifo_reserved;       ///< (Unused)
        uint32_t fifo_max;            ///< (Unused)
        uint32_t batch_buffer_bytes;  ///< (Unused)
        uint16_t q_point_1;           ///< q point for sensor values
        uint16_t q_point_2;           ///< q point for accuracy or bias fields
        uint16_t q_point_3;           ///< q point for sensor data change sensitivity
        uint32_t vendor_id_len;       ///< [bytes]
        char vendor_ID[48];           ///< Vendor name and part number
        uint32_t sensor_specific_len; ///< [bytes]
        uint8_t sensor_specific[48];  ///< See SH-2 Reference Manual

        // Default constructor
        bno08x_meta_data_t()
            : me_version(0)
            , mh_version(0)
            , sh_version(0)
            , range(0)
            , resolution(0)
            , revision(0)
            , power_mA(0)
            , min_period_us(0)
            , max_period_us(0)
            , fifo_reserved(0)
            , fifo_max(0)
            , batch_buffer_bytes(0)
            , q_point_1(0)
            , q_point_2(0)
            , q_point_3(0)
            , vendor_id_len(0)
            , sensor_specific_len(0)
        {
            memset(vendor_ID, 0, sizeof(vendor_ID));
            memset(sensor_specific, 0, sizeof(sensor_specific));
        }

        // Conversion constructor from sh2_SensorMetadata_t
        bno08x_meta_data_t(const sh2_SensorMetadata_t& src)
        {
            me_version = src.meVersion;
            mh_version = src.mhVersion;
            sh_version = src.shVersion;
            range = src.range;
            resolution = src.resolution;
            revision = src.revision;
            power_mA = src.power_mA;
            min_period_us = src.minPeriod_uS;
            max_period_us = src.maxPeriod_uS;
            fifo_reserved = src.fifoReserved;
            fifo_max = src.fifoMax;
            batch_buffer_bytes = src.batchBufferBytes;
            q_point_1 = src.qPoint1;
            q_point_2 = src.qPoint2;
            q_point_3 = src.qPoint3;
            vendor_id_len = src.vendorIdLen;
            sensor_specific_len = src.sensorSpecificLen;
            memcpy(vendor_ID, src.vendorId, vendor_id_len);
            memcpy(sensor_specific, src.sensorSpecific, sensor_specific_len);
        }
} bno08x_meta_data_t;

static const constexpr uint8_t TOTAL_RPT_COUNT = 38; ///< Amount of possible reports returned from BNO08x.
