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

enum class BNO08xCalSel : uint8_t
{
    accelerometer = SH2_CAL_ACCEL,
    gyro = SH2_CAL_GYRO,
    magnetometer = SH2_CAL_MAG,
    planar_accelerometer = SH2_CAL_PLANAR,
    all = (SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG | SH2_CAL_PLANAR)
};

/// @brief Reason for previous IMU reset (returned by get_reset_reason())
enum class BNO08xResetReason : uint8_t
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
enum class BNO08xAccuracy : uint8_t
{
    UNRELIABLE,
    LOW,
    MED,
    HIGH,
    UNDEFINED
};
using IMUAccuracy = BNO08xAccuracy; // legacy version compatibility

const constexpr char* BNO08xAccuracy_to_str(BNO08xAccuracy accuracy)
{
    switch (accuracy)
    {
        case BNO08xAccuracy::UNRELIABLE:
            return "UNRELIABLE";
        case BNO08xAccuracy::LOW:
            return "LOW";
        case BNO08xAccuracy::MED:
            return "MED";
        case BNO08xAccuracy::HIGH:
            return "HIGH";
        case BNO08xAccuracy::UNDEFINED:
            return "UNDEFINED";
        default:
            return "UNDEFINED";
    }
}

/// @brief BNO08xActivity Classifier enable bits passed to enable_activity_classifier()
/// See ref manual 6.5.36.1
enum class BNO08xActivityEnable : uint32_t
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
    ALL = (UNKNOWN | IN_VEHICLE | ON_BICYCLE | ON_FOOT | STILL | TILTING | WALKING | RUNNING | ON_STAIRS)
};

/// @brief BNO08xActivity states returned from BNO08x::activity_classifier.get()
enum class BNO08xActivity : uint8_t
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

/**
 * @brief Converts a BNO08xActivity enum to string.
 *
 * @return The resulting string conversion of the enum.
 */
const constexpr char* BNO08xActivity_to_str(BNO08xActivity activity)
{
    switch (activity)
    {
        case BNO08xActivity::UNKNOWN:
            return "UNKNOWN";
        case BNO08xActivity::IN_VEHICLE:
            return "IN_VEHICLE";
        case BNO08xActivity::ON_BICYCLE:
            return "ON_BICYCLE";
        case BNO08xActivity::ON_FOOT:
            return "ON_FOOT";
        case BNO08xActivity::STILL:
            return "STILL";
        case BNO08xActivity::TILTING:
            return "TILTING";
        case BNO08xActivity::WALKING:
            return "WALKING";
        case BNO08xActivity::RUNNING:
            return "RUNNING";
        case BNO08xActivity::ON_STAIRS:
            return "ON_STAIRS";
        case BNO08xActivity::UNDEFINED:
            return "UNDEFINED";
        default:
            return "UNDEFINED";
    }
}

/// @brief BNO08xStability states returned from BNO08x::stability_classifier.get()
enum class BNO08xStability : uint8_t
{
    UNKNOWN = 0,    // 0 = unknown
    ON_TABLE = 1,   // 1 = on table
    STATIONARY = 2, // 2 = stationary
    STABLE = 3,     // 3 = stable
    MOTION = 4,     // 4 = in motion
    RESERVED = 5,   // 5 = reserved (not used)
    UNDEFINED = 6   // used for unit tests
};

/**
 * @brief Converts a BNO08xStability enum to string.
 *
 * @return The resulting string conversion of the enum.
 */
const constexpr char* BNO08xStability_to_str(BNO08xStability stability)
{
    switch (stability)
    {
        case BNO08xStability::UNKNOWN:
            return "UNKNOWN";
        case BNO08xStability::ON_TABLE:
            return "ON_TABLE";
        case BNO08xStability::STATIONARY:
            return "STATIONARY";
        case BNO08xStability::STABLE:
            return "STABLE";
        case BNO08xStability::MOTION:
            return "MOTION";
        case BNO08xStability::RESERVED:
            return "RESERVED";
        case BNO08xStability::UNDEFINED:
            return "UNDEFINED";
        default:
            return "UNDEFINED";
    }
}

enum class BNO08xFrsID : uint16_t
{
    STATIC_CALIBRATION_AGM = 0x7979,
    NOMINAL_CALIBRATION = 0x4D4D,
    STATIC_CALIBRATION_SRA = 0x8A8A,
    NOMINAL_CALIBRATION_SRA = 0x4E4E,
    DYNAMIC_CALIBRATION = 0x1F1F,
    ME_POWER_MGMT = 0xD3E2,
    SYSTEM_ORIENTATION = 0x2D3E,
    ACCEL_ORIENTATION = 0x2D41,
    SCREEN_ACCEL_ORIENTATION = 0x2D43,
    GYROSCOPE_ORIENTATION = 0x2D46,
    MAGNETOMETER_ORIENTATION = 0x2D4C,
    ARVR_STABILIZATION_RV = 0x3E2D,
    ARVR_STABILIZATION_GRV = 0x3E2E,
    TAP_DETECT_CONFIG = 0xC269,
    SIG_MOTION_DETECT_CONFIG = 0xC274,
    SHAKE_DETECT_CONFIG = 0x7D7D,
    MAX_FUSION_PERIOD = 0xD7D7,
    SERIAL_NUMBER = 0x4B4B,
    ES_PRESSURE_CAL = 0x39AF,
    ES_TEMPERATURE_CAL = 0x4D20,
    ES_HUMIDITY_CAL = 0x1AC9,
    ES_AMBIENT_LIGHT_CAL = 0x39B1,
    ES_PROXIMITY_CAL = 0x4DA2,
    ALS_CAL = 0xD401,
    PROXIMITY_SENSOR_CAL = 0xD402,
    PICKUP_DETECTOR_CONFIG = 0x1B2A,
    FLIP_DETECTOR_CONFIG = 0xFC94,
    STABILITY_DETECTOR_CONFIG = 0xED85,
    ACTIVITY_TRACKER_CONFIG = 0xED88,
    SLEEP_DETECTOR_CONFIG = 0xED87,
    TILT_DETECTOR_CONFIG = 0xED89,
    POCKET_DETECTOR_CONFIG = 0xEF27,
    CIRCLE_DETECTOR_CONFIG = 0xEE51,
    USER_RECORD = 0x74B4,
    ME_TIME_SOURCE_SELECT = 0xD403,
    UART_FORMAT = 0xA1A1,
    GYRO_INTEGRATED_RV_CONFIG = 0xA1A2,
    META_RAW_ACCELEROMETER = 0xE301,
    META_ACCELEROMETER = 0xE302,
    META_LINEAR_ACCELERATION = 0xE303,
    META_GRAVITY = 0xE304,
    META_RAW_GYROSCOPE = 0xE305,
    META_GYROSCOPE_CALIBRATED = 0xE306,
    META_GYROSCOPE_UNCALIBRATED = 0xE307,
    META_RAW_MAGNETOMETER = 0xE308,
    META_MAGNETIC_FIELD_CALIBRATED = 0xE309,
    META_MAGNETIC_FIELD_UNCALIBRATED = 0xE30A,
    META_ROTATION_VECTOR = 0xE30B,
    META_GAME_ROTATION_VECTOR = 0xE30C,
    META_GEOMAGNETIC_ROTATION_VECTOR = 0xE30D,
    META_PRESSURE = 0xE30E,
    META_AMBIENT_LIGHT = 0xE30F,
    META_HUMIDITY = 0xE310,
    META_PROXIMITY = 0xE311,
    META_TEMPERATURE = 0xE312,
    META_TAP_DETECTOR = 0xE313,
    META_STEP_DETECTOR = 0xE314,
    META_STEP_COUNTER = 0xE315,
    META_SIGNIFICANT_MOTION = 0xE316,
    META_STABILITY_CLASSIFIER = 0xE317,
    META_SHAKE_DETECTOR = 0xE318,
    META_FLIP_DETECTOR = 0xE319,
    META_PICKUP_DETECTOR = 0xE31A,
    META_STABILITY_DETECTOR = 0xE31B,
    META_PERSONAL_ACTIVITY_CLASSIFIER = 0xE31C,
    META_SLEEP_DETECTOR = 0xE31D,
    META_TILT_DETECTOR = 0xE31E,
    META_POCKET_DETECTOR = 0xE31F,
    META_CIRCLE_DETECTOR = 0xE320,
    META_HEART_RATE_MONITOR = 0xE321,
    META_ARVR_STABILIZED_RV = 0xE322,
    META_ARVR_STABILIZED_GRV = 0xE323,
    META_GYRO_INTEGRATED_RV = 0xE324
};

const constexpr char* BNO08xFrsID_to_str(BNO08xFrsID id)
{
    switch (id)
    {
        case BNO08xFrsID::STATIC_CALIBRATION_AGM:
            return "STATIC_CALIBRATION_AGM";
        case BNO08xFrsID::NOMINAL_CALIBRATION:
            return "NOMINAL_CALIBRATION";
        case BNO08xFrsID::STATIC_CALIBRATION_SRA:
            return "STATIC_CALIBRATION_SRA";
        case BNO08xFrsID::NOMINAL_CALIBRATION_SRA:
            return "NOMINAL_CALIBRATION_SRA";
        case BNO08xFrsID::DYNAMIC_CALIBRATION:
            return "DYNAMIC_CALIBRATION";
        case BNO08xFrsID::ME_POWER_MGMT:
            return "ME_POWER_MGMT";
        case BNO08xFrsID::SYSTEM_ORIENTATION:
            return "SYSTEM_ORIENTATION";
        case BNO08xFrsID::ACCEL_ORIENTATION:
            return "ACCEL_ORIENTATION";
        case BNO08xFrsID::SCREEN_ACCEL_ORIENTATION:
            return "SCREEN_ACCEL_ORIENTATION";
        case BNO08xFrsID::GYROSCOPE_ORIENTATION:
            return "GYROSCOPE_ORIENTATION";
        case BNO08xFrsID::MAGNETOMETER_ORIENTATION:
            return "MAGNETOMETER_ORIENTATION";
        case BNO08xFrsID::ARVR_STABILIZATION_RV:
            return "ARVR_STABILIZATION_RV";
        case BNO08xFrsID::ARVR_STABILIZATION_GRV:
            return "ARVR_STABILIZATION_GRV";
        case BNO08xFrsID::TAP_DETECT_CONFIG:
            return "TAP_DETECT_CONFIG";
        case BNO08xFrsID::SIG_MOTION_DETECT_CONFIG:
            return "SIG_MOTION_DETECT_CONFIG";
        case BNO08xFrsID::SHAKE_DETECT_CONFIG:
            return "SHAKE_DETECT_CONFIG";
        case BNO08xFrsID::MAX_FUSION_PERIOD:
            return "MAX_FUSION_PERIOD";
        case BNO08xFrsID::SERIAL_NUMBER:
            return "SERIAL_NUMBER";
        case BNO08xFrsID::ES_PRESSURE_CAL:
            return "ES_PRESSURE_CAL";
        case BNO08xFrsID::ES_TEMPERATURE_CAL:
            return "ES_TEMPERATURE_CAL";
        case BNO08xFrsID::ES_HUMIDITY_CAL:
            return "ES_HUMIDITY_CAL";
        case BNO08xFrsID::ES_AMBIENT_LIGHT_CAL:
            return "ES_AMBIENT_LIGHT_CAL";
        case BNO08xFrsID::ES_PROXIMITY_CAL:
            return "ES_PROXIMITY_CAL";
        case BNO08xFrsID::ALS_CAL:
            return "ALS_CAL";
        case BNO08xFrsID::PROXIMITY_SENSOR_CAL:
            return "PROXIMITY_SENSOR_CAL";
        case BNO08xFrsID::PICKUP_DETECTOR_CONFIG:
            return "PICKUP_DETECTOR_CONFIG";
        case BNO08xFrsID::FLIP_DETECTOR_CONFIG:
            return "FLIP_DETECTOR_CONFIG";
        case BNO08xFrsID::STABILITY_DETECTOR_CONFIG:
            return "STABILITY_DETECTOR_CONFIG";
        case BNO08xFrsID::ACTIVITY_TRACKER_CONFIG:
            return "ACTIVITY_TRACKER_CONFIG";
        case BNO08xFrsID::SLEEP_DETECTOR_CONFIG:
            return "SLEEP_DETECTOR_CONFIG";
        case BNO08xFrsID::TILT_DETECTOR_CONFIG:
            return "TILT_DETECTOR_CONFIG";
        case BNO08xFrsID::POCKET_DETECTOR_CONFIG:
            return "POCKET_DETECTOR_CONFIG";
        case BNO08xFrsID::CIRCLE_DETECTOR_CONFIG:
            return "CIRCLE_DETECTOR_CONFIG";
        case BNO08xFrsID::USER_RECORD:
            return "USER_RECORD";
        case BNO08xFrsID::ME_TIME_SOURCE_SELECT:
            return "ME_TIME_SOURCE_SELECT";
        case BNO08xFrsID::UART_FORMAT:
            return "UART_FORMAT";
        case BNO08xFrsID::GYRO_INTEGRATED_RV_CONFIG:
            return "GYRO_INTEGRATED_RV_CONFIG";
        case BNO08xFrsID::META_RAW_ACCELEROMETER:
            return "META_RAW_ACCELEROMETER";
        case BNO08xFrsID::META_ACCELEROMETER:
            return "META_ACCELEROMETER";
        case BNO08xFrsID::META_LINEAR_ACCELERATION:
            return "META_LINEAR_ACCELERATION";
        case BNO08xFrsID::META_GRAVITY:
            return "META_GRAVITY";
        case BNO08xFrsID::META_RAW_GYROSCOPE:
            return "META_RAW_GYROSCOPE";
        case BNO08xFrsID::META_GYROSCOPE_CALIBRATED:
            return "META_GYROSCOPE_CALIBRATED";
        case BNO08xFrsID::META_GYROSCOPE_UNCALIBRATED:
            return "META_GYROSCOPE_UNCALIBRATED";
        case BNO08xFrsID::META_RAW_MAGNETOMETER:
            return "META_RAW_MAGNETOMETER";
        case BNO08xFrsID::META_MAGNETIC_FIELD_CALIBRATED:
            return "META_MAGNETIC_FIELD_CALIBRATED";
        case BNO08xFrsID::META_MAGNETIC_FIELD_UNCALIBRATED:
            return "META_MAGNETIC_FIELD_UNCALIBRATED";
        case BNO08xFrsID::META_ROTATION_VECTOR:
            return "META_ROTATION_VECTOR";
        case BNO08xFrsID::META_GAME_ROTATION_VECTOR:
            return "META_GAME_ROTATION_VECTOR";
        case BNO08xFrsID::META_GEOMAGNETIC_ROTATION_VECTOR:
            return "META_GEOMAGNETIC_ROTATION_VECTOR";
        case BNO08xFrsID::META_PRESSURE:
            return "META_PRESSURE";
        case BNO08xFrsID::META_AMBIENT_LIGHT:
            return "META_AMBIENT_LIGHT";
        case BNO08xFrsID::META_HUMIDITY:
            return "META_HUMIDITY";
        case BNO08xFrsID::META_PROXIMITY:
            return "META_PROXIMITY";
        case BNO08xFrsID::META_TEMPERATURE:
            return "META_TEMPERATURE";
        case BNO08xFrsID::META_TAP_DETECTOR:
            return "META_TAP_DETECTOR";
        case BNO08xFrsID::META_STEP_DETECTOR:
            return "META_STEP_DETECTOR";
        case BNO08xFrsID::META_STEP_COUNTER:
            return "META_STEP_COUNTER";
        case BNO08xFrsID::META_SIGNIFICANT_MOTION:
            return "META_SIGNIFICANT_MOTION";
        case BNO08xFrsID::META_STABILITY_CLASSIFIER:
            return "META_STABILITY_CLASSIFIER";
        case BNO08xFrsID::META_SHAKE_DETECTOR:
            return "META_SHAKE_DETECTOR";
        case BNO08xFrsID::META_FLIP_DETECTOR:
            return "META_FLIP_DETECTOR";
        case BNO08xFrsID::META_PICKUP_DETECTOR:
            return "META_PICKUP_DETECTOR";
        case BNO08xFrsID::META_STABILITY_DETECTOR:
            return "META_STABILITY_DETECTOR";
        case BNO08xFrsID::META_PERSONAL_ACTIVITY_CLASSIFIER:
            return "META_PERSONAL_ACTIVITY_CLASSIFIER";
        case BNO08xFrsID::META_SLEEP_DETECTOR:
            return "META_SLEEP_DETECTOR";
        case BNO08xFrsID::META_TILT_DETECTOR:
            return "META_TILT_DETECTOR";
        case BNO08xFrsID::META_POCKET_DETECTOR:
            return "META_POCKET_DETECTOR";
        case BNO08xFrsID::META_CIRCLE_DETECTOR:
            return "META_CIRCLE_DETECTOR";
        case BNO08xFrsID::META_HEART_RATE_MONITOR:
            return "META_HEART_RATE_MONITOR";
        case BNO08xFrsID::META_ARVR_STABILIZED_RV:
            return "META_ARVR_STABILIZED_RV";
        case BNO08xFrsID::META_ARVR_STABILIZED_GRV:
            return "META_ARVR_STABILIZED_GRV";
        case BNO08xFrsID::META_GYRO_INTEGRATED_RV:
            return "META_GYRO_INTEGRATED_RV";
        default:
            return "UNKNOWN";
    }
}

/// @brief Struct to represent unit quaternion.
typedef struct bno08x_quat_t
{
        float real;
        float i;
        float j;
        float k;
        float rad_accuracy;
        BNO08xAccuracy accuracy;

        bno08x_quat_t()
            : real(0.0f)
            , i(0.0f)
            , j(0.0f)
            , k(0.0f)
            , rad_accuracy(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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
        float rad_accuracy;
        BNO08xAccuracy accuracy;

        bno08x_euler_angle_t()
            : x(0.0f)
            , y(0.0f)
            , z(0.0f)
            , rad_accuracy(0.0f)
            , accuracy(BNO08xAccuracy::UNDEFINED)
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
        uint8_t confidence[10];
        BNO08xActivity mostLikelyState;
        BNO08xAccuracy accuracy;
        uint8_t page;
        bool lastPage;

        bno08x_activity_classifier_t()
            : confidence({})
            , mostLikelyState(BNO08xActivity::UNDEFINED)
            , accuracy(BNO08xAccuracy::UNDEFINED)
            , page(0U)
            , lastPage(false)
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
        uint32_t timestamp_us;
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t temperature;
        BNO08xAccuracy accuracy;

        bno08x_raw_gyro_t()
            : timestamp_us(0UL)
            , x(0U)
            , y(0U)
            , z(0U)
            , temperature(0U)
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
        uint32_t timestamp_us;
        int16_t x;
        int16_t y;
        int16_t z;
        BNO08xAccuracy accuracy;

        bno08x_raw_accel_t()
            : timestamp_us(0UL)
            , x(0U)
            , y(0U)
            , z(0U)
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
        uint32_t timestamp_us;
        int16_t x;
        int16_t y;
        int16_t z;
        BNO08xAccuracy accuracy;

        bno08x_raw_magf_t()
            : timestamp_us(0UL)
            , x(0U)
            , y(0U)
            , z(0U)
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
        char vendor_ID[48];           ///< Vendor name and part number
        uint8_t sensor_specific[48];  ///< See SH-2 Reference Manual
        uint32_t vendor_id_len;       ///< [bytes]
        uint32_t sensor_specific_len; ///< [bytes]
        uint32_t range;               ///< Same units as sensor reports
        uint32_t resolution;          ///< Same units as sensor reports
        uint32_t min_period_us;       ///< [uS] min period to use with enable_report
        uint32_t max_period_us;       ///< [uS] max period to use with enable_report
        uint32_t fifo_reserved;       ///< (Unused)
        uint32_t fifo_max;            ///< (Unused)
        uint32_t batch_buffer_bytes;  ///< (Unused)
        uint16_t revision;            ///< Metadata record format revision
        uint16_t power_mA;            ///< [mA] Fixed point 16Q10 format
        uint16_t q_point_1;           ///< q point for sensor values
        uint16_t q_point_2;           ///< q point for accuracy or bias fields
        uint16_t q_point_3;           ///< q point for sensor data change sensitivity
        uint8_t me_version;           ///< Motion Engine Version
        uint8_t mh_version;           ///< Motion Hub Version
        uint8_t sh_version;           ///< SensorHub Version
        

        // Default constructor
        bno08x_meta_data_t()
            : vendor_ID({})
            , sensor_specific({})
            , vendor_id_len(0)
            , sensor_specific_len(0)
            , range(0)
            , resolution(0)
            , min_period_us(0)
            , max_period_us(0)
            , fifo_reserved(0)
            , fifo_max(0)
            , batch_buffer_bytes(0)
            , revision(0)
            , power_mA(0)
            , q_point_1(0)
            , q_point_2(0)
            , q_point_3(0)
            , me_version(0)
            , mh_version(0)
            , sh_version(0)
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
