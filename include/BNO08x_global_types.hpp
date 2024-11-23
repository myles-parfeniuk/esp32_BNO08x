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

/// @brief BNO08xActivity states returned from get_activity_classifier()
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

/// @brief BNO08xStability states returned from get_stability_classifier()
enum class BNO08xStability
{
    UNKNOWN = 0,    // 0 = unknown
    ON_TABLE = 1,   // 1 = on table
    STATIONARY = 2, // 2 = stationary
    UNDEFINED = 3   // used for unit tests
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

typedef struct bno08x_activity_classifier_data_t
{
        uint8_t page;
        bool lastPage;
        BNO08xActivity mostLikelyState;
        uint8_t confidence[10];

        bno08x_activity_classifier_data_t()
            : page(0U)
            , lastPage(false)
            , mostLikelyState(BNO08xActivity::UNDEFINED)
            , confidence({})
        {
        }

        // conversion from sh2_PersonalActivityClassifier_t
        bno08x_activity_classifier_data_t& operator=(const sh2_PersonalActivityClassifier_t& source)
        {
            this->page = source.page;
            this->lastPage = source.lastPage;
            this->mostLikelyState = static_cast<BNO08xActivity>(source.mostLikelyState);

            for (int i = 0; i < 10; ++i)
                this->confidence[i] = source.confidence[i];

            return *this;
        }
} bno08x_activity_classifier_data_t;

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

        // overloaded assignment operators to handle both sh2 structs:

        bno08x_quat_t& operator=(const sh2_RotationVectorWAcc_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->accuracy = source.accuracy;
            return *this;
        }

        bno08x_quat_t& operator=(const sh2_RotationVector_t& source)
        {
            this->real = source.real;
            this->i = source.i;
            this->j = source.j;
            this->k = source.k;
            this->accuracy = 0.0f;
            return *this;
        }

} bno08x_quat_t;

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

typedef sh2_Accelerometer_t bno08x_accel_data_t; ///< Acceleration data.
typedef sh2_MagneticField_t bno08x_magf_data_t;  ///< Magnetic field data.
typedef sh2_StepCounter bno08x_step_counter_data_t;
typedef sh2_Gyroscope_t bno08x_gyro_data_t;
typedef sh2_RawGyroscope_t bno08x_raw_gyro_data_t;
typedef sh2_RawAccelerometer bno08x_raw_accel_data_t;
typedef sh2_RawMagnetometer_t bno08x_raw_magf_data_t;

typedef bno08x_config_t imu_config_t; // legacy version compatibility