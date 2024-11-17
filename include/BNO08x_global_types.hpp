/**
 * @file BNO08x_global_types.hpp
 * @author Myles Parfeniuk
 */
#pragma once

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

/// @brief Sensor accuracy returned during sensor calibration
enum class BNO08xAccuracy
{
    LOW = 1,
    MED,
    HIGH,
    UNDEFINED
};
using IMUAccuracy = BNO08xAccuracy; // legacy version compatibility

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
using IMUResetReason = BNO08xResetReason; // legacy version compatibility

/// @brief Activity Classifier enable bits passed to enable_activity_classifier()
enum class ActivityEnable
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

/// @brief Activity states returned from get_activity_classifier()
enum class Activity
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

/// @brief Stability states returned from get_stability_classifier()
enum class Stability
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

typedef bno08x_config_t imu_config_t; // legacy version compatibility