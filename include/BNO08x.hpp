/**
 * @file BNO08x.hpp
 * @author Myles Parfeniuk
 */
#pragma once
// standard library includes
#include <inttypes.h>
#include <math.h>
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
        BNO08x(bno08x_config_t imu_config = bno08x_config_t());
        ~BNO08x();
        bool initialize();

        void hard_reset();

        void register_cb(std::function<void()> cb_fxn);
        void print_product_ids(); 

    private:
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
                bool sh2_HAL;        ///< True if sh2_open() has been called successfully.

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
        esp_err_t init_sh2_HAL();

        esp_err_t deinit_gpio();
        esp_err_t deinit_gpio_inputs();
        esp_err_t deinit_gpio_outputs();
        esp_err_t deinit_hint_isr();
        esp_err_t deinit_spi();
        esp_err_t deinit_sh2_HAL();

        esp_err_t wait_for_hint();

        sh2_Hal_t sh2_HAL; ///< SH2 hardware abstraction layer struct for use with SH-2 lib.

        EventGroupHandle_t
                evt_grp_spi; ///<Event group for indicating when bno08x hint pin has triggered and when new data has been processed. Used by calls to sending or receiving functions.

        std::vector<std::function<void()>> cb_list; // Vector for storing any call-back functions added with register_cb()

        bno08x_config_t imu_config{};                   ///<IMU configuration settings
        spi_bus_config_t bus_config{};                  ///<SPI bus GPIO configuration settings
        spi_device_interface_config_t imu_spi_config{}; ///<SPI slave device settings
        spi_device_handle_t spi_hdl{};                  ///<SPI device handle
        spi_transaction_t spi_transaction{};            ///<SPI transaction handle
        bno08x_init_status_t
                init_status; ///<Initialization status of various functionality, used by deconstructor during cleanup, set during initialization.

        sh2_ProductIds_t product_ID;                 ///< TODO
        sh2_SensorValue_t* sensor_report_val = NULL; ///< TODO

        bool reset_occurred = false;

        static void IRAM_ATTR hint_handler(void* arg);

        static const constexpr uint16_t RX_DATA_LENGTH = 300U; ///<length buffer containing data received over spi

        static const constexpr TickType_t HOST_INT_TIMEOUT_DEFAULT_MS =
                3000UL /
                portTICK_PERIOD_MS; ///<Max wait between HINT being asserted by BNO08x before transaction is considered failed (in miliseconds), when no reports are enabled (ie during reset)

        static const constexpr TickType_t HARD_RESET_DELAY_MS =
                100UL /
                portTICK_PERIOD_MS; ///<How long RST pin is held low during hard reset (min 10ns according to datasheet, but should be longer for stable operation)

        static const constexpr uint32_t SCLK_MAX_SPEED = 3000000UL; ///<Max SPI SCLK speed BNO08x is capable of

        // evt_grp_spi bits
        static const constexpr EventBits_t EVT_GRP_SPI_HINT_ASSERTED_BIT =
                (1U << 0U); ///<When this bit is set it indicates the BNO08x has asserted its host interrupt pin, thus an SPI transaction should commence.
        static const constexpr EventBits_t EVT_GRP_SPI_RX_DONE_BIT =
                (1U << 1U); ///<When this bit is set it indicates a receive procedure has completed.
        static const constexpr EventBits_t EVT_GRP_SPI_TX_DONE_BIT = (1U << 2U); ///<When this bit is set, it indicates a queued packet has been sent.

        static const constexpr char* TAG = "BNO08x"; ///< Class tag used for serial print statements

        friend class BNO08xSH2HAL;
};