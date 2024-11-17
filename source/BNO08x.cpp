#include "BNO08x.hpp"
#include "BNO08x_macros.hpp"

/**
 * @brief BNO08x imu constructor.
 *
 * Construct a BNO08x object for managing a BNO08x sensor.
 *
 * @param imu_config Configuration settings (optional), default settings can be seen in bno08x_config_t
 * @return void, nothing to return
 */
BNO08x::BNO08x(bno08x_config_t imu_config)
    : spi_task_hdl(NULL)
    , data_proc_task_hdl(NULL)
    , evt_grp_spi(xEventGroupCreate())
    , evt_grp_report_en(xEventGroupCreate())
    , evt_grp_task_flow(xEventGroupCreate())
    , queue_rx_data(xQueueCreate(1, sizeof(bno08x_rx_packet_t)))
    , queue_tx_data(xQueueCreate(1, sizeof(bno08x_tx_packet_t)))
    , queue_frs_read_data(xQueueCreate(1, RX_DATA_LENGTH * sizeof(uint8_t)))
    , queue_reset_reason(xQueueCreate(1, sizeof(uint32_t)))
    , imu_config(imu_config)
    , calibration_status(1)
{
}

/**
 * @brief BNO08x imu deconstructor.
 *
 * Deconstructs a BNO08x object and releases any utilized resources.
 *
 * @return void, nothing to return
 */
BNO08x::~BNO08x()
{
    // disable interrupts before beginning so we can ensure SPI task doesn't attempt to run
    gpio_intr_disable(imu_config.io_int);

    // delete any tasks if they have been created
    ESP_ERROR_CHECK(kill_all_tasks());

    // deinitialize spi if has been initialized
    ESP_ERROR_CHECK(deinit_spi());

    // deinitialize hint ISR if it has been initialized
    ESP_ERROR_CHECK(deinit_hint_isr());

    // deinitialize GPIO if they have been initialized
    ESP_ERROR_CHECK(deinit_gpio());

    // delete queues
    vQueueDelete(queue_rx_data);
    vQueueDelete(queue_tx_data);
    vQueueDelete(queue_frs_read_data);
    vQueueDelete(queue_reset_reason);

    // delete event groups
    vEventGroupDelete(evt_grp_spi);
    vEventGroupDelete(evt_grp_report_en);
    vEventGroupDelete(evt_grp_task_flow);

    // clear callback list
    cb_list.clear();
}

/**
 * @brief Initializes BNO08x sensor
 *
 * Resets sensor and goes through initialization process.
 * Configures GPIO, required ISRs, and launches two tasks, one to manage SPI transactions, another to process any received data.
 *
 * @return true if initialization was success, false if otherwise
 */
bool BNO08x::initialize()
{
    // initialize configuration arguments
    if (init_config_args() != ESP_OK)
        return false;

    // initialize GPIO
    if (init_gpio() != ESP_OK)
        return false;

    // initialize HINT ISR
    if (init_hint_isr() != ESP_OK)
        return false;

    // initialize SPI
    if (init_spi() != ESP_OK)
        return false;

    // launch tasks
    if (launch_tasks() != ESP_OK)
        return false;

    // reset BNO08x
    if (!hard_reset())
        return false;

    if (get_reset_reason() == BNO08xResetReason::UNDEFINED)
    {
        ESP_LOGE(TAG, "Initialization failed, undefined reset reason returned after reset.");
        return false;
    }

    ESP_LOGI(TAG, "Successfully initialized....");
    return true;
}

/**
 * @brief Initializes required esp-idf SPI data structures with values from user passed bno08x_config_t struct.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_config_args()
{
    if ((imu_config.io_cs == GPIO_NUM_NC))
    {
        ESP_LOGE(TAG, "Initialization failed, CS GPIO cannot be unassigned.");
        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_miso == GPIO_NUM_NC))
    {
        ESP_LOGE(TAG, "Initialization failed, MISO GPIO cannot be unassigned.");
        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_mosi == GPIO_NUM_NC))
    {
        ESP_LOGE(TAG, "Initialization failed, MOSI GPIO cannot be unassigned.");
        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_sclk == GPIO_NUM_NC))
    {
        ESP_LOGE(TAG, "Initialization failed, SCLK GPIO cannot be unassigned.");
        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_rst == GPIO_NUM_NC))
    {
        ESP_LOGE(TAG, "RST GPIO cannot be unassigned.");
        return ESP_ERR_INVALID_ARG;
    }

    reset_all_data_to_defaults(); // reset data members that are returned by public getter APIs (for ex. get_roll_deg())

    // SPI bus config
    bus_config.mosi_io_num = imu_config.io_mosi; // assign mosi gpio pin
    bus_config.miso_io_num = imu_config.io_miso; // assign miso gpio pin
    bus_config.sclk_io_num = imu_config.io_sclk; // assign sclk gpio pin
    bus_config.quadhd_io_num = -1;               // hold signal gpio (not used)
    bus_config.quadwp_io_num = -1;               // write protect signal gpio (not used)

    // SPI slave device specific config
    imu_spi_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus high when idle)

    if (imu_config.sclk_speed > SCLK_MAX_SPEED) // max sclk speed of 3MHz for BNO08x
    {
        ESP_LOGE(TAG, "Max SPI clock speed exceeded, %ld overwritten with 3MHz", imu_config.sclk_speed);
        imu_config.sclk_speed = SCLK_MAX_SPEED;
    }

    imu_spi_config.clock_source = SPI_CLK_SRC_DEFAULT;
    imu_spi_config.clock_speed_hz = imu_config.sclk_speed; // assign SCLK speed
    imu_spi_config.address_bits = 0;                       // 0 address bits, not using this system
    imu_spi_config.command_bits = 0;                       // 0 command bits, not using this system
    imu_spi_config.spics_io_num = -1;                      // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
                                                           // driver, it must be handled via calls to gpio pins
    imu_spi_config.queue_size = static_cast<int>(CONFIG_ESP32_BNO08X_SPI_QUEUE_SZ); // set max allowable queued SPI transactions

    return ESP_OK;
}

/**
 * @brief Initializes required gpio inputs.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio_inputs()
{
    esp_err_t ret = ESP_OK;

    // configure input(s) (HINT)
    gpio_config_t inputs_config;
    inputs_config.pin_bit_mask = (1ULL << imu_config.io_int);
    inputs_config.mode = GPIO_MODE_INPUT;
    inputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    inputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputs_config.intr_type = GPIO_INTR_NEGEDGE;

    ret = gpio_config(&inputs_config);

    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Initialization failed, failed to configure HINT gpio.");
    else
        init_status.gpio_inputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up

    return ret;
}

/**
 * @brief Initializes required gpio outputs.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio_outputs()
{
    esp_err_t ret = ESP_OK;

    // configure output(s) (CS, RST, and WAKE)
    gpio_config_t outputs_config;

    outputs_config.pin_bit_mask = (imu_config.io_wake != GPIO_NUM_NC)
                                          ? ((1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst) | (1ULL << imu_config.io_wake))
                                          : ((1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst));

    outputs_config.mode = GPIO_MODE_OUTPUT;
    outputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs_config.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&outputs_config);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Initialization failed, failed to configure CS, RST, and WAKE (if used) gpio.");
    else
        init_status.gpio_outputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up

    return ret;
}

/**
 * @brief Initializes required gpio.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio()
{
    esp_err_t ret = ESP_OK;

    /*GPIO config for pins not controlled by SPI peripheral*/

    ret = init_gpio_outputs();
    if (ret != ESP_OK)
        return ret;

    ret = init_gpio_inputs();
    if (ret != ESP_OK)
        return ret;

    gpio_set_level(imu_config.io_cs, 1);
    gpio_set_level(imu_config.io_rst, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    return ret;
}

/**
 * @brief Initializes host interrupt ISR.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_hint_isr()
{
    esp_err_t ret = ESP_OK;

    // check if installation of ISR service has been requested by user (default is true)
    if (imu_config.install_isr_service)
        ret = gpio_install_isr_service(0); // install isr service

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialization failed, failed to install global ISR service.");
        return ret;
    }
    else
    {
        init_status.isr_service = true; // set isr service to initialized such that deconstructor knows to clean it up (this will be ignored if
                                        // imu_config.install_isr_service == false)
    }

    ret = gpio_isr_handler_add(imu_config.io_int, hint_handler, (void*) this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialization failed, failed to add hint_handler ISR.");
        return ret;
    }
    else
    {
        init_status.isr_handler = true; // set isr handler to initialized such that deconstructor knows to clean it up
    }

    gpio_intr_disable(imu_config.io_int); // disable interrupts initially before reset

    return ret;
}

/**
 * @brief Initializes SPI.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_spi()
{
    esp_err_t ret = ESP_OK;
    uint8_t tx_buffer[50] = {0}; // for dummy transaction to stabilize SPI peripheral

    // initialize the spi peripheral
    ret = spi_bus_initialize(imu_config.spi_peripheral, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialization failed, SPI bus failed to initialize.");
        return ret;
    }
    else
    {
        init_status.spi_bus = true;
    }

    // add the imu device to the bus
    ret = spi_bus_add_device(imu_config.spi_peripheral, &imu_spi_config, &spi_hdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialization failed, failed to add device to SPI bus.");
        return ret;
    }
    else
    {
        init_status.spi_device = true;
    }

    // do first SPI operation into nowhere before BNO085 reset to let periphiral stabilize (Anton B.)
    spi_transaction.length = 8;
    spi_transaction.rxlength = 0;
    spi_transaction.tx_buffer = tx_buffer;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.flags = 0;
    spi_device_polling_transmit(spi_hdl, &spi_transaction); // send data packet

    return ret;
}

/**
 * @brief Deinitializes GPIO, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio()
{
    esp_err_t ret = ESP_OK;

    if (init_status.gpio_inputs)
    {
        ret = deinit_gpio_inputs();
        if (ret != ESP_OK)
            return ret;
    }

    if (init_status.gpio_outputs)
    {
        ret = deinit_gpio_outputs();
        if (ret != ESP_OK)
            return ret;
    }

    return ret;
}

/**
 * @brief Deinitializes GPIO inputs, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio_inputs()
{
    esp_err_t ret = ESP_OK;

    ret = gpio_reset_pin(imu_config.io_int);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio HINT pin to default state.");

    return ret;
}

/**
 * @brief Deinitializes GPIO outputs, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio_outputs()
{
    esp_err_t ret = ESP_OK;

    if (imu_config.io_wake != GPIO_NUM_NC)
    {
        ret = gpio_reset_pin(imu_config.io_wake);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Deconstruction failed, could reset gpio WAKE pin to default state.");
            return ret;
        }
    }

    ret = gpio_reset_pin(imu_config.io_cs);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio CS pin to default state.");
        return ret;
    }

    ret = gpio_reset_pin(imu_config.io_rst);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio RST pin to default state.");
        return ret;
    }

    return ret;
}

/**
 * @brief Deinitializes host interrupt ISR, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_hint_isr()
{
    esp_err_t ret = ESP_OK;

    if (init_status.isr_handler)
    {
        ret = gpio_isr_handler_remove(imu_config.io_int);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Deconstruction failed, could not remove hint ISR handler.");
            return ret;
        }
    }

    if (init_status.isr_service)
    {
        // only remove the ISR service if it was requested to be installed by user
        if (imu_config.install_isr_service)
        {
            gpio_uninstall_isr_service();
        }
    }

    return ret;
}

/**
 * @brief Deinitializes SPI.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_spi()
{
    esp_err_t ret = ESP_OK;

    if (init_status.spi_device)
    {
        ret = spi_bus_remove_device(spi_hdl);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Deconstruction failed, could not remove spi device.");
            return ret;
        }
    }

    if (init_status.spi_bus)
    {
        ret = spi_bus_free(imu_config.spi_peripheral);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Deconstruction failed, could free SPI peripheral.");
            return ret;
        }
    }

    return ret;
}

/**
 * @brief Waits for data to be received over SPI, or host_int_timeout_ms to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 * This function is for when the validity of packets is not a concern, it is for flushing packets
 * we do not care about.
 *
 * @return True if data has been received over SPI within host_int_timeout_ms.
 */
bool BNO08x::wait_for_rx_done()
{
    bool success = false;

    // if no reports are enabled we can assume interrupts are disabled (see spi_task())
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    // wait until an interrupt has been asserted and data received or timeout has occured
    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT, pdTRUE, pdTRUE, host_int_timeout_ms))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                ESP_LOGI(TAG, "int asserted");
        #endif
        // clang-format on

        success = true;
    }
    else
    {
        ESP_LOGE(TAG, "Interrupt to host device never asserted.");
        success = false;
    }

    return success;
}

/**
 * @brief Waits for a valid or invalid packet to be received or host_int_timeout_ms to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 *
 * @return True if valid packet has been received within host_int_timeout_ms, false if otherwise.
 */
bool BNO08x::wait_for_data()
{
    bool success = false;

    // if no reports are enabled we can assume interrupts are disabled (see spi_task())
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    // check to see receive operation has finished
    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT, pdTRUE, pdTRUE, host_int_timeout_ms))
    {
        // wait until processing is done, this should never go to timeout; however, it will be set slightly after EVT_GRP_SPI_RX_DONE_BIT
        if (xEventGroupWaitBits(
                    evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET_BIT | EVT_GRP_SPI_RX_INVALID_PACKET_BIT, pdFALSE, pdFALSE, host_int_timeout_ms))
        {
            // only return true if packet is valid
            if (xEventGroupGetBits(evt_grp_spi) & EVT_GRP_SPI_RX_VALID_PACKET_BIT)
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                                ESP_LOGI(TAG, "Valid packet received.");
                #endif
                // clang-format on

                success = true;
            }
            else
            {
                ESP_LOGE(TAG, "Invalid packet received.");
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "Interrupt to host device never asserted.");
    }

    xEventGroupClearBits(evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET_BIT | EVT_GRP_SPI_RX_INVALID_PACKET_BIT);
    return success;
}

/**
 * @brief Waits for a queued packet to be sent or host_int_timeout_ms to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 *
 * @return True if packet was sent within host_int_timeout_ms, false if otherwise.
 */
bool BNO08x::wait_for_tx_done()
{
    /* if no reports are enabled we can assume interrupts are disabled (see spi_task()) */
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_TX_DONE_BIT, pdTRUE, pdTRUE, host_int_timeout_ms))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                ESP_LOGI(TAG, "Packet sent successfully.");
        #endif
        // clang-format on

        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Packet failed to send.");
    }

    return false;
}

/**
 * @brief Hard resets BNO08x sensor.
 *
 * @return void, nothing to return
 */
bool BNO08x::hard_reset()
{
    bool success = false;

    // resetting disables all reports
    xEventGroupClearBits(evt_grp_report_en, EVT_GRP_RPT_ALL_BITS);

    gpio_set_level(imu_config.io_cs, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    gpio_set_level(imu_config.io_rst, 0); // set reset pin low
    vTaskDelay(HARD_RESET_DELAY_MS);      // 10ns min, set to larger delay to let things stabilize(Anton)
    gpio_set_level(imu_config.io_rst, 1); // bring out of reset

    // Receive advertisement message on boot (see SH2 Ref. Manual 5.2 & 5.3)
    if (!wait_for_rx_done()) // wait for receive operation to complete
    {
        ESP_LOGE(TAG, "Reset Failed, interrupt to host device never asserted.");
    }
    else
    {
        if (first_boot)
            ESP_LOGI(TAG, "Received advertisement message.");

        // The BNO080 will then transmit an unsolicited Initialize Response (see SH2 Ref. Manual 6.4.5.2)
        if (!wait_for_rx_done())
        {
            ESP_LOGE(TAG, "Failed to receive initialize response on boot.");
        }
        else if (first_boot)
        {
            ESP_LOGI(TAG, "Received initialize response.");
            success = true;
        }
    }

    return success;
}

/**
 * @brief Soft resets BNO08x sensor using executable channel.
 *
 * @return True if reset was success.
 */
bool BNO08x::soft_reset()
{
    bool success = false;
    uint8_t commands[20] = {0};

    // reseting resets all reports
    xEventGroupClearBits(evt_grp_report_en, EVT_GRP_RPT_ALL_BITS);

    commands[0] = 1;
    queue_packet(CHANNEL_EXECUTABLE, 1, commands);
    success = wait_for_tx_done();

    // flush any packets received
    flush_rx_packets(3);

    return success;
}

/**
 * @brief Requests product ID, prints the returned info over serial, and returns the reason for the most resent reset.
 *
 * @return The reason for the most recent recent reset ( 1 = POR (power on reset), 2 = internal reset, 3 = watchdog
 * timer, 4 = external reset 5 = other)
 */
BNO08xResetReason BNO08x::get_reset_reason()
{
    uint32_t reset_reason = 0;

    // queue request for product ID command
    queue_request_product_id_command();
    // wait for transmit to finish
    if (!wait_for_tx_done())
        ESP_LOGE(TAG, "Failed to send product ID report request");
    else
    {
        // receive product ID report
        if (wait_for_data())
            xQueueReceive(queue_reset_reason, &reset_reason, host_int_timeout_ms);
        else
            ESP_LOGE(TAG, "Failed to receive product ID report.");
    }

    return static_cast<BNO08xResetReason>(reset_reason);
}

/**
 * @brief Turns on/ brings BNO08x sensor out of sleep mode using executable channel.
 *
 * @return True if exiting sleep mode was success.
 */
bool BNO08x::mode_on()
{
    bool success = false;
    uint8_t commands[20] = {0};

    commands[0] = 2;
    queue_packet(CHANNEL_EXECUTABLE, 1, commands);
    success = wait_for_tx_done();

    // flush any packets received
    flush_rx_packets(3);

    return success;
}

/**
 * @brief Puts BNO08x sensor into sleep/low power mode using executable channel.
 *
 * @return True if entering sleep mode was success.
 */
bool BNO08x::mode_sleep()
{
    bool success = false;
    uint8_t commands[20] = {0};

    commands[0] = 3;
    queue_packet(CHANNEL_EXECUTABLE, 1, commands);
    success = wait_for_tx_done();

    // flush any packets received
    flush_rx_packets(3);

    return success;
}

/**
 * @brief Receives a SHTP packet via SPI and sends it to data_proc_task()
 *
 * @return void, nothing to return
 */
esp_err_t BNO08x::receive_packet()
{
    bno08x_rx_packet_t packet;
    esp_err_t ret = ESP_OK;

    if (gpio_get_level(imu_config.io_int)) // ensure INT pin is low
        return ESP_ERR_INVALID_STATE;

    gpio_set_level(imu_config.io_cs, 0); // assert chip select

    // receive packet header
    ret = receive_packet_header(&packet);
    if (ret != ESP_OK)
    {
        gpio_set_level(imu_config.io_cs, 1); // de-assert chip select
        return ret;
    }

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        ESP_LOGW(TAG, "packet rx length: %d", packet.length);
    #endif
    // clang-format on

    if (packet.length == 0)
    {
        gpio_set_level(imu_config.io_cs, 1); // de-assert chip select
        return ESP_ERR_INVALID_RESPONSE;
    }

    ret = receive_packet_body(&packet);
    if (ret == ESP_OK)
    {
        xQueueSend(queue_rx_data, &packet, 0); // send received data to data_proc_task
        xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT);
    }

    gpio_set_level(imu_config.io_cs, 1); // de-assert chip select

    return ret;
}

/**
 * @brief Receives a SHTP packet header via SPI.
 *
 * @param packet Pointer to bno08x_rx_packet_t to save header to.
 *
 * @return ESP_OK if receive was success.
 */
esp_err_t BNO08x::receive_packet_header(bno08x_rx_packet_t* packet)
{

    esp_err_t ret = ESP_OK;
    uint8_t dummy_header_tx[4] = {0};

    // setup transaction to receive first 4 bytes (packet header)
    spi_transaction.rx_buffer = packet->header;
    spi_transaction.tx_buffer = dummy_header_tx;
    spi_transaction.length = 4 * 8;
    spi_transaction.rxlength = 4 * 8;
    spi_transaction.flags = 0;

    ret = spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive first 4 bytes (packet header)

    if (ret == ESP_OK)
    {
        // calculate length of packet from received header
        packet->length = PARSE_PACKET_LENGTH(packet);
        packet->length &= ~(1U << 15U); // clear the MSbit
    }

    return ret;
}

/**
 * @brief Receives a SHTP packet body via SPI.
 *
 * @param packet Pointer to bno08x_rx_packet_t to save body to.
 *
 * @return ESP_OK if receive was success.
 */
esp_err_t BNO08x::receive_packet_body(bno08x_rx_packet_t* packet)
{
    esp_err_t ret = ESP_OK;

    packet->length -= 4; // remove 4 header bytes from packet length (we already read those)

    // setup transacton to read the data packet
    spi_transaction.rx_buffer = packet->body;
    spi_transaction.tx_buffer = NULL;
    spi_transaction.length = packet->length * 8;
    spi_transaction.rxlength = packet->length * 8;
    spi_transaction.flags = 0;

    ret = spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive rest of packet

    return ret;
}

/**
 * @brief Enables a sensor report for a given ID.
 *
 * @param report_ID The report ID of the sensor, i.e. SENSOR_REPORT_ID_X
 * @param time_between_reports The desired time in microseconds between each report. The BNO08x will send reports according to this interval.
 * @param report_evt_grp_bit The event group bit for the respective report, to indicate to spi_task() it's enabled, i.e. EVT_GRP_RPT_X
 *
 * If no reports were enabled prior to call, this function will re-enable interrupts on hint pin.
 *
 * @return void, nothing to return
 */
void BNO08x::enable_report(uint8_t report_ID, uint32_t time_between_reports, const EventBits_t report_evt_grp_bit, uint32_t special_config)
{
    // if no reports have been enabled before this one, we can assume the IMU has gone to sleep, wake it up with a reset
    if ((xEventGroupGetBits(evt_grp_report_en) & ~report_evt_grp_bit) == 0)
        hard_reset();

    update_report_period_trackers(report_ID, time_between_reports);

    queue_feature_command(report_ID, time_between_reports, special_config);
    if (wait_for_tx_done()) // wait for transmit operation to complete
    {
        xEventGroupSetBits(evt_grp_report_en, report_evt_grp_bit);

        // if no reports were enabled before this one, we can assume hint interrupt was disabled, re-enable to read reports
        if ((xEventGroupGetBits(evt_grp_report_en) & ~report_evt_grp_bit) == 0)
        {
            gpio_intr_enable(imu_config.io_int);
        }
    }

    // flush the first few reports returned to ensure new data
    flush_rx_packets(2);
}

/**
 * @brief Disables a sensor report for a given ID by setting its time interval to 0.
 *
 * @param report_ID The report ID of the sensor, i.e. SENSOR_REPORT_ID_X
 * @param report_evt_grp_bit The event group bit for the respective report, to indicate to spi_task() it's disabled, i.e. EVT_GRP_RPT_X
 *
 * If no reports are enabled after disabling, this function will disable interrupts on hint pin.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_report(uint8_t report_ID, const EventBits_t report_evt_grp_bit)
{
    update_report_period_trackers(report_ID, 0);
    queue_feature_command(report_ID, 0);
    if (wait_for_tx_done()) // wait for transmit operation to complete
    {
        xEventGroupClearBits(evt_grp_report_en, report_evt_grp_bit);

        /*
        6.5.5 of SH-2 Ref manual: "Note that SH-2 protocol version 1.0.1 and higher will send Get Feature Response messages
        unsolicited if a sensor’s rate changes (e.g. due to change in the rate of a related sensor."

        For ex. after calling disable_rotation_vector(), the following response packet will be sent:

        I (3497) BNO08x: SHTP Header:
                        Raw 32 bit word: 0x15000205
                        Packet Length:   21
                        Channel Number:  2
                        Sequence Number: 5
                        Channel Type: Control
                        Body:
                                0xFC  0x05  0x00  0x00  0x00  0x00
                                0x00  0x00  0x00  0x00  0x00  0x00
                                0x00  0x00  0x00  0x00  0x00

        The 0xFC indicates it is a get feature response, the 0x05 indicates it is for the rotation vector, the rest of the body will be 0.
        It might be wise to  detect the response for the respective report being disabled, but is probably not necessary. For now, all get feature
        responses are detected as valid packets.
        */

        // no reports enabled, disable hint to avoid wasting processing time
        if ((xEventGroupGetBits(evt_grp_report_en)) == 0)
        {
            host_int_timeout_ms = HOST_INT_TIMEOUT_DEFAULT_MS;
            gpio_intr_disable(imu_config.io_int);
        }
    }
}

/**
 * @brief Queues an SHTP packet to be sent via SPI.
 *
 * @param SHTP channel number
 * @param data_length data length in bytes
 * @param commands array containing data to be sent
 *
 * @return void, nothing to return
 */
void BNO08x::queue_packet(uint8_t channel_number, uint8_t data_length, uint8_t* commands)
{

    static uint8_t sequence_number[6] = {0}; // Sequence num of each com channel, 6 in total
    bno08x_tx_packet_t packet;

    packet.length = data_length + 4; // add 4 bytes for header

    packet.body[0] = packet.length & 0xFF;              // packet length LSB
    packet.body[1] = packet.length >> 8;                // packet length MSB
    packet.body[2] = channel_number;                    // channel number to write to
    packet.body[3] = sequence_number[channel_number]++; // increment and send sequence number (packet counter)

    // save commands to send to tx_buffer
    for (int i = 0; i < data_length; i++)
    {
        packet.body[i + 4] = commands[i];
    }

    xQueueSend(queue_tx_data, &packet, 0);
}

/**
 * @brief Sends a queued SHTP packet via SPI.
 *
 * @param packet The packet queued to be sent.
 *
 * @return void, nothing to return
 */
void BNO08x::send_packet(bno08x_tx_packet_t* packet)
{
    // setup transaction to send packet
    spi_transaction.length = packet->length * 8;
    spi_transaction.rxlength = 0;
    spi_transaction.tx_buffer = packet->body;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.flags = 0;

    gpio_set_level(imu_config.io_cs, 0);                    // assert chip select
    spi_device_polling_transmit(spi_hdl, &spi_transaction); // send data packet

    gpio_set_level(imu_config.io_cs, 1); // de-assert chip select

    xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_TX_DONE_BIT);
}

void BNO08x::flush_rx_packets(uint8_t flush_count)
{
    for (int i = 0; i < flush_count; i++)
        wait_for_rx_done();
}

/**
 * @brief Queues a packet containing a command.
 *
 * @param command The command to be sent.
 * @param commands Command data array, pre-packed with exception of first 3 elements (command info)
 *
 * @return void, nothing to return
 */
void BNO08x::queue_command(uint8_t command, uint8_t* commands)
{
    static uint8_t command_sequence_number = 0; // Sequence num of command, sent within command packet.

    commands[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
    commands[1] = command_sequence_number++;   // Increments automatically each function call
    commands[2] = command;                     // Command

    queue_packet(CHANNEL_CONTROL, 12, commands);
}

/**
 * @brief Queues a packet containing the request product ID command.
 *
 * @return void, nothing to return
 */
void BNO08x::queue_request_product_id_command()
{
    uint8_t commands[20] = {0};

    commands[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // request product ID and reset info
    commands[1] = 0;                              // reserved
    queue_packet(CHANNEL_CONTROL, 2, commands);
}

/**
 * @brief Sends command to calibrate accelerometer, gyro, and magnetometer.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_all()
{
    queue_calibrate_command(CALIBRATE_ACCEL_GYRO_MAG);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate accelerometer.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_accelerometer()
{
    queue_calibrate_command(CALIBRATE_ACCEL);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate gyro.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_gyro()
{
    queue_calibrate_command(CALIBRATE_GYRO);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate magnetometer.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_magnetometer()
{
    queue_calibrate_command(CALIBRATE_MAG);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate planar accelerometer
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_planar_accelerometer()
{
    queue_calibrate_command(CALIBRATE_PLANAR_ACCEL);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Queues a packet containing a command to calibrate the specified sensor.
 *
 * @param sensor_to_calibrate The sensor to calibrate.
 *
 * @return void, nothing to return
 */
void BNO08x::queue_calibrate_command(uint8_t sensor_to_calibrate)
{
    uint8_t commands[20] = {0};

    switch (sensor_to_calibrate)
    {
        case CALIBRATE_ACCEL:
            commands[3] = 1;
            break;

        case CALIBRATE_GYRO:
            commands[4] = 1;
            break;

        case CALIBRATE_MAG:
            commands[5] = 1;
            break;

        case CALIBRATE_PLANAR_ACCEL:
            commands[7] = 1;
            break;

        case CALIBRATE_ACCEL_GYRO_MAG:
            commands[3] = 1;
            commands[4] = 1;
            commands[5] = 1;
            break;

        case CALIBRATE_STOP:
            // do nothing, send packet of all 0s
            break;

        default:

            break;
    }

    calibration_status = 1;

    queue_command(COMMAND_ME_CALIBRATE, commands);
}

/**
 * @brief Requests ME calibration status from BNO08x (see Ref. Manual 6.4.7.2)
 *
 * @return void, nothing to return
 */
void BNO08x::request_calibration_status()
{
    uint8_t commands[20] = {0};

    commands[6] = 0x01; // P3 - 0x01 - Subcommand: Get ME Calibration

    // Using this commands packet, send a command
    queue_command(COMMAND_ME_CALIBRATE, commands);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Returns true if calibration has completed.
 *
 * @return True if calibration complete, false if otherwise.
 */
bool BNO08x::calibration_complete()
{
    if (calibration_status == 0)
        return true;

    return false;
}

/**
 * @brief Sends command to end calibration procedure.
 *
 * @return void, nothing to return
 */
void BNO08x::end_calibration()
{
    queue_calibrate_command(CALIBRATE_STOP); // Disables all calibrations
    wait_for_tx_done();                      // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS);      // allow some time for command to be executed
}

/**
 * @brief Sends command to save internal calibration data (See Ref. Manual 6.4.7).
 *
 * @return void, nothing to return
 */
void BNO08x::save_calibration()
{
    uint8_t commands[20] = {0};

    // Using this shtpData packet, send a command
    queue_command(COMMAND_DCD, commands); // Save DCD command
    wait_for_tx_done();                   // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS);   // allow some time for command to be executed
}

/**
 * @brief Runs full calibration routine.
 *
 * Enables game rotation vector and magnetometer, starts ME calibration process.
 * Waits for accuracy of returned quaternions and magnetic field vectors to be high, then saves calibration data and
 * returns.
 *
 * @return True if calibration succeeded, false if otherwise.
 */
bool BNO08x::run_full_calibration_routine()
{
    float magf_x = 0;
    float magf_y = 0;
    float magf_z = 0;
    BNO08xAccuracy magnetometer_accuracy = BNO08xAccuracy::LOW;

    float quat_I = 0;
    float quat_J = 0;
    float quat_K = 0;
    float quat_real = 0;
    BNO08xAccuracy quat_accuracy = BNO08xAccuracy::LOW;

    uint16_t high_accuracy = 0;
    uint16_t save_calibration_attempt = 0;

    // Enable dynamic calibration for accel, gyro, and mag
    calibrate_all(); // Turn on cal for Accel, Gyro, and Mag

    // Enable Game Rotation Vector output
    enable_game_rotation_vector(100000UL); // Send data update every 100ms

    // Enable Magnetic Field output
    enable_magnetometer(100000UL); // Send data update every 100ms

    while (1)
    {
        if (data_available())
        {
            magf_x = get_magf_X();
            magf_y = get_magf_Y();
            magf_z = get_magf_Z();
            magnetometer_accuracy = get_magf_accuracy();

            quat_I = get_quat_I();
            quat_J = get_quat_J();
            quat_K = get_quat_K();
            quat_real = get_quat_real();
            quat_accuracy = get_quat_accuracy();

            ESP_LOGI(TAG, "Magnetometer: x: %.3f y: %.3f z: %.3f, accuracy: %d", magf_x, magf_y, magf_z, static_cast<uint8_t>(magnetometer_accuracy));
            ESP_LOGI(TAG, "Quaternion Rotation Vector: i: %.3f j: %.3f k: %.3f, real: %.3f, accuracy: %d", quat_I, quat_J, quat_K, quat_real,
                    static_cast<uint8_t>(quat_accuracy));

            vTaskDelay(5 / portTICK_PERIOD_MS);

            if ((magnetometer_accuracy >= BNO08xAccuracy::MED) && (quat_accuracy == BNO08xAccuracy::HIGH))
                high_accuracy++;
            else
                high_accuracy = 0;

            if (high_accuracy > 10)
            {
                save_calibration();
                request_calibration_status();

                save_calibration_attempt = 0;

                while (save_calibration_attempt < 20)
                {
                    if (data_available())
                    {
                        if (calibration_complete())
                        {
                            ESP_LOGW(TAG, "Calibration data successfully stored.");
                            return true;
                        }
                        else
                        {
                            save_calibration();
                            request_calibration_status();
                            save_calibration_attempt++;
                        }
                    }
                }

                vTaskDelay(1 / portTICK_PERIOD_MS);

                if (save_calibration_attempt >= 20)
                    ESP_LOGE(TAG, "Calibration data failed to store.");

                return false;
            }
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Checks if BNO08x has asserted interrupt and sent data.
 *
 * @param ignore_no_reports_enabled Forces a wait for data even if no reports are enabled (default is false), used for unit tests.
 *
 * @return True if new data has been parsed and saved, false if otherwise.
 */
bool BNO08x::data_available(bool ignore_no_reports_enabled)
{
    if (!ignore_no_reports_enabled)
        if (xEventGroupGetBits(evt_grp_report_en) == 0)
        {
            ESP_LOGE(TAG, "No reports enabled.");
            return false;
        }

    return wait_for_data();
}

/**
 * @brief Registers a callback to execute when new data from a report is received.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type and void input parameters.
 *
 * @return void, nothing to return
 */
void BNO08x::register_cb(std::function<void()> cb_fxn)
{
    cb_list.push_back(cb_fxn);
}

/**
 * @brief Parses a packet received from bno08x, updating any data according to received reports.
 *
 * @param packet The packet to be parsed.
 * @param notify_users Bool reference that is set to true if users should be notified of new data through callbacks/polling, false if packet is valid
 *                     but users don't need to be notified.
 *
 * @return 0 if invalid packet, non-zero if otherwise.
 */
uint16_t BNO08x::parse_packet(bno08x_rx_packet_t* packet, bool& notify_users)
{
    notify_users = true;

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        ESP_LOGW(TAG, "SHTP Header RX'd: 0x%X 0x%X 0x%X 0x%X", packet->header[0], packet->header[1], packet->header[2], packet->header[3]);
    #endif
    // clang-format on

    switch (packet->body[0])
    {
        case SHTP_REPORT_PRODUCT_ID_RESPONSE:
            return parse_product_id_report(packet);
            break;

        case SHTP_REPORT_GET_FEATURE_RESPONSE:
            notify_users = false;
            return parse_feature_get_response_report(packet);
            break;

        case SHTP_REPORT_FRS_READ_RESPONSE:
            return parse_frs_read_response_report(packet);
            break;

        default:

            break;
    }

    switch (packet->header[2])
    {
        case CHANNEL_REPORTS:
            if (packet->body[0] == SHTP_REPORT_BASE_TIMESTAMP)
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                ESP_LOGI(TAG, "RX'd packet, channel report");
                #endif
                // clang-format on

                // this will update the rawAccelX, etc variables depending on which feature report is found
                return parse_input_report(packet);
            }
            break;

        case CHANNEL_CONTROL:
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                    ESP_LOGI(TAG, "RX'd packet, channel control");
            #endif
            // clang-format on

            // this will update responses to commands, calibrationStatus, etc.
            return parse_command_report(packet);

            break;

        case CHANNEL_GYRO:
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                    ESP_LOGI(TAG, "Rx packet, channel gyro");
            #endif
            // clang-format on

            // this will update the rawAccelX, etc variables depending on which feature report is found
            return parse_gyro_integrated_rotation_vector_report(packet);

            break;

        case CHANNEL_COMMAND:
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                    ESP_LOGI(TAG, "Rx packet, channel command");
            #endif
            // clang-format on

            // todo add proper handling
            notify_users = false;
            return 1;
            break;

        case CHANNEL_WAKE_REPORTS:
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                    ESP_LOGI(TAG, "Rx packet, wake reports");
            #endif
            // clang-format on

            // todo add proper handling
            notify_users = false;
            return 1;
            break;

        default:

            break;
    }

    return 0;
}

/**
 * @brief Parses product id report and prints device info.
 *
 * @param packet The packet containing product id report.
 *
 * @return 1, always valid.
 */
uint16_t BNO08x::parse_product_id_report(bno08x_rx_packet_t* packet)
{
    const uint32_t product_id = PARSE_PRODUCT_ID_REPORT_PRODUCT_ID(packet);
    const uint32_t reset_reason = PARSE_PRODUCT_ID_REPORT_RESET_REASON(packet);
    const uint32_t sw_part_number = PARSE_PRODUCT_ID_REPORT_SW_PART_NO(packet);
    const uint32_t sw_version_major = PARSE_PRODUCT_ID_REPORT_SW_VERSION_MAJOR(packet);
    const uint32_t sw_version_minor = PARSE_PRODUCT_ID_REPORT_SW_VERSION_MINOR(packet);
    const uint32_t sw_build_number = PARSE_PRODUCT_ID_REPORT_SW_BUILD_NO(packet);
    const uint32_t sw_version_patch = PARSE_PRODUCT_ID_REPORT_SW_VERSION_PATCH(packet);

    if (first_boot)
    {
        // print product ID info packet
        ESP_LOGI(TAG,
                "Product ID Info:                           \n\r"
                "                ---------------------------\n\r"
                "                Product ID: 0x%" PRIx32 "\n\r"
                "                SW Version Major: 0x%" PRIx32 "\n\r"
                "                SW Version Minor: 0x%" PRIx32 "\n\r"
                "                SW Part Number:   0x%" PRIx32 "\n\r"
                "                SW Build Number:  0x%" PRIx32 "\n\r"
                "                SW Version Patch: 0x%" PRIx32 "\n\r"
                "                ---------------------------\n\r",
                product_id, sw_version_major, sw_version_minor, sw_part_number, sw_build_number, sw_version_patch);

        first_boot = false;
    }

    xQueueSend(queue_reset_reason, &reset_reason, 0);

    return 1;
}

/**
 * @brief Sends packet to be parsed to meta data function call (FRS_read_data()) through queue.
 *
 * @param packet The packet containing the frs read report.
 *
 * @return 1, always valid, parsing for this happens in frs_read_word()
 */
uint16_t BNO08x::parse_frs_read_response_report(bno08x_rx_packet_t* packet)
{
    xQueueSend(queue_frs_read_data, &packet->body, 0);
    return 1;
}

uint16_t BNO08x::parse_feature_get_response_report(bno08x_rx_packet_t* packet)
{
    uint16_t report_ID = 0;

    // TODO: add get feature requests and handle this properly, this is just to handle the unsolcited get feature responses due to report rate changes
    switch (packet->body[1])
    {
        case SENSOR_REPORT_ID_ACCELEROMETER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_GYROSCOPE:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_MAGNETIC_FIELD:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_LINEAR_ACCELERATION:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_GRAVITY:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_UNCALIBRATED_GYRO:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_GAME_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_GEOMAGNETIC_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_TAP_DETECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_STEP_COUNTER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_STABILITY_CLASSIFIER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_RAW_ACCELEROMETER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_RAW_GYROSCOPE:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_RAW_MAGNETOMETER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        case SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR:
            report_ID = packet->body[0];
            break;

        default:
            break;
    }

    return report_ID;
}

/**
 * @brief Parses received input report sent by BNO08x.
 *
 * Unit responds with packet that contains the following:
 *
 * packet->header[0:3]: First, a 4 byte header
 * packet->body[0:4]: Then a 5 byte timestamp of microsecond ticks since reading was taken
 * packet->body[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector, etc...)
 * packet->body[5 + 1]: Sequence number (See Ref.Manual 6.5.8.2)
 * packet->body[5 + 2]: Status
 * packet->body[3]: Delay
 * packet->body[4:5]: i/accel x/gyro x/etc
 * packet->body[6:7]: j/accel y/gyro y/etc
 * packet->body[8:9]: k/accel z/gyro z/etc
 * packet->body[10:11]: real/gyro temp/etc
 * packet->body[12:13]: Accuracy estimate
 *
 * @param packet bno8x_rx_packet_t containing the input report to parse
 *
 * @return The report ID of the respective sensor, for ex. SENSOR_REPORT_ID_ACCELEROMETER, 0 if invalid.
 */
uint16_t BNO08x::parse_input_report(bno08x_rx_packet_t* packet)
{
    uint8_t status = PARSE_INPUT_REPORT_STATUS_BITS(packet);
    uint16_t data_length = PARSE_PACKET_LENGTH(packet);
    uint16_t report_ID = PARSE_INPUT_REPORT_REPORT_ID(packet);
    uint16_t data[6] = {0};

    data_length &= ~(1U << 15U); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // ignore it for now. TODO catch this as an error and exit

    data_length -= 4; // Remove the header bytes from the data count
    time_stamp = PARSE_PACKET_TIMESTAMP(packet);

    parse_input_report_data(packet, data, data_length);

    // Store these generic values to their proper global variable
    switch (report_ID)
    {
        case SENSOR_REPORT_ID_ACCELEROMETER:
            update_accelerometer_data(data, status);
            break;

        case SENSOR_REPORT_ID_LINEAR_ACCELERATION:
            update_lin_accelerometer_data(data, status);
            break;

        case SENSOR_REPORT_ID_GYROSCOPE:
            update_calibrated_gyro_data(data, status);
            break;

        case SENSOR_REPORT_ID_UNCALIBRATED_GYRO:
            update_uncalibrated_gyro_data(data, status);
            break;

        case SENSOR_REPORT_ID_MAGNETIC_FIELD:
            update_magf_data(data, status);
            break;

        case SENSOR_REPORT_ID_TAP_DETECTOR:
            update_tap_detector_data(packet);
            break;

        case SENSOR_REPORT_ID_STEP_COUNTER:
            update_step_counter_data(data);
            break;

        case SENSOR_REPORT_ID_STABILITY_CLASSIFIER:
            update_stability_classifier_data(packet);
            break;

        case SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER:
            update_personal_activity_classifier_data(packet);
            break;

        case SENSOR_REPORT_ID_RAW_ACCELEROMETER:
            update_raw_accelerometer_data(data);
            break;

        case SENSOR_REPORT_ID_RAW_GYROSCOPE:
            update_raw_gyro_data(data);
            break;

        case SENSOR_REPORT_ID_RAW_MAGNETOMETER:
            update_raw_magf_data(data);
            break;

        case SHTP_REPORT_COMMAND_RESPONSE:
            update_command_data(packet);
            break;

        case SENSOR_REPORT_ID_GRAVITY:
            update_gravity_data(data, status);
            break;

        default:
            if (IS_ROTATION_VECTOR_REPORT(packet))
            {
                update_rotation_vector_data(data, status);
            }

            break;
    }

    // TODO additional feature reports may be strung together. Parse them all.

    return report_ID;
}

/**
 * @brief Parses data from received input report.
 *
 * @param packet bno8x_rx_packet_t containing the input report to parse
 * @param data uint16_t array to store parsed data in
 * @param data_length length of data in bytes parsed from packet header
 *
 * @return void, nothing to return
 */
void BNO08x::parse_input_report_data(bno08x_rx_packet_t* packet, uint16_t* data, uint16_t data_length)
{

    data[0] = PARSE_INPUT_REPORT_DATA_1(packet);
    data[1] = PARSE_INPUT_REPORT_DATA_2(packet);
    data[2] = PARSE_INPUT_REPORT_DATA_3(packet);

    if (data_length - 5 > 9)
    {
        data[3] = PARSE_INPUT_REPORT_DATA_4(packet);
    }
    if (data_length - 5 > 11)
    {
        data[4] = PARSE_INPUT_REPORT_DATA_5(packet);
    }
    if (data_length - 5 > 13)
    {
        data[5] = PARSE_INPUT_REPORT_DATA_6(packet);
    }
}

/**
 * @brief Parses received gyro integrated rotation vector report sent by BNO08x.
 *
 * Unit responds with packet that contains the following:
 *
 * packet->header[0:3]: First, a 4 byte header
 * packet->body[0:1]: Raw quaternion component I
 * packet->body[2:3]: Raw quaternion component J
 * packet->body[4:5]: Raw quaternion component K
 * packet->body[6:7]: Raw quaternion real component
 * packet->body[8:9]: Raw gyroscope angular velocity in X axis
 * packet->body[10:11]: Raw gyroscope angular velocity in Y axis
 * packet->body[12:13]: Raw gyroscope angular velocity in Z axis
 *
 * @param packet bno8x_rx_packet_t containing the gyro integrated rotation vector report report to parse
 *
 * @return Integrated rotation vector report ID (always valid)
 */
uint16_t BNO08x::parse_gyro_integrated_rotation_vector_report(bno08x_rx_packet_t* packet)
{
    // the gyro-integrated input reports are sent via the special gyro channel and do not include the usual ID, sequence, and status fields
    update_integrated_gyro_rotation_vector_data(packet);

    return SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR;
}

/**
 * @brief Parses received command report sent by BNO08x (See Ref. Manual 6.3.9)
 *
 * @return The command report ID, 0 if invalid.
 */
uint16_t BNO08x::parse_command_report(bno08x_rx_packet_t* packet)
{
    uint8_t command = 0;

    if (packet->body[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        // The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        command = packet->body[2]; // This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibration_status = packet->body[5 + 0]; // R0 - Status (0 = success, non-zero = fail)
        }
        return packet->body[0];
    }
    else
    {
        // This sensor report ID is unhandled.
        // See SH2 Ref. Manual to add additional feature reports as needed
    }

    return 0;
}

/**
 * @brief Updates accelerometer data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_accelerometer_data(uint16_t* data, uint8_t status)
{
    accel_accuracy = status;
    raw_accel_X = data[0];
    raw_accel_Y = data[1];
    raw_accel_Z = data[2];
}

/**
 * @brief Updates linear accelerometer data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_lin_accelerometer_data(uint16_t* data, uint8_t status)
{
    accel_lin_accuracy = status;
    raw_lin_accel_X = data[0];
    raw_lin_accel_Y = data[1];
    raw_lin_accel_Z = data[2];
}

/**
 * @brief Updates linear gyro data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_calibrated_gyro_data(uint16_t* data, uint8_t status)
{
    raw_calib_gyro_X = data[0];
    raw_calib_gyro_Y = data[1];
    raw_calib_gyro_Z = data[2];
}

/**
 * @brief Updates uncalibrated gyro data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_uncalibrated_gyro_data(uint16_t* data, uint8_t status)
{
    raw_uncalib_gyro_X = data[0];
    raw_uncalib_gyro_Y = data[1];
    raw_uncalib_gyro_Z = data[2];
    raw_bias_X = data[3];
    raw_bias_Y = data[4];
    raw_bias_Z = data[5];
}

/**
 * @brief Updates magnetic field data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_magf_data(uint16_t* data, uint8_t status)
{
    magf_accuracy = status;
    raw_magf_X = data[0];
    raw_magf_Y = data[1];
    raw_magf_Z = data[2];
}

/**
 * @brief Updates gravity data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_gravity_data(uint16_t* data, uint8_t status)
{
    gravity_accuracy = status;
    gravity_X = data[0];
    gravity_Y = data[1];
    gravity_Z = data[2];
}

/**
 * @brief Updates roation vector data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 * @param status uint8_t containing parsed status bits (represents accuracy)
 *
 * @return void, nothing to return
 */
void BNO08x::update_rotation_vector_data(uint16_t* data, uint8_t status)
{
    quat_accuracy = status;
    raw_quat_I = data[0];
    raw_quat_J = data[1];
    raw_quat_K = data[2];
    raw_quat_real = data[3];

    // Only available on rotation vector and ar/vr stabilized rotation vector,
    //  not game rot vector and not ar/vr stabilized rotation vector
    raw_quat_radian_accuracy = data[4];
}

/**
 * @brief Updates step counter data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 *
 * @return void, nothing to return
 */
void BNO08x::update_step_counter_data(uint16_t* data)
{
    step_count = data[2]; // bytes 8/9
}

/**
 * @brief Updates raw accelerometer data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 *
 * @return void, nothing to return
 */
void BNO08x::update_raw_accelerometer_data(uint16_t* data)
{
    mems_raw_accel_X = data[0];
    mems_raw_accel_Y = data[1];
    mems_raw_accel_Z = data[2];
}

/**
 * @brief Updates raw gyro data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 *
 * @return void, nothing to return
 */
void BNO08x::update_raw_gyro_data(uint16_t* data)
{
    mems_raw_gyro_X = data[0];
    mems_raw_gyro_Y = data[1];
    mems_raw_gyro_Z = data[2];
}

/**
 * @brief Updates raw magnetic field data from parsed input report.
 *
 * @param data uint16_t array containing parsed input report data.
 *
 * @return void, nothing to return
 */
void BNO08x::update_raw_magf_data(uint16_t* data)
{
    mems_raw_magf_X = data[0];
    mems_raw_magf_Y = data[1];
    mems_raw_magf_Z = data[2];
}

/**
 * @brief Updates tap detector data from parsed input report.
 *
 * @param packet bno08x_rx_packet_t containing the packet with tap detector report.
 *
 * @return void, nothing to return
 */
void BNO08x::update_tap_detector_data(bno08x_rx_packet_t* packet)
{
    tap_detector = packet->body[5 + 4]; // Byte 4 only
}

/**
 * @brief Updates stability classifier data from parsed input report.
 *
 * @param packet bno08x_rx_packet_t containing the packet with stability classifier report.
 *
 * @return void, nothing to return
 */
void BNO08x::update_stability_classifier_data(bno08x_rx_packet_t* packet)
{
    stability_classifier = packet->body[5 + 4]; // Byte 4 only
}

/**
 * @brief Updates activity classifier data from parsed input report.
 *
 * @param packet bno08x_rx_packet_t containing the packet with activity classifier report.
 *
 * @return void, nothing to return
 */
void BNO08x::update_personal_activity_classifier_data(bno08x_rx_packet_t* packet)
{
    activity_classifier = packet->body[5 + 5]; // Most likely state

    // Load activity classification confidences into the array
    if (activity_confidences != nullptr)
        for (int i = 0; i < 9; i++)                            // Hardcoded to max of 9. TODO - bring in array size
            activity_confidences[i] = packet->body[5 + 6 + i]; // 5 bytes of timestamp, byte 6 is first confidence
                                                               // byte
}

/**
 * @brief Updates command data from parsed input report.
 *
 * @param packet bno08x_rx_packet_t containing the packet with command response report.
 *
 * @return void, nothing to return
 */
void BNO08x::update_command_data(bno08x_rx_packet_t* packet)
{
    uint8_t command = 0;

    // the BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
    command = packet->body[5 + 2]; // This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE)
        calibration_status = packet->body[5 + 5]; // R0 - Status (0 = success, non-zero = fail)
}

/**
 * @brief Updates integrated gyro rotation vector data from SHTP channel 5 (CHANNEL_GYRO) special report data.
 *
 * @param packet bno08x_rx_packet_t containing the packet with command response report.
 *
 * @return void, nothing to return
 */
void BNO08x::update_integrated_gyro_rotation_vector_data(bno08x_rx_packet_t* packet)
{
    raw_quat_I = PARSE_GYRO_REPORT_RAW_QUAT_I(packet);
    raw_quat_J = PARSE_GYRO_REPORT_RAW_QUAT_J(packet);
    raw_quat_K = PARSE_GYRO_REPORT_RAW_QUAT_K(packet);
    raw_quat_real = PARSE_GYRO_REPORT_RAW_QUAT_REAL(packet);
    integrated_gyro_velocity_X = PARSE_GYRO_REPORT_RAW_GYRO_VEL_X(packet);
    integrated_gyro_velocity_Y = PARSE_GYRO_REPORT_RAW_GYRO_VEL_Y(packet);
    integrated_gyro_velocity_Z = PARSE_GYRO_REPORT_RAW_GYRO_VEL_Z(packet);
}

/**
 * @brief Sends command to enable game rotation vector reports (See Ref. Manual 6.5.19)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_game_rotation_vector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_GAME_ROTATION_VECTOR, time_between_reports, EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to enable rotation vector reports (See Ref. Manual 6.5.18)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_rotation_vector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_ROTATION_VECTOR, time_between_reports, EVT_GRP_RPT_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to enable ARVR stabilized rotation vector reports (See Ref. Manual 6.5.42)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_ARVR_stabilized_rotation_vector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR, time_between_reports, EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to enable ARVR stabilized game rotation vector reports (See Ref. Manual 6.5.43)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_ARVR_stabilized_game_rotation_vector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR, time_between_reports, EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to enable gyro integrated rotation vector reports (See Ref. Manual 6.5.44)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_gyro_integrated_rotation_vector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR, time_between_reports, EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to enable accelerometer reports (See Ref. Manual 6.5.9)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_accelerometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_ACCELEROMETER, time_between_reports, EVT_GRP_RPT_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to enable linear accelerometer reports (See Ref. Manual 6.5.10)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_linear_accelerometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_LINEAR_ACCELERATION, time_between_reports, EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to enable gravity reading reports (See Ref. Manual 6.5.11)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_gravity(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_GRAVITY, time_between_reports, EVT_GRP_RPT_GRAVITY_BIT);
}

/**
 * @brief Sends command to enable calibrated gyro reports (See Ref. Manual 6.5.13)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_calibrated_gyro(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_GYROSCOPE, time_between_reports, EVT_GRP_RPT_GYRO_BIT);
}

/**
 * @brief Sends command to enable uncalibrated gyro reports (See Ref. Manual 6.5.14)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_uncalibrated_gyro(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_UNCALIBRATED_GYRO, time_between_reports, EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT);
}

/**
 * @brief Sends command to enable magnetometer reports (See Ref. Manual 6.5.16)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_magnetometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_MAGNETIC_FIELD, time_between_reports, EVT_GRP_RPT_MAGNETOMETER_BIT);
}

/**
 * @brief Sends command to enable tap detector reports (See Ref. Manual 6.5.27)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_tap_detector(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_TAP_DETECTOR, time_between_reports, EVT_GRP_RPT_TAP_DETECTOR_BIT);
}

/**
 * @brief Sends command to enable step counter reports (See Ref. Manual 6.5.29)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_step_counter(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_STEP_COUNTER, time_between_reports, EVT_GRP_RPT_STEP_COUNTER_BIT);
}

/**
 * @brief Sends command to enable activity stability classifier reports (See Ref. Manual 6.5.31)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_stability_classifier(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_STABILITY_CLASSIFIER, time_between_reports, EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT);
}

/**
 * @brief Sends command to enable activity classifier reports (See Ref. Manual 6.5.36)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 *  @param activities_to_enable Desired activities to enable (0x1F enables all).
 *  @param activity_confidence_vals Returned activity level confidences.
 * @return void, nothing to return
 */
void BNO08x::enable_activity_classifier(uint32_t time_between_reports, ActivityEnable activities_to_enable, uint8_t (&activity_confidence_vals)[9])
{
    activity_confidences = activity_confidence_vals; // Store pointer to array
    enable_report(SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER, time_between_reports, EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT,
            static_cast<uint16_t>(activities_to_enable));
}

/**
 * @brief Sends command to enable raw MEMs accelerometer reports (See Ref. Manual 6.5.8)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_mems_accelerometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_RAW_ACCELEROMETER, time_between_reports, EVT_GRP_RPT_RAW_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to enable raw MEMs gyro reports (See Ref. Manual 6.5.12)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_mems_gyro(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_RAW_GYROSCOPE, time_between_reports, EVT_GRP_RPT_RAW_GYRO_BIT);
}

/**
 * @brief Sends command to enable raw MEMs magnetometer reports (See Ref. Manual 6.5.15)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_mems_magnetometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_RAW_MAGNETOMETER, time_between_reports, EVT_GRP_RPT_RAW_MAGNETOMETER_BIT);
}

/**
 * @brief Sends command to disable rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_rotation_vector()
{
    disable_report(SENSOR_REPORT_ID_ROTATION_VECTOR, EVT_GRP_RPT_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to disable game rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_game_rotation_vector()
{
    disable_report(SENSOR_REPORT_ID_GAME_ROTATION_VECTOR, EVT_GRP_RPT_GAME_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to disable ARVR stabilized rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_ARVR_stabilized_rotation_vector()
{
    disable_report(SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR, EVT_GRP_RPT_ARVR_S_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to disable ARVR stabilized game rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_ARVR_stabilized_game_rotation_vector()
{
    disable_report(SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR, EVT_GRP_RPT_ARVR_S_GAME_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to disable gyro integrated rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_gyro_integrated_rotation_vector()
{
    disable_report(SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR, EVT_GRP_RPT_GYRO_ROTATION_VECTOR_BIT);
}

/**
 * @brief Sends command to disable accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_accelerometer()
{
    disable_report(SENSOR_REPORT_ID_ACCELEROMETER, EVT_GRP_RPT_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to disable linear accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_linear_accelerometer()
{
    disable_report(SENSOR_REPORT_ID_LINEAR_ACCELERATION, EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to disable gravity reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_gravity()
{
    disable_report(SENSOR_REPORT_ID_GRAVITY, EVT_GRP_RPT_GRAVITY_BIT);
}

/**
 * @brief Sends command to disable calibrated gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_calibrated_gyro()
{
    disable_report(SENSOR_REPORT_ID_GYROSCOPE, EVT_GRP_RPT_GYRO_BIT);
}

/**
 * @brief Sends command to disable uncalibrated gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_uncalibrated_gyro()
{
    disable_report(SENSOR_REPORT_ID_UNCALIBRATED_GYRO, EVT_GRP_RPT_GYRO_UNCALIBRATED_BIT);
}

/**
 * @brief Sends command to disable magnetometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_magnetometer()
{
    disable_report(SENSOR_REPORT_ID_MAGNETIC_FIELD, EVT_GRP_RPT_MAGNETOMETER_BIT);
}

/**
 * @brief Sends command to disable tap detector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_tap_detector()
{
    disable_report(SENSOR_REPORT_ID_TAP_DETECTOR, EVT_GRP_RPT_TAP_DETECTOR_BIT);
}

/**
 * @brief Sends command to disable step counter reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_step_counter()
{
    disable_report(SENSOR_REPORT_ID_STEP_COUNTER, EVT_GRP_RPT_STEP_COUNTER_BIT);
}

/**
 * @brief Sends command to disable stability reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_stability_classifier()
{
    disable_report(SENSOR_REPORT_ID_STABILITY_CLASSIFIER, EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT);
}

/**
 * @brief Sends command to disable activity classifier reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_activity_classifier()
{
    activity_confidences = nullptr;
    disable_report(SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER, EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT);
}

/**
 * @brief Sends command to disable raw accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_mems_accelerometer()
{
    disable_report(SENSOR_REPORT_ID_RAW_ACCELEROMETER, EVT_GRP_RPT_RAW_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to disable raw gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_mems_gyro()
{
    disable_report(SENSOR_REPORT_ID_RAW_GYROSCOPE, EVT_GRP_RPT_RAW_GYRO_BIT);
}

/**
 * @brief Sends command to disable raw magnetometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_mems_magnetometer()
{
    disable_report(SENSOR_REPORT_ID_RAW_MAGNETOMETER, EVT_GRP_RPT_RAW_MAGNETOMETER_BIT);
}

/**
 * @brief Sends command to tare an axis (See Ref. Manual 6.4.4.1)
 *
 * @param axis_sel Which axes to zero, can be TARE_AXIS_ALL (all axes) or TARE_AXIS_Z (only yaw)
 * @param rotation_vector_basis Which rotation vector type to zero axes can be TARE_ROTATION_VECTOR,
 * TARE_GAME_ROTATION_VECTOR, TARE_GEOMAGNETIC_ROTATION_VECTOR, etc..
 * @return void, nothing to return
 */
void BNO08x::tare_now(uint8_t axis_sel, uint8_t rotation_vector_basis)
{
    queue_tare_command(TARE_NOW, axis_sel, rotation_vector_basis);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to save tare into non-volatile memory of BNO08x (See Ref. Manual 6.4.4.2)
 *
 * @return void, nothing to return
 */
void BNO08x::save_tare()
{
    queue_tare_command(TARE_PERSIST);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to clear persistent tare settings in non-volatile memory of BNO08x (See Ref. Manual 6.4.4.3)
 *
 * @return void, nothing to return
 */
void BNO08x::clear_tare()
{
    queue_tare_command(TARE_SET_REORIENTATION);
    wait_for_tx_done();                 // wait for transmit operation to complete
    vTaskDelay(CMD_EXECUTION_DELAY_MS); // allow some time for command to be executed
}

/**
 * @brief Converts a register value to a float using its associated Q point. (See
 * https://en.wikipedia.org/wiki/Q_(number_format))
 *
 * @param q_point Q point value associated with register.
 * @param fixed_point_value The fixed point value to convert.
 *
 * @return void, nothing to return
 */
float BNO08x::q_to_float(int16_t fixed_point_value, uint8_t q_point)
{
    float q_float = fixed_point_value;
    q_float *= pow(2, q_point * -1);
    return (q_float);
}

/**
 * @brief Return timestamp of most recent report.
 *
 * @return uint32_t containing the timestamp of the most recent report in microseconds.
 */
uint32_t BNO08x::get_time_stamp()
{
    return time_stamp;
}

/**
 * @brief Resets all data returned by public getter APIs to initial values of 0 and low accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::reset_all_data_to_defaults()
{
    time_stamp = 0UL;

    raw_accel_X = 0U;
    raw_accel_Y = 0U;
    raw_accel_Z = 0U;
    accel_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

    raw_lin_accel_X = 0U;
    raw_lin_accel_Y = 0U;
    raw_lin_accel_Z = 0U;
    accel_lin_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

    raw_calib_gyro_X = 0U;
    raw_calib_gyro_Y = 0U;
    raw_calib_gyro_Z = 0U;

    // reset quaternion to nan
    raw_quat_I = 0U;
    raw_quat_J = 0U;
    raw_quat_K = 0U;
    raw_quat_real = 0U;
    raw_quat_radian_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);
    quat_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

    integrated_gyro_velocity_X = 0U;
    integrated_gyro_velocity_Y = 0U;
    integrated_gyro_velocity_Z = 0U;

    gravity_X = 0U;
    gravity_Y = 0U;
    gravity_Z = 0U;
    gravity_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

    raw_uncalib_gyro_X = 0U;
    raw_uncalib_gyro_Y = 0U;
    raw_uncalib_gyro_Z = 0U;
    raw_bias_X = 0U;
    raw_bias_Y = 0U;
    raw_bias_Z = 0U;

    raw_magf_X = 0U;
    raw_magf_Y = 0U;
    raw_magf_Z = 0U;
    magf_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

    tap_detector = 0U;
    step_count = 0U;
    stability_classifier = 0U;
    activity_classifier = 0U;

    mems_raw_accel_X = 0U;
    mems_raw_accel_Y = 0U;
    mems_raw_accel_Z = 0U;

    mems_raw_gyro_X = 0U;
    mems_raw_gyro_Y = 0U;
    mems_raw_gyro_Z = 0U;

    mems_raw_magf_X = 0U;
    mems_raw_magf_Y = 0U;
    mems_raw_magf_Z = 0U;
}

/**
 * @brief Get the full magnetic field vector.
 *
 * @param x Reference variable to save reported x magnitude.
 * @param y Reference variable to save reported y magnitude.
 * @param x Reference variable to save reported z magnitude.
 * @param accuracy Reference variable to save reported accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_magf(float& x, float& y, float& z, BNO08xAccuracy& accuracy)
{
    x = q_to_float(raw_magf_X, MAGNETOMETER_Q1);
    y = q_to_float(raw_magf_Y, MAGNETOMETER_Q1);
    z = q_to_float(raw_magf_Z, MAGNETOMETER_Q1);
    accuracy = static_cast<BNO08xAccuracy>(magf_accuracy);
}

/**
 * @brief Get X component of magnetic field vector.
 *
 * @return The reported X component of magnetic field vector.
 */
float BNO08x::get_magf_X()
{
    float mag = q_to_float(raw_magf_X, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get Y component of magnetic field vector.
 *
 * @return The reported Y component of magnetic field vector.
 */
float BNO08x::get_magf_Y()
{
    float mag = q_to_float(raw_magf_Y, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get Z component of magnetic field vector.
 *
 * @return The reported Z component of magnetic field vector.
 */
float BNO08x::get_magf_Z()
{
    float mag = q_to_float(raw_magf_Z, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get accuracy of reported magnetic field vector.
 *
 * @return The accuracy of reported magnetic field vector.
 */
BNO08xAccuracy BNO08x::get_magf_accuracy()
{
    return static_cast<BNO08xAccuracy>(magf_accuracy);
}

/**
 * @brief Get full reported gravity vector, units in m/s^2
 *
 * @param x Reference variable to save X axis gravity.
 * @param y Reference variable to save Y axis axis gravity.
 * @param z Reference variable to save Z axis axis gravity.
 * @param accuracy Reference variable to save reported gravity accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_gravity(float& x, float& y, float& z, BNO08xAccuracy& accuracy)
{
    x = q_to_float(gravity_X, GRAVITY_Q1);
    y = q_to_float(gravity_Y, GRAVITY_Q1);
    z = q_to_float(gravity_Z, GRAVITY_Q1);
    accuracy = static_cast<BNO08xAccuracy>(gravity_accuracy);
}

/**
 * @brief Get the reported x axis gravity.
 *
 * @return x axis gravity in m/s^2
 */
float BNO08x::get_gravity_X()
{
    return q_to_float(gravity_X, GRAVITY_Q1);
}

/**
 * @brief Get the reported y axis gravity.
 *
 * @return y axis gravity in m/s^2
 */
float BNO08x::get_gravity_Y()
{
    return q_to_float(gravity_Y, GRAVITY_Q1);
}

/**
 * @brief Get the reported z axis gravity.
 *
 * @return z axis gravity in m/s^2
 */
float BNO08x::get_gravity_Z()
{
    return q_to_float(gravity_Z, GRAVITY_Q1);
}

/**
 * @brief Get the reported gravity accuracy.
 *
 * @return Accuracy of reported gravity.
 */
BNO08xAccuracy BNO08x::get_gravity_accuracy()
{
    return static_cast<BNO08xAccuracy>(gravity_accuracy);
}

/**
 * @brief Get the reported rotation about x axis.
 *
 * @return Rotation about the x axis in radians.
 */
float BNO08x::get_roll()
{
    float t_0 = 0.0;
    float t_1 = 0.0;
    float dq_w = get_quat_real();
    float dq_x = get_quat_I();
    float dq_y = get_quat_J();
    float dq_z = get_quat_K();

    float norm = sqrt(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);
    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // roll (x-axis rotation)
    t_0 = 2.0 * (dq_w * dq_x + dq_y * dq_z);
    t_1 = 1.0 - (2.0 * ((dq_x * dq_x) + (dq_y * dq_y)));

    return atan2(t_0, t_1);
}

/**
 * @brief Get the reported rotation about y axis.
 *
 * @return Rotation about the y axis in radians.
 */
float BNO08x::get_pitch()
{
    float t_2 = 0.0;
    float dq_w = get_quat_real();
    float dq_x = get_quat_I();
    float dq_y = get_quat_J();
    float dq_z = get_quat_K();
    float norm = sqrt(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);

    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    t_2 = 2.0 * ((dq_w * dq_y) - (dq_z * dq_x));
    t_2 = t_2 > 1.0 ? 1.0 : t_2;
    t_2 = t_2 < -1.0 ? -1.0 : t_2;

    return asin(t_2);
}

/**
 * @brief Get the reported rotation about z axis.
 *
 * @return Rotation about the z axis in radians.
 */
float BNO08x::get_yaw()
{
    float t_3 = 0.0;
    float t_4 = 0.0;
    float dq_w = get_quat_real();
    float dq_x = get_quat_I();
    float dq_y = get_quat_J();
    float dq_z = get_quat_K();
    float norm = sqrt(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);

    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // yaw (z-axis rotation)
    t_3 = 2.0 * ((dq_w * dq_z) + (dq_x * dq_y));
    t_4 = 1.0 - (2.0 * ((dq_y * dq_y) + (dq_z * dq_z)));

    return atan2(t_3, t_4);
}

/**
 * @brief Get the reported rotation about x axis.
 *
 * @return Rotation about the x axis in degrees.
 */
float BNO08x::get_roll_deg()
{
    return get_roll() * (180.0 / M_PI);
}

/**
 * @brief Get the reported rotation about y axis.
 *
 * @return Rotation about the y axis in degrees.
 */
float BNO08x::get_pitch_deg()
{
    return get_pitch() * (180.0 / M_PI);
}

/**
 * @brief Get the reported rotation about z axis.
 *
 * @return Rotation about the z axis in degrees.
 */
float BNO08x::get_yaw_deg()
{
    return get_yaw() * (180.0 / M_PI);
}

/**
 * @brief Get the full quaternion reading.
 *
 * @param i Reference variable to save reported i component of quaternion.
 * @param j Reference variable to save reported j component of quaternion.
 * @param k Reference variable to save reported k component of quaternion.
 * @param real Reference variable to save reported real component of quaternion.
 * @param rad_accuracy Reference variable to save reported raw quaternion radian accuracy.
 * @param accuracy Reference variable to save reported quaternion accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_quat(float& i, float& j, float& k, float& real, float& rad_accuracy, BNO08xAccuracy& accuracy)
{
    i = q_to_float(raw_quat_I, ROTATION_VECTOR_Q1);
    j = q_to_float(raw_quat_J, ROTATION_VECTOR_Q1);
    k = q_to_float(raw_quat_K, ROTATION_VECTOR_Q1);
    real = q_to_float(raw_quat_real, ROTATION_VECTOR_Q1);
    rad_accuracy = q_to_float(raw_quat_radian_accuracy, ROTATION_VECTOR_Q1);
    accuracy = static_cast<BNO08xAccuracy>(quat_accuracy);
}

/**
 * @brief Get I component of reported quaternion.
 *
 * @return The I component of reported quaternion.
 */
float BNO08x::get_quat_I()
{
    float quat = q_to_float(raw_quat_I, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get J component of reported quaternion.
 *
 * @return The J component of reported quaternion.
 */
float BNO08x::get_quat_J()
{
    float quat = q_to_float(raw_quat_J, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get K component of reported quaternion.
 *
 * @return The K component of reported quaternion.
 */
float BNO08x::get_quat_K()
{
    float quat = q_to_float(raw_quat_K, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get real component of reported quaternion.
 *
 * @return The real component of reported quaternion.
 */
float BNO08x::get_quat_real()
{
    float quat = q_to_float(raw_quat_real, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get radian accuracy of reported quaternion.
 *
 * @return The radian accuracy of reported quaternion.
 */
float BNO08x::get_quat_radian_accuracy()
{
    float quat = q_to_float(raw_quat_radian_accuracy, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get accuracy of reported quaternion.
 *
 * @return The accuracy of reported quaternion.
 */
BNO08xAccuracy BNO08x::get_quat_accuracy()
{
    return static_cast<BNO08xAccuracy>(quat_accuracy);
}

/**
 * @brief Get full acceleration (total acceleration of device, units in m/s^2).
 *
 * @param x Reference variable to save X axis acceleration.
 * @param y Reference variable to save Y axis acceleration.
 * @param z Reference variable to save Z axis acceleration.
 * @param accuracy Reference variable to save reported acceleration accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_accel(float& x, float& y, float& z, BNO08xAccuracy& accuracy)
{
    x = q_to_float(raw_accel_X, ACCELEROMETER_Q1);
    y = q_to_float(raw_accel_Y, ACCELEROMETER_Q1);
    z = q_to_float(raw_accel_Z, ACCELEROMETER_Q1);
    accuracy = static_cast<BNO08xAccuracy>(accel_accuracy);
}

/**
 * @brief Get x axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported x axis acceleration.
 */
float BNO08x::get_accel_X()
{
    return q_to_float(raw_accel_X, ACCELEROMETER_Q1);
}

/**
 * @brief Get y axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported y axis acceleration.
 */
float BNO08x::get_accel_Y()
{
    return q_to_float(raw_accel_Y, ACCELEROMETER_Q1);
}

/**
 * @brief Get z axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported z axis acceleration.
 */
float BNO08x::get_accel_Z()
{
    return q_to_float(raw_accel_Z, ACCELEROMETER_Q1);
}

/**
 * @brief Get accuracy of linear acceleration.
 *
 * @return Accuracy of linear acceleration.
 */
BNO08xAccuracy BNO08x::get_accel_accuracy()
{
    return static_cast<BNO08xAccuracy>(accel_accuracy);
}

/**
 * @brief Get full linear acceleration (acceleration of the device minus gravity, units in m/s^2).
 *
 * @param x Reference variable to save X axis acceleration.
 * @param y Reference variable to save Y axis acceleration.
 * @param z Reference variable to save Z axis acceleration.
 * @param accuracy Reference variable to save reported linear acceleration accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_linear_accel(float& x, float& y, float& z, BNO08xAccuracy& accuracy)
{
    x = q_to_float(raw_lin_accel_X, LINEAR_ACCELEROMETER_Q1);
    y = q_to_float(raw_lin_accel_Y, LINEAR_ACCELEROMETER_Q1);
    z = q_to_float(raw_lin_accel_Z, LINEAR_ACCELEROMETER_Q1);
    accuracy = static_cast<BNO08xAccuracy>(accel_lin_accuracy);
}

/**
 * @brief Get x axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported x axis linear acceleration.
 */
float BNO08x::get_linear_accel_X()
{
    return q_to_float(raw_lin_accel_X, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get y axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported y axis linear acceleration.
 */
float BNO08x::get_linear_accel_Y()
{
    return q_to_float(raw_lin_accel_Y, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get z axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported z axis linear acceleration.
 */
float BNO08x::get_linear_accel_Z()
{
    return q_to_float(raw_lin_accel_Z, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get accuracy of linear acceleration.
 *
 * @return Accuracy of linear acceleration.
 */
BNO08xAccuracy BNO08x::get_linear_accel_accuracy()
{
    return static_cast<BNO08xAccuracy>(accel_lin_accuracy);
}

/**
 * @brief Get full raw acceleration from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8).
 *
 * @param x Reference variable to save raw X axis acceleration.
 * @param y Reference variable to save raw Y axis acceleration.
 * @param z Reference variable to save raw Z axis acceleration.
 *
 * @return void, nothing to return
 */
void BNO08x::get_raw_mems_accel(uint16_t& x, uint16_t& y, uint16_t& z)
{
    x = mems_raw_accel_X;
    y = mems_raw_accel_X;
    z = mems_raw_accel_X;
}

/**
 * @brief Get raw accelerometer x axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer x axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_accel_X()
{
    return mems_raw_accel_X;
}

/**
 * @brief Get raw accelerometer y axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer y axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_accel_Y()
{
    return mems_raw_accel_Y;
}

/**
 * @brief Get raw accelerometer z axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer z axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_accel_Z()
{
    return mems_raw_accel_Z;
}

/**
 * @brief Get raw gyroscope full reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @param x Reference variable to save raw X axis data.
 * @param y Reference variable to save raw Y axis data.
 * @param z Reference variable to save raw Z axis data.
 *
 * @return void, nothing to return
 */
void BNO08x::get_raw_mems_gyro(uint16_t& x, uint16_t& y, uint16_t& z)
{
    x = mems_raw_gyro_X;
    y = mems_raw_gyro_Y;
    z = mems_raw_gyro_Z;
}

/**
 * @brief Get raw gyroscope x axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope x axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_gyro_X()
{
    return mems_raw_gyro_X;
}

/**
 * @brief Get raw gyroscope y axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope y axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_gyro_Y()
{
    return mems_raw_gyro_Y;
}

/**
 * @brief Get raw gyroscope z axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope z axis reading from physical MEMs sensor.
 */
uint16_t BNO08x::get_raw_mems_gyro_Z()
{
    return mems_raw_gyro_Z;
}

/**
 * @brief Get raw magnetometer full reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @param x Reference variable to save raw X axis data.
 * @param y Reference variable to save raw Y axis data.
 * @param z Reference variable to save raw Z axis data.
 *
 * @return void, nothing to return
 */
void BNO08x::get_raw_mems_magf(uint16_t& x, uint16_t& y, uint16_t& z)
{
    x = mems_raw_magf_X;
    y = mems_raw_magf_Y;
    z = mems_raw_magf_Z;
}

/**
 * @brief Get raw magnetometer x axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer x axis reading from physical magnetometer sensor.
 */
uint16_t BNO08x::get_raw_mems_magf_X()
{
    return mems_raw_magf_X;
}

/**
 * @brief Get raw magnetometer y axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer y axis reading from physical magnetometer sensor.
 */
uint16_t BNO08x::get_raw_mems_magf_Y()
{
    return mems_raw_magf_Y;
}

/**
 * @brief Get raw magnetometer z axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer z axis reading from physical magnetometer sensor.
 */
uint16_t BNO08x::get_raw_mems_magf_Z()
{
    return mems_raw_magf_Z;
}

/**
 * @brief Get full rotational velocity with drift compensation (units in Rad/s).
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 *
 * @return void, nothing to return
 */
void BNO08x::get_calibrated_gyro_velocity(float& x, float& y, float& z)
{
    x = q_to_float(raw_calib_gyro_X, GYRO_Q1);
    y = q_to_float(raw_calib_gyro_Y, GYRO_Q1);
    z = q_to_float(raw_calib_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_calibrated_gyro_velocity_X()
{
    return q_to_float(raw_calib_gyro_X, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro y axis angular velocity measurement.
 *
 * @return The angular reported y axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_calibrated_gyro_velocity_Y()
{
    return q_to_float(raw_calib_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro z axis angular velocity measurement.
 *
 * @return The angular reported z axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_calibrated_gyro_velocity_Z()
{
    return q_to_float(raw_calib_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get full rotational velocity without drift compensation (units in Rad/s). An estimate of drift is given but
 * not applied.
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 * @param b_x Reference variable to save X axis drift estimate
 * @param b_y Reference variable to save Y axis drift estimate
 * @param b_z Reference variable to save Z axis drift estimate
 *
 * @return void, nothing to return
 */
void BNO08x::get_uncalibrated_gyro_velocity(float& x, float& y, float& z, float& b_x, float& b_y, float& b_z)
{
    x = q_to_float(raw_uncalib_gyro_X, GYRO_Q1);
    y = q_to_float(raw_uncalib_gyro_Y, GYRO_Q1);
    z = q_to_float(raw_uncalib_gyro_Z, GYRO_Q1);
    b_x = q_to_float(raw_bias_X, GYRO_Q1);
    b_y = q_to_float(raw_bias_Y, GYRO_Q1);
    b_z = q_to_float(raw_bias_Z, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_velocity_X()
{
    return q_to_float(raw_uncalib_gyro_X, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Y axis angular velocity measurement.
 *
 * @return The angular reported Y axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_velocity_Y()
{
    return q_to_float(raw_uncalib_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Z axis angular velocity measurement.
 *
 * @return The angular reported Z axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_velocity_Z()
{
    return q_to_float(raw_uncalib_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro x axis drift estimate.
 *
 * @return The angular reported x axis drift estimate.
 */
float BNO08x::get_uncalibrated_gyro_bias_X()
{
    return q_to_float(raw_bias_X, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Y axis drift estimate.
 *
 * @return The angular reported Y axis drift estimate.
 */
float BNO08x::get_uncalibrated_gyro_bias_Y()
{
    return q_to_float(raw_bias_Y, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Z axis drift estimate.
 *
 * @return The angular reported Z axis drift estimate.
 */
float BNO08x::get_uncalibrated_gyro_bias_Z()
{
    return q_to_float(raw_bias_Z, GYRO_Q1);
}

/**
 * @brief Full rotational velocity from gyro-integrated rotation vector (See Ref. Manual 6.5.44)
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 *
 * @return void, nothing to return
 */
void BNO08x::get_integrated_gyro_velocity(float& x, float& y, float& z)
{
    x = q_to_float(integrated_gyro_velocity_X, ANGULAR_VELOCITY_Q1);
    y = q_to_float(integrated_gyro_velocity_Y, ANGULAR_VELOCITY_Q1);
    z = q_to_float(integrated_gyro_velocity_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get x axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported x axis angular velocity.
 */
float BNO08x::get_integrated_gyro_velocity_X()
{
    return q_to_float(integrated_gyro_velocity_X, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get y axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported y axis angular velocity.
 */
float BNO08x::get_integrated_gyro_velocity_Y()
{
    return q_to_float(integrated_gyro_velocity_Y, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get z axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported Z axis angular velocity.
 */
float BNO08x::get_integrated_gyro_velocity_Z()
{
    return q_to_float(integrated_gyro_velocity_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get if tap has occured.
 *
 * @return 7 bit tap code indicated which axis taps have occurred. (See Ref. Manual 6.5.27)
 */
uint8_t BNO08x::get_tap_detector()
{
    return tap_detector;
}

/**
 * @brief Get the counted amount of steps.
 *
 * @return The current amount of counted steps.
 */
uint16_t BNO08x::get_step_count()
{
    return step_count;
}

/**
 * @brief Get the current stability classifier (Seee Ref. Manual 6.5.31)
 *
 * @return The current stability (0 = unknown, 1 = on table, 2 = stationary)
 */
Stability BNO08x::get_stability_classifier()
{
    return static_cast<Stability>(stability_classifier);
}

/**
 * @brief Get the current activity classifier (Seee Ref. Manual 6.5.36)
 *
 * @return The current activity:
 *         0 = unknown
 *         1 = in vehicle
 *         2 = on bicycle
 *         3 = on foot
 *         4 = still
 *         5 = tilting
 *         6 = walking
 *         7 = runnning
 *         8 = on stairs
 */
Activity BNO08x::get_activity_classifier()
{
    return static_cast<Activity>(activity_classifier);
}

/**
 * @brief Prints the header of the passed SHTP packet to serial console with ESP_LOG statement.
 *
 * @param packet The packet containing the header to be printed.
 * @return void, nothing to return
 */
void BNO08x::print_header(bno08x_rx_packet_t* packet)
{
    // print most recent header
    ESP_LOGI(TAG,
            "SHTP Header:\n\r"
            "                       Raw 32 bit word: 0x%02X%02X%02X%02X\n\r"
            "                       Packet Length:   %d\n\r"
            "                       Channel Number:  %d\n\r"
            "                       Sequence Number: %d\n\r"
            "                       Channel Type: %s\n\r",
            (int) packet->header[0], (int) packet->header[1], (int) packet->header[2], (int) packet->header[3], (int) (packet->length + 4),
            (int) packet->header[2], (int) packet->header[3],
            (packet->header[2] == 0)   ? "Command"
            : (packet->header[2] == 1) ? "Executable"
            : (packet->header[2] == 2) ? "Control"
            : (packet->header[2] == 3) ? "Sensor-report"
            : (packet->header[2] == 4) ? "Wake-report"
            : (packet->header[2] == 5) ? "Gyro-vector"
                                       : "Unknown");
}

/**
 * @brief Prints the passed SHTP packet to serial console with ESP_LOG statement.
 *
 * @param packet The packet to be printed.
 * @return void, nothing to return
 */
void BNO08x::print_packet(bno08x_rx_packet_t* packet)
{
    uint8_t i = 0;
    uint16_t print_length = 0;
    char packet_string[600];
    char byte_string[8];

    if (packet->length > 40)
        print_length = 40;
    else
        print_length = packet->length;

    sprintf(packet_string, "                 Body: \n\r                       ");
    for (i = 0; i < print_length; i++)
    {
        sprintf(byte_string, " 0x%02X ", packet->body[i]);
        strcat(packet_string, byte_string);

        if ((i + 1) % 6 == 0) // add a newline every 6 bytes
            strcat(packet_string, "\n\r                       ");
    }

    ESP_LOGI(TAG,
            "SHTP Header:\n\r"
            "                       Raw 32 bit word: 0x%02X%02X%02X%02X\n\r"
            "                       Packet Length:   %d\n\r"
            "                       Channel Number:  %d\n\r"
            "                       Sequence Number: %d\n\r"
            "                       Channel Type: %s\n\r"
            "%s",
            (int) packet->header[0], (int) packet->header[1], (int) packet->header[2], (int) packet->header[3], (int) (packet->length + 4),
            (int) packet->header[2], (int) packet->header[3],
            (packet->header[2] == 0)   ? "Command"
            : (packet->header[2] == 1) ? "Executable"
            : (packet->header[2] == 2) ? "Control"
            : (packet->header[2] == 3) ? "Sensor-report"
            : (packet->header[2] == 4) ? "Wake-report"
            : (packet->header[2] == 5) ? "Gyro-vector"
                                       : "Unknown",
            packet_string);
}

/**
 * @brief Gets Q1 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q1 value for.
 *
 * @return Q1 value for requested sensor.
 */
int16_t BNO08x::get_Q1(uint16_t record_ID)
{
    // Q1 is lower 16 bits of word 7
    return static_cast<uint16_t>((FRS_read_word(record_ID, 7) & 0xFFFFUL));
}

/**
 * @brief Gets Q2 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q2 value for.
 *
 * @return Q2 value for requested sensor.
 */
int16_t BNO08x::get_Q2(uint16_t record_ID)
{
    // Q2 is upper 16 bits of word 7
    return static_cast<uint16_t>((FRS_read_word(record_ID, 7) >> 16U));
}

/**
 * @brief Gets Q3 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q3 value for.
 *
 * @return Q3 value for requested sensor.
 */
int16_t BNO08x::get_Q3(uint16_t record_ID)
{
    // Q3 is upper 16 bits of word 8
    return static_cast<uint16_t>((FRS_read_word(record_ID, 8) >> 16U));
}

/**
 * @brief Gets resolution from BNO08x FRS (flash record system).
 *
 * @param record_ID Which record ID/ sensor to get resolution value for.
 *
 * @return The resolution value for the requested sensor.
 */
float BNO08x::get_resolution(uint16_t record_ID)
{
    int16_t Q = get_Q1(record_ID); // use same q as sensor's input report for range calc

    // resolution is word 2
    uint32_t value = FRS_read_word(record_ID, 2);

    return q_to_float(value, Q); // return resolution
}

/**
 * @brief Gets range from BNO08x FRS (flash record system).
 *
 * @param record_ID Which record ID/ sensor to get range value for.
 *
 * @return The range value for the requested sensor.
 */
float BNO08x::get_range(uint16_t record_ID)
{
    int16_t Q = get_Q1(record_ID); // use same q as sensor's input report for range calc

    // resolution is word 1
    uint32_t value = FRS_read_word(record_ID, 1);

    return q_to_float(value, Q); // return range
}

/**
 * @brief Reads meta data word from BNO08x FRS (flash record system) given the record ID and word number. (See Ref.
 * Manual 5.1 & 6.3.7)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param word_number Desired word to read.
 *
 * @return Requested meta data word, 0 if failed.
 */
uint32_t BNO08x::FRS_read_word(uint16_t record_ID, uint8_t word_number)
{
    uint32_t frs_read = 0;

    if (FRS_read_data(record_ID, word_number, 1)) // start at desired word and only read one 1 word
        frs_read = meta_data[0];

    return frs_read;
}

/**
 * @brief Requests meta data from BNO08x FRS (flash record system) given the record ID. Contains Q points and other
 * info. (See Ref. Manual 5.1 & 6.3.6)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param start_location Start byte location.
 * @param words_to_read Length of words to read.
 *
 * @return True if read request acknowledged (HINT was asserted)
 */
bool BNO08x::FRS_read_request(uint16_t record_ID, uint16_t read_offset, uint16_t block_size)
{
    uint8_t commands[20] = {0};

    commands[0] = SHTP_REPORT_FRS_READ_REQUEST; // FRS Read Request
    commands[1] = 0;                            // Reserved
    commands[2] = (read_offset >> 0) & 0xFF;    // Read Offset LSB
    commands[3] = (read_offset >> 8) & 0xFF;    // Read Offset MSB
    commands[4] = (record_ID >> 0) & 0xFF;      // FRS Type LSB
    commands[5] = (record_ID >> 8) & 0xFF;      // FRS Type MSB
    commands[6] = (block_size >> 0) & 0xFF;     // Block size LSB
    commands[7] = (block_size >> 8) & 0xFF;     // Block size MSB

    // Transmit packet on channel 2, 8 bytes
    queue_packet(CHANNEL_CONTROL, 8, commands);
    return wait_for_tx_done();
}

/**
 * @brief Read meta data from BNO08x FRS (flash record system) given the record ID. Contains Q points and other info.
 * (See Ref. Manual 5.1 & 6.3.7)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param start_location Start byte location.
 * @param words_to_read Length of words to read.
 *
 * @return True if meta data read successfully.
 */
bool BNO08x::FRS_read_data(uint16_t record_ID, uint8_t start_location, uint8_t words_to_read)
{
    uint8_t counter;
    uint8_t packet_body[RX_DATA_LENGTH];
    uint8_t data_length;
    uint8_t frs_status;
    uint32_t data_1;
    uint32_t data_2;
    uint8_t spot = 0;

    if (FRS_read_request(record_ID, start_location, words_to_read))
    {
        while (1)
        {
            while (1)
            {
                counter = 0;

                while (!wait_for_rx_done())
                {
                    counter++;

                    if (counter > 100)
                        return false;
                }

                if (xQueueReceive(queue_frs_read_data, &packet_body, host_int_timeout_ms))
                {
                    if (PARSE_FRS_READ_RESPONSE_REPORT_RECORD_ID(packet_body) == record_ID)
                        break;
                }
            }

            data_length = packet_body[1] >> 4;
            frs_status = packet_body[1] & 0x0F;

            data_1 = PARSE_FRS_READ_RESPONSE_REPORT_DATA_1(packet_body);
            data_2 = PARSE_FRS_READ_RESPONSE_REPORT_DATA_2(packet_body);

            if (data_length > 0)
                meta_data[spot++] = data_1;

            if (data_length > 1)
                meta_data[spot++] = data_2;

            if (spot >= MAX_METADATA_LENGTH)
                return true;

            if (frs_status == 3 || frs_status == 6 || frs_status == 7)
                return true;
        }
    }

    return false;
}

/**
 * @brief Queues a packet containing a command with a request for sensor reports, reported periodically. (See Ref.
 * Manual 6.5.4)
 *
 * @param report_ID ID of sensor report to be enabled.
 * @param time_between_reports Desired time between reports in microseconds.
 * @param specific_config Specific config word (used with personal activity classifier)
 *
 * @return void, nothing to return
 */
void BNO08x::queue_feature_command(uint8_t report_ID, uint32_t time_between_reports, uint32_t specific_config)
{
    uint8_t commands[20] = {0};

    commands[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     // Set feature command (See Ref. Manual 6.5.4)
    commands[1] = report_ID;                           // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    commands[2] = 0;                                   // Feature flags
    commands[3] = 0;                                   // Change sensitivity (LSB)
    commands[4] = 0;                                   // Change sensitivity (MSB)
    commands[5] = (time_between_reports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    commands[6] = (time_between_reports >> 8) & 0xFF;  // Report interval
    commands[7] = (time_between_reports >> 16) & 0xFF; // Report interval
    commands[8] = (time_between_reports >> 24) & 0xFF; // Report interval (MSB)
    commands[9] = 0;                                   // Batch Interval (LSB)
    commands[10] = 0;                                  // Batch Interval
    commands[11] = 0;                                  // Batch Interval
    commands[12] = 0;                                  // Batch Interval (MSB)
    commands[13] = (specific_config >> 0) & 0xFF;      // Sensor-specific config (LSB)
    commands[14] = (specific_config >> 8) & 0xFF;      // Sensor-specific config
    commands[15] = (specific_config >> 16) & 0xFF;     // Sensor-specific config
    commands[16] = (specific_config >> 24) & 0xFF;     // Sensor-specific config (MSB)

    // Transmit packet on channel 2, 17 bytes
    queue_packet(CHANNEL_CONTROL, 17, commands);
}

/**
 * @brief Queues a packet containing a command related to zeroing sensor's axes. (See Ref. Manual 6.4.4.1)
 *
 * @param command Tare command to be sent.
 * @param axis Specified axis (can be z or all at once)
 * @param rotation_vector_basis Which rotation vector type to zero axes of, BNO08x saves seperate data for Rotation
 * Vector, Gaming Rotation Vector, etc..)
 *
 * @return void, nothing to return
 */
void BNO08x::queue_tare_command(uint8_t command, uint8_t axis, uint8_t rotation_vector_basis)
{
    uint8_t commands[20] = {0};

    commands[3] = command;

    if (command == TARE_NOW)
    {
        commands[4] = axis;
        commands[5] = rotation_vector_basis;
    }

    queue_command(COMMAND_TARE, commands);
}

/**
 * @brief Static function used to launch spi task.
 *
 * Used such that spi_task() can be non-static class member.
 *
 * @param arg void pointer to BNO08x imu object
 * @return void, nothing to return
 */
void BNO08x::spi_task_trampoline(void* arg)
{
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    imu->spi_task();             // launch spi task from object
}

/**
 * @brief Task responsible for SPI transactions. Executed when HINT in is asserted by BNO08x
 *
 * @return void, nothing to return
 */
void BNO08x::spi_task()
{
    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        static uint64_t prev_time = esp_timer_get_time();
        static uint64_t current_time = 0;
    #endif
    // clang-format on

    bno08x_tx_packet_t tx_packet;

    while (1)
    {
        /*only re-enable interrupts if there are reports enabled, if no reports are enabled
         interrupts will be re-enabled upon the next user call to wait_for_tx_done(), wait_for_rx_done(), or wait_for_data()*/
        if (xEventGroupGetBits(evt_grp_report_en) != 0)
            gpio_intr_enable(imu_config.io_int);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until notified by ISR (hint_handler)

        if (CHECK_TASKS_RUNNING(evt_grp_task_flow, EVT_GRP_TSK_FLW_RUNNING_BIT)) // ensure deconstructor has not requested that task be deleted
        {

            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                current_time = esp_timer_get_time();
                ESP_LOGI(TAG, "HINT asserted, time since last assertion: %llu", (current_time - prev_time));
                prev_time = current_time;
            #endif
            // clang-format on

            if (xQueueReceive(queue_tx_data, &tx_packet, 0)) // check for queued packet to be sent, non blocking
                send_packet(&tx_packet);                     // send packet
            else
                receive_packet(); // receive packet
        }
        else
        {
            // exit loop, deconstructor has requested task be deleted
            break;
        }
    }

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    vTaskDelete(NULL);
}

/**
 * @brief Static function used to launch data processing task.
 *
 * Used such that data_proc_task() can be non-static class member.
 *
 * @param arg void pointer to BNO08x imu object
 * @return void, nothing to return
 */
void BNO08x::data_proc_task_trampoline(void* arg)
{
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    imu->data_proc_task();       // launch data processing task task from object
}

/**
 * @brief Task responsible parsing packets. Executed when SPI task sends a packet to be parsed, notifies wait_for_data() call.
 *
 * @return void, nothing to return
 */
void BNO08x::data_proc_task()
{
    bno08x_rx_packet_t packet;
    bool notify_users = false;

    while (1) // receive packet from spi_task()
    {
        if (xQueueReceive(queue_rx_data, &packet, portMAX_DELAY) == pdTRUE)
        {
            if (CHECK_TASKS_RUNNING(evt_grp_task_flow, EVT_GRP_TSK_FLW_RUNNING_BIT)) // ensure deconstructor has not requested that task be deleted
            {
                if ((parse_packet(&packet, notify_users) != 0)) // check if packet is valid
                {
                    // get feature reports are valid packets but we don't need to notify the user unless they explicitly have requested them
                    if (notify_users)
                    {
                        // execute any registered callbacks
                        for (auto& cb_fxn : cb_list)
                            cb_fxn();

                        xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET_BIT); // indicate valid packet to wait_for_data()
                    }
                }
                else
                {
                    // clang-format off
                    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                        print_packet(&packet);
                    #endif
                    // clang-format on

                    xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_INVALID_PACKET_BIT); // indicated invalid packet to wait_for_data()
                }
            }
            else
            {
                // exit loop, deconstructor has requested task be deleted
                break;
            }
        }
    }

    // self delete task
    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor task deletion is completed
    vTaskDelete(NULL);
}

/**
 * @brief Launches spi_task and data_proc_task on constructor call.
 *
 * @return ESP_OK if tasks successfully created.
 */
esp_err_t BNO08x::launch_tasks()
{
    BaseType_t task_created = pdFALSE;

    xEventGroupSetBits(evt_grp_task_flow, EVT_GRP_TSK_FLW_RUNNING_BIT); // set task flow to running

    // launch data processing task
    task_created = xTaskCreate(
            &data_proc_task_trampoline, "bno08x_data_processing_task", CONFIG_ESP32_BNO08X_DATA_PROC_TASK_SZ, this, 7, &data_proc_task_hdl);

    if (task_created != pdTRUE)
    {
        ESP_LOGE(TAG, "Initialization failed, data_proc_task failed to launch.");
        return ESP_ERR_INVALID_STATE;
    }
    else
    {
        init_status.data_proc_task = true;
        init_status.task_count++;
    }

    task_created = xTaskCreate(&spi_task_trampoline, "bno08x_spi_task", 4096, this, 8, &spi_task_hdl); // launch SPI task

    if (task_created != pdTRUE)
    {
        ESP_LOGE(TAG, "Initialization failed, spi_task failed to launch.");
        return ESP_ERR_INVALID_STATE;
    }
    else
    {
        init_status.spi_task = true;
        init_status.task_count++;
    }

    return ESP_OK;
}

/**
 * @brief Deletes spi_task and data_proc_task safely on deconstructor call.
 *
 * @return ESP_OK if tasks successfully deleted.
 */
esp_err_t BNO08x::kill_all_tasks()
{
    static const constexpr uint8_t TASK_DELETE_TIMEOUT_MS = 10;
    uint8_t kill_count = 0;
    bno08x_rx_packet_t dummy_packet;
    sem_kill_tasks = xSemaphoreCreateCounting(init_status.task_count, 0);

    memset(&dummy_packet, 0, sizeof(bno08x_rx_packet_t));

    xEventGroupClearBits(
            evt_grp_task_flow, EVT_GRP_TSK_FLW_RUNNING_BIT); // clear task running bit in task flow event group to request deletion of tasks

    if (init_status.task_count != 0)
    {
        if (init_status.spi_task)
            xTaskNotifyGive(spi_task_hdl); // notify spi task for self deletion

        if (init_status.data_proc_task)
            xQueueSend(queue_rx_data, &dummy_packet, 0); // send a dummy packet to wake up data_proc task for self-deletion

        for (uint8_t i = 0; i < init_status.task_count; i++)
            if (xSemaphoreTake(sem_kill_tasks, TASK_DELETE_TIMEOUT_MS / portTICK_PERIOD_MS) == pdTRUE)
                kill_count++;

        if (kill_count != init_status.task_count)
        {
            ESP_LOGE(TAG, "Task deletion timed out in deconstructor call.");
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

/**
 * @brief Updates period of respective report in report_period_trackers and recalculates host_int_timeout_ms according to next longest report period.
 *
 *
 * @param report_ID report ID to update period of.
 * @return void, nothing to return
 */
void BNO08x::update_report_period_trackers(uint8_t report_ID, uint32_t new_period)
{
    uint8_t idx = report_ID_to_report_period_tracker_idx(report_ID);

    if (idx != REPORT_CNT)
    {
        report_period_trackers[idx].period = new_period;

        if (new_period > largest_sample_period_us)
        {
            current_slowest_report_ID = report_ID;
            largest_sample_period_us = new_period;
            host_int_timeout_ms = (2 * (largest_sample_period_us / 1000UL)) / portTICK_PERIOD_MS;

            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                ESP_LOGW(TAG, "new hint timeout: %d", static_cast<uint16_t>(host_int_timeout_ms));
            #endif
            // clang-format on
        }
        else
        {
            if (current_slowest_report_ID == report_ID)
            {
                largest_sample_period_us = 0;

                // no longer true, find the slowest
                for (int i = 0; i < REPORT_CNT; i++)
                {
                    if (report_period_trackers[i].period > largest_sample_period_us)
                    {
                        current_slowest_report_ID = report_period_trackers[i].report_ID;
                        largest_sample_period_us = report_period_trackers[i].period;
                        host_int_timeout_ms = (2 * (largest_sample_period_us / 1000UL)) / portTICK_PERIOD_MS;

                        // clang-format off
                        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                            ESP_LOGW(TAG, "new hint timeout (due to period tracker search): %d", static_cast<uint16_t>(host_int_timeout_ms));
                        #endif
                        // clang-format on
                    }
                }
            }
        }
    }
}

/**
 * @brief Converts report id to respective index in report_period_trackers.
 *
 *
 * @param report_ID report ID to return index for.
 * @return Index in report_period_trackers corresponding to passed report ID.
 */
uint8_t BNO08x::report_ID_to_report_period_tracker_idx(uint8_t report_ID)
{
    switch (report_ID)
    {
        case SENSOR_REPORT_ID_ACCELEROMETER:
            return 0;
        case SENSOR_REPORT_ID_GYROSCOPE:
            return 1;
        case SENSOR_REPORT_ID_MAGNETIC_FIELD:
            return 2;
        case SENSOR_REPORT_ID_LINEAR_ACCELERATION:
            return 3;
        case SENSOR_REPORT_ID_ROTATION_VECTOR:
            return 4;
        case SENSOR_REPORT_ID_GRAVITY:
            return 5;
        case SENSOR_REPORT_ID_UNCALIBRATED_GYRO:
            return 6;
        case SENSOR_REPORT_ID_GAME_ROTATION_VECTOR:
            return 7;
        case SENSOR_REPORT_ID_GEOMAGNETIC_ROTATION_VECTOR:
            return 8;
        case SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR:
            return 9;
        case SENSOR_REPORT_ID_TAP_DETECTOR:
            return 10;
        case SENSOR_REPORT_ID_STEP_COUNTER:
            return 11;
        case SENSOR_REPORT_ID_STABILITY_CLASSIFIER:
            return 12;
        case SENSOR_REPORT_ID_RAW_ACCELEROMETER:
            return 13;
        case SENSOR_REPORT_ID_RAW_GYROSCOPE:
            return 14;
        case SENSOR_REPORT_ID_RAW_MAGNETOMETER:
            return 15;
        case SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER:
            return 16;
        case SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR:
            return 17;
        case SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR:
            return 18;
        default:
            return 19;
    }
}

/**
 * @brief HINT interrupt service routine, handles falling edge of BNO08x HINT pin.
 *
 * ISR that launches SPI task to perform transaction upon assertion of BNO08x interrupt pin.
 *
 * @return void, nothing to return
 */
void IRAM_ATTR BNO08x::hint_handler(void* arg)
{
    BaseType_t xHighPriorityTaskWoken = pdFALSE;
    BNO08x* imu = (BNO08x*) arg; // cast argument received by gpio_isr_handler_add ("this" pointer to imu object
                                 // created by constructor call)

    gpio_intr_disable(imu->imu_config.io_int);                          // disable interrupts
    vTaskNotifyGiveFromISR(imu->spi_task_hdl, &xHighPriorityTaskWoken); // notify SPI task BNO08x is ready for
                                                                        // servicing
    portYIELD_FROM_ISR(xHighPriorityTaskWoken);                         // perform context switch if necessary
}
