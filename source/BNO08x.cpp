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
    , queue_rx_data(xQueueCreate(1, sizeof(sh2_packet_t)))
    , queue_tx_data(xQueueCreate(1, sizeof(sh2_packet_t)))
    , imu_config(imu_config)
{
}

/**
 * @brief BNO08x imu deconstructor.
 *
 * Deconstructs a BNO08x object and releases any utilized resources.
 *
 * @return void, nothing to return.
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
 * @return True if initialization was success, false if otherwise.
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

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    ESP_LOGI(TAG, "Successfully initialized....");
    #endif
    // clang-format on

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, CS GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_miso == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, MISO GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_mosi == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, MOSI GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_sclk == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, SCLK GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_rst == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "RST GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Max SPI clock speed exceeded, %ld overwritten with 3MHz", imu_config.sclk_speed);
        #endif
        // clang-format on

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
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to configure HINT gpio.");
        #endif
        // clang-format on
    }
    else
    {
        init_status.gpio_inputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up
    }

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
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to configure CS, RST, and WAKE (if used) gpio.");
        #endif
        // clang-format on
    }
    else
    {
        init_status.gpio_outputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up
    }

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to install global ISR service.");
        #endif
        // clang-format on

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

        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add hint_handler ISR.");
        #endif
        // clang-format on

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, SPI bus failed to initialize.");
        #endif
        // clang-format on

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add device to SPI bus.");
        #endif
        // clang-format on

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
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add device to SPI bus.");
        #endif
        // clang-format on
    }

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
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could reset gpio WAKE pin to default state.");
            #endif
            // clang-format on

            return ret;
        }
    }

    ret = gpio_reset_pin(imu_config.io_cs);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio CS pin to default state.");
        #endif
        // clang-format on

        return ret;
    }

    ret = gpio_reset_pin(imu_config.io_rst);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio RST pin to default state.");
        #endif
        // clang-format on

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
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could not remove hint ISR handler.");
            #endif
            // clang-format on

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
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could not remove spi device.");
            #endif
            // clang-format on

            return ret;
        }
    }

    if (init_status.spi_bus)
    {
        ret = spi_bus_free(imu_config.spi_peripheral);
        if (ret != ESP_OK)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could free SPI peripheral.");
            #endif
            // clang-format on

            return ret;
        }
    }

    return ret;
}

/**
 * @brief Hard resets BNO08x sensor.
 *
 * @return True if reset succeeded.
 */
bool BNO08x::hard_reset()
{
    bool reset_success = false;

    // resetting disables all reports
    xEventGroupClearBits(evt_grp_report_en, EVT_GRP_RPT_ALL_BITS);

    gpio_set_level(imu_config.io_cs, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    gpio_set_level(imu_config.io_rst, 0); // set reset pin low
    vTaskDelay(HARD_RESET_DELAY_MS);      // 10ns min, set to larger delay to let things stabilize(Anton)
    gpio_set_level(imu_config.io_rst, 1); // bring out of reset

    // Receive advertisement message on boot (see SH2 Ref. Manual 5.2 & 5.3)

    return reset_success; 
}


/**
 * @brief Receives/sends a SHTP packet via SPI. Sends any received packets to data_proc_task().
 *
 * @return void, nothing to return
 */
esp_err_t BNO08x::transmit_packet()
{
    static sh2_packet_t rx_packet, tx_packet;
    esp_err_t ret = ESP_OK;

    if (gpio_get_level(imu_config.io_int)) // ensure INT pin is low
        return ESP_ERR_INVALID_STATE;

    gpio_set_level(imu_config.io_cs, 0); // assert chip select

    if (xQueueReceive(queue_tx_data, &tx_packet, 0) == pdFALSE) // check for queued packet to be sent, non blocking
    {
        memset(&tx_packet, 0U, sizeof(sh2_packet_t)); // no queued packet to send, set everything to 0
    }

    // receive/send packet header
    ret = transmit_packet_header(&rx_packet, &tx_packet);
    if (ret != ESP_OK)
    {
        gpio_set_level(imu_config.io_cs, 1); // de-assert chip select
        return ret;
    }

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
    ESP_LOGW(TAG, "packet rx length: %d", rx_packet.length);
    #endif
    // clang-format on

    if (rx_packet.length == 0)
    {
        gpio_set_level(imu_config.io_cs, 1); // de-assert chip select
        return ESP_ERR_INVALID_RESPONSE;
    }

    ret = transmit_packet_body(&rx_packet, &tx_packet);
    if (ret == ESP_OK)
    {
        // tx_packet non-zero length implies one was rx'd through queue
        if (tx_packet.length != 0)
            xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_TX_DONE_BIT);

        xQueueSend(queue_rx_data, &rx_packet, 0); // send received data to data_proc_task
        xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT);
    }

    gpio_set_level(imu_config.io_cs, 1); // de-assert chip select

    return ret;
}

/**
 * @brief Receives/sends a SHTP packet header via SPI.
 *
 * @param rx_packet Pointer to packet to receive header into.
 * @param tx_packet Pointer to packet with header to send.
 *
 * @return ESP_OK if receive was success.
 */
esp_err_t BNO08x::transmit_packet_header(sh2_packet_t* rx_packet, sh2_packet_t* tx_packet)
{

    esp_err_t ret = ESP_OK;

    // setup transaction to send/receive first 4 bytes (packet header)
    spi_transaction.rx_buffer = rx_packet->header;
    spi_transaction.tx_buffer = tx_packet->header;
    spi_transaction.length = 4 * 8;
    spi_transaction.rxlength = 4 * 8;
    spi_transaction.flags = 0;

    ret = spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive first 4 bytes (packet header)

    if (ret == ESP_OK)
    {
        // calculate length of packet from received header
        rx_packet->length = PARSE_PACKET_LENGTH(rx_packet);
        rx_packet->length &= ~(1U << 15U); // clear the MSbit
        rx_packet->length -= 4;            // remove 4 header bytes from rx packet length (we already read those)

        if (tx_packet->length != 0)
            tx_packet->length -= 4; // remove 4 header bytes from tx packet length (we already sent those)
    }

    return ret;
}

/**
 * @brief Receives/sends a SHTP packet body via SPI.
 *
 * @param rx_packet Pointer to packet to save body to.
 * @param packet_tx Pointer to packet with body to send.
 *
 * @return ESP_OK if receive was success.
 */
esp_err_t BNO08x::transmit_packet_body(sh2_packet_t* rx_packet, sh2_packet_t* tx_packet)
{
    esp_err_t ret = ESP_OK;
    const uint16_t transaction_length = (rx_packet->length > tx_packet->length) ? rx_packet->length : tx_packet->length;
    // setup transacton to read the data packet
    spi_transaction.rx_buffer = rx_packet->body;
    spi_transaction.tx_buffer = tx_packet->body;
    spi_transaction.length = transaction_length * 8;
    spi_transaction.rxlength = rx_packet->length * 8;
    spi_transaction.flags = 0;

    ret = spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive rest of packet

    return ret;
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
 * @brief Prints the header of the passed SHTP packet to serial console with ESP_LOG statement.
 *
 * @param packet The packet containing the header to be printed.
 * @return void, nothing to return
 */
void BNO08x::print_header(sh2_packet_t* packet)
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
void BNO08x::print_packet(sh2_packet_t* packet)
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

    while (1)
    {
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

            transmit_packet();
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
    sh2_packet_t packet;

    while (1) // receive packet from spi_task()
    {
        if (xQueueReceive(queue_rx_data, &packet, portMAX_DELAY) == pdTRUE)
        {
            if (CHECK_TASKS_RUNNING(evt_grp_task_flow, EVT_GRP_TSK_FLW_RUNNING_BIT)) // ensure deconstructor has not requested that task be deleted
            {
                // PROCESS RX HERE
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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, data_proc_task failed to launch.");
        #endif
        // clang-format on

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
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, spi_task failed to launch.");
        #endif
        // clang-format on

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
    sh2_packet_t dummy_packet;
    sem_kill_tasks = xSemaphoreCreateCounting(init_status.task_count, 0);

    memset(&dummy_packet, 0, sizeof(sh2_packet_t));

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
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
            ESP_LOGE(TAG, "Task deletion timed out in deconstructor call.");
            #endif
            // clang-format on

            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
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
