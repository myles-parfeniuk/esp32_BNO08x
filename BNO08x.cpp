#include "BNO08x.hpp"

bool BNO08x::isr_service_installed = {false};
bno08x_config_t BNO08x::default_imu_config;

/**
 * @brief BNO08x imu constructor.
 *
 * Construct a BNO08x object for managing a BNO08x sensor.
 * Initializes required GPIO pins, interrupts, SPI peripheral.
 *
 * @param imu_config Configuration settings (optional), default settings can be seen in bno08x_config_t
 * @return void, nothing to return
 */
BNO08x::BNO08x(bno08x_config_t imu_config)
    : evt_grp_spi(xEventGroupCreate())
    , evt_grp_report_en(xEventGroupCreate())
    , queue_rx_data(xQueueCreate(1, sizeof(bno08x_rx_packet_t)))
    , queue_tx_data(xQueueCreate(1, sizeof(bno08x_tx_packet_t)))
    , queue_frs_read_data(xQueueCreate(1, RX_DATA_LENGTH * sizeof(uint8_t)))
    , queue_reset_reason(xQueueCreate(1, sizeof(uint32_t)))
    , imu_config(imu_config)
    , calibration_status(1)

{
    uint8_t tx_buffer[50] = {0};

    // SPI bus config
    bus_config.mosi_io_num = imu_config.io_mosi; // assign mosi gpio pin
    bus_config.miso_io_num = imu_config.io_miso; // assign miso gpio pin
    bus_config.sclk_io_num = imu_config.io_sclk; // assign sclk gpio pin
    bus_config.quadhd_io_num = -1;               // hold signal gpio (not used)
    bus_config.quadwp_io_num = -1;               // write protect signal gpio (not used)

    // SPI slave device specific config
    imu_spi_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus high when idle)

    if (imu_config.sclk_speed > 3000000) // max sclk speed of 3MHz for BNO08x
    {
        ESP_LOGE(TAG, "Max clock speed exceeded, %lld overwritten with 3000000Hz", imu_config.sclk_speed);
        imu_config.sclk_speed = 3000000;
    }

    imu_spi_config.clock_source = SPI_CLK_SRC_DEFAULT;
    imu_spi_config.clock_speed_hz = imu_config.sclk_speed; // assign SCLK speed
    imu_spi_config.address_bits = 0;                       // 0 address bits, not using this system
    imu_spi_config.command_bits = 0;                       // 0 command bits, not using this system
    imu_spi_config.spics_io_num = -1;                      // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
                                                           // driver, it must be handled via calls to gpio pins
    imu_spi_config.queue_size = 5;                         // only allow for 5 queued transactions at a time

    // SPI non-driver-controlled GPIO config
    // configure outputs
    gpio_config_t outputs_config;

    if (imu_config.io_wake != GPIO_NUM_NC)
        outputs_config.pin_bit_mask =
                (1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst) | (1ULL << imu_config.io_wake); // configure CS, RST, and wake gpio pins
    else
        outputs_config.pin_bit_mask = (1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst);

    outputs_config.mode = GPIO_MODE_OUTPUT;
    outputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&outputs_config);
    gpio_set_level(imu_config.io_cs, 1);
    gpio_set_level(imu_config.io_rst, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    // configure input (HINT pin)
    gpio_config_t inputs_config;
    inputs_config.pin_bit_mask = (1ULL << imu_config.io_int);
    inputs_config.mode = GPIO_MODE_INPUT;
    inputs_config.pull_up_en = GPIO_PULLUP_ENABLE;
    inputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputs_config.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&inputs_config);

    // check if GPIO ISR service has been installed (only has to be done once regardless of SPI slaves being used)
    if (!isr_service_installed)
    {
        gpio_install_isr_service(0); // install isr service
        isr_service_installed = true;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(imu_config.io_int, hint_handler, (void*) this));
    gpio_intr_disable(imu_config.io_int); // disable interrupts initially before reset

    // initialize the spi peripheral
    spi_bus_initialize(imu_config.spi_peripheral, &bus_config, SPI_DMA_CH_AUTO);
    // add the imu device to the bus
    spi_bus_add_device(imu_config.spi_peripheral, &imu_spi_config, &spi_hdl);

    // do first SPI operation into nowhere before BNO085 reset to let periphiral stabilize (Anton B.)
    spi_transaction.length = 8;
    spi_transaction.rxlength = 0;
    spi_transaction.tx_buffer = tx_buffer;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.flags = 0;
    spi_device_polling_transmit(spi_hdl, &spi_transaction); // send data packet
}

/**
 * @brief Initializes BNO08x sensor
 *
 * Resets sensor and goes through initializing process outlined in BNO08x datasheet.
 * Launches two tasks, one to manage SPI transactions, another to process any received data.
 *
 * @return void, nothing to return
 */
bool BNO08x::initialize()
{
    // launch tasks
    data_proc_task_hdl = NULL;
    spi_task_hdl = NULL;
    xTaskCreate(&data_proc_task_trampoline, "bno08x_data_processing_task", 4096, this, 7, &data_proc_task_hdl); // launch data processing task
    xTaskCreate(&spi_task_trampoline, "bno08x_spi_task", 4096, this, 8, &spi_task_hdl);                         // launch SPI task

    if (!hard_reset())
        return false;

    if (get_reset_reason() != 0)
    {
        ESP_LOGI(TAG, "Successfully initialized....");
        return true;
    }

    return false;
}

/**
 * @brief Waits for data to be received over SPI, or HOST_INT_TIMEOUT_MS to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 * This function is for when the validity of packets is not a concern, it is for flushing packets
 * we do not care about.
 *
 * @return True if data has been received over SPI within HOST_INT_TIMEOUT_MS.
 */
bool BNO08x::wait_for_rx_done()
{
    bool success = false;

    // if no reports are enabled we can assume interrupts are disabled (see spi_task())
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    // wait until an interrupt has been asserted and data received or timeout has occured
    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT, pdTRUE, pdTRUE, HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS))
    {
        if (imu_config.debug_en)
            ESP_LOGI(TAG, "int asserted");

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
 * @brief Waits for a valid or invalid packet to be received or HOST_INT_TIMEOUT_MS to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 *
 * @return True if valid packet has been received within HOST_INT_TIMEOUT_MS, false if otherwise.
 */
bool BNO08x::wait_for_data()
{
    bool success = false;

    // if no reports are enabled we can assume interrupts are disabled (see spi_task())
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    // check to see receive operation has finished
    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT, pdTRUE, pdTRUE, HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS))
    {
        // wait until processing is done
        if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET | EVT_GRP_SPI_RX_INVALID_PACKET, pdFALSE, pdFALSE,
                    HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS))
        {
            // only return true if packet is valid
            if (xEventGroupGetBits(evt_grp_spi) & EVT_GRP_SPI_RX_VALID_PACKET)
            {
                if (imu_config.debug_en)
                    ESP_LOGI(TAG, "Valid packet received.");
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

    xEventGroupClearBits(evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET | EVT_GRP_SPI_RX_INVALID_PACKET);
    return success;
}

/**
 * @brief Waits for a queued packet to be sent or HOST_INT_TIMEOUT_MS to elapse.
 *
 * If no reports are currently enabled the hint pin interrupt will be re-enabled by this function.
 *
 * @return True if packet was sent within HOST_INT_TIMEOUT_MS, false if otherwise.
 */
bool BNO08x::wait_for_tx_done()
{
    // if no reports are enabled we can assume interrupts are disabled (see spi_task())
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
        gpio_intr_enable(imu_config.io_int); // re-enable interrupts

    if (xEventGroupWaitBits(evt_grp_spi, EVT_GRP_SPI_TX_DONE, pdTRUE, pdTRUE, HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS))
    {

        if (imu_config.debug_en)
            ESP_LOGI(TAG, "Packet sent successfully.");

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
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 10ns min, set to 50ms to let things stabilize(Anton)
    gpio_set_level(imu_config.io_rst, 1); // bring out of reset

    // Receive advertisement message on boot (see SH2 Ref. Manual 5.2 & 5.3)
    if (!wait_for_rx_done()) // wait for receive operation to complete
    {
        ESP_LOGE(TAG, "Reset Failed, interrupt to host device never asserted.");
    }
    else
    {
        ESP_LOGI(TAG, "Received advertisement message.");

        // The BNO080 will then transmit an unsolicited Initialize Response (see SH2 Ref. Manual 6.4.5.2)
        if (!wait_for_rx_done())
        {
            ESP_LOGE(TAG, "Failed to receive initialize response on boot.");
        }
        else
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
    for (int i = 0; i < 3; i++)
    {
        wait_for_rx_done();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    return success;
}

/**
 * @brief Requests product ID, prints the returned info over serial, and returns the reason for the most resent reset. 
 *
 * @return The reason for the most recent recent reset ( 1 = POR (power on reset), 2 = internal reset, 3 = watchdog
 * timer, 4 = external reset 5 = other)
 */
uint8_t BNO08x::get_reset_reason()
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
            xQueueReceive(queue_reset_reason, &reset_reason, HOST_INT_TIMEOUT_MS/portTICK_PERIOD_MS);
        else
            ESP_LOGE(TAG, "Failed to receive product ID report.");
    }

    return reset_reason;
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
    for (int i = 0; i < 3; i++)
    {
        wait_for_rx_done();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

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
    for (int i = 0; i < 3; i++)
    {
        wait_for_rx_done();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    return success;
}

/**
 * @brief Receives a SHTP packet via SPI and sends it to data_proc_task()
 *
 * @return void, nothing to return
 */
bool BNO08x::receive_packet()
{
    bno08x_rx_packet_t packet;
    uint8_t dummy_header_tx[4] = {0};

    if (gpio_get_level(imu_config.io_int)) // ensure INT pin is low
        return false;

    // setup transaction to receive first 4 bytes (packet header)
    spi_transaction.rx_buffer = packet.header;
    spi_transaction.tx_buffer = dummy_header_tx;
    spi_transaction.length = 4 * 8;
    spi_transaction.rxlength = 4 * 8;
    spi_transaction.flags = 0;

    gpio_set_level(imu_config.io_cs, 0);                    // assert chip select
    spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive first 4 bytes (packet header)

    // calculate length of packet from received header
    packet.length = (((uint16_t) packet.header[1]) << 8) | ((uint16_t) packet.header[0]);
    packet.length &= ~(1 << 15); // Clear the MSbit

    if (imu_config.debug_en)
        ESP_LOGW(TAG, "packet rx length: %d", packet.length);

    if (packet.length == 0)
        return false;

    packet.length -= 4; // remove 4 header bytes from packet length (we already read those)

    // setup transacton to read the data packet
    spi_transaction.rx_buffer = packet.body;
    spi_transaction.tx_buffer = NULL;
    spi_transaction.length = packet.length * 8;
    spi_transaction.rxlength = packet.length * 8;
    spi_transaction.flags = 0;

    spi_device_polling_transmit(spi_hdl, &spi_transaction); // receive rest of packet

    gpio_set_level(imu_config.io_cs, 1); // de-assert chip select

    xQueueSend(queue_rx_data, &packet, 0); // send received data to data_proc_task
    xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_DONE_BIT);

    return true;
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
void BNO08x::enable_report(uint8_t report_ID, uint32_t time_between_reports, const EventBits_t report_evt_grp_bit)
{
    queue_feature_command(report_ID, time_between_reports);
    if (wait_for_tx_done()) // wait for transmit operation to complete
    {
        xEventGroupSetBits(evt_grp_report_en, report_evt_grp_bit);

        // if no reports were enabled before this one, we can assume hint interrupt was disabled, re-enable to read reports
        if ((xEventGroupGetBits(evt_grp_report_en) & ~report_evt_grp_bit) == 0)
            gpio_intr_enable(imu_config.io_int);
    }

    // flush the first few reports returned to ensure new data
    for (int i = 0; i < 3; i++)
        wait_for_rx_done();
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
    queue_feature_command(report_ID, 0);
    if (wait_for_tx_done()) // wait for transmit operation to complete
    {
        xEventGroupClearBits(evt_grp_report_en, report_evt_grp_bit);

        // no reports enabled, disable hint to avoid wasting processing time
        if ((xEventGroupGetBits(evt_grp_report_en)) == 0)
            gpio_intr_disable(imu_config.io_int);
    }
}

/**
 * @brief Queues an SHTP packet to be sent via SPI.
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

    xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_TX_DONE);
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
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate accelerometer.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_accelerometer()
{
    queue_calibrate_command(CALIBRATE_ACCEL);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate gyro.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_gyro()
{
    queue_calibrate_command(CALIBRATE_GYRO);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate magnetometer.
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_magnetometer()
{
    queue_calibrate_command(CALIBRATE_MAG);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate planar accelerometer
 *
 * @return void, nothing to return
 */
void BNO08x::calibrate_planar_accelerometer()
{
    queue_calibrate_command(CALIBRATE_PLANAR_ACCEL);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Queues a packet containing a command to calibrate the specified sensor.
 *
 * @param sensor_to_calibrate The sensor to calibrate.
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
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Returns true if calibration has completed.
 *
 * @return void, nothing to return
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
    vTaskDelay(50 / portTICK_PERIOD_MS);     // allow some time for command to be executed
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
    vTaskDelay(50 / portTICK_PERIOD_MS);  // allow some time for command to be executed
}

/**
 * @brief Runs full calibration routine.
 *
 * Enables game rotation vector and magnetometer, starts ME calibration process.
 * Waits for accuracy of returned quaternions and magnetic field vectors to be high, then saves calibration data and
 * returns.
 *
 * @return void, nothing to return
 */
bool BNO08x::run_full_calibration_routine()
{
    float magf_x = 0;
    float magf_y = 0;
    float magf_z = 0;
    uint8_t magnetometer_accuracy = (uint8_t) IMUAccuracy::LOW;

    float quat_I = 0;
    float quat_J = 0;
    float quat_K = 0;
    float quat_real = 0;
    uint8_t quat_accuracy = (uint8_t) IMUAccuracy::LOW;

    uint16_t high_accuracy = 0;
    uint16_t save_calibration_attempt = 0;

    // Enable dynamic calibration for accel, gyro, and mag
    calibrate_all(); // Turn on cal for Accel, Gyro, and Mag

    // Enable Game Rotation Vector output
    enable_game_rotation_vector(100); // Send data update every 100ms

    // Enable Magnetic Field output
    enable_magnetometer(100); // Send data update every 100ms

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

            ESP_LOGI(TAG, "Magnetometer: x: %.3f y: %.3f z: %.3f, accuracy: %d", magf_x, magf_y, magf_z, magnetometer_accuracy);
            ESP_LOGI(TAG, "Quaternion Rotation Vector: i: %.3f j: %.3f k: %.3f, real: %.3f, accuracy: %d", quat_I, quat_J, quat_K, quat_real,
                    quat_accuracy);

            vTaskDelay(5 / portTICK_PERIOD_MS);

            if ((magnetometer_accuracy >= (uint8_t) IMUAccuracy::MED) && (quat_accuracy == (uint8_t) IMUAccuracy::HIGH))
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
 * @return true if new data has been parsed and saved
 */
bool BNO08x::data_available()
{
    if (xEventGroupGetBits(evt_grp_report_en) == 0)
    {
        ESP_LOGE(TAG, "No reports enabled.");
        return false;
    }

    return wait_for_data();
}

/**
 * @brief Parses a packet received from bno08x, updating any data according to received reports.
 *
 * @param packet The packet to be parsed.
 * @return 0 if invalid packet.
 */
uint16_t BNO08x::parse_packet(bno08x_rx_packet_t* packet)
{
    if (imu_config.debug_en)
        ESP_LOGE(TAG, "SHTP Header RX'd: 0x%X 0x%X 0x%X 0x%X", packet->header[0], packet->header[1], packet->header[2], packet->header[3]);

    if (packet->body[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) // check to see that product ID matches what it should
    {
        return parse_product_id_report(packet);
    }

    if (packet->body[0] == SHTP_REPORT_FRS_READ_RESPONSE)
    {
        return parse_frs_read_response_report(packet);
    }

    // Check to see if this packet is a sensor reporting its data to us
    if (packet->header[2] == CHANNEL_REPORTS && packet->body[0] == SHTP_REPORT_BASE_TIMESTAMP)
    {
        if (imu_config.debug_en)
            ESP_LOGI(TAG, "RX'd packet, channel report");

        return parse_input_report(packet); // This will update the rawAccelX, etc variables depending on which feature
                                           // report is found
    }
    else if (packet->header[2] == CHANNEL_CONTROL)
    {
        if (imu_config.debug_en)
            ESP_LOGI(TAG, "RX'd packet, channel control");

        return parse_command_report(packet); // This will update responses to commands, calibrationStatus, etc.
    }
    else if (packet->header[2] == CHANNEL_GYRO)
    {
        if (imu_config.debug_en)
            ESP_LOGI(TAG, "Rx packet, channel gyro");

        return parse_input_report(packet); // This will update the rawAccelX, etc variables depending on which feature
                                           // report is found
    }

    return 0;
}

/**
 * @brief Parses product id report and prints device info.
 *
 * @param packet The packet containing product id report.
 * @return 1, always valid.
 */
uint16_t BNO08x::parse_product_id_report(bno08x_rx_packet_t* packet)
{
    uint32_t reset_reason = (uint32_t) packet->body[1];
    uint32_t sw_part_number = ((uint32_t) packet->body[7] << 24) | ((uint32_t) packet->body[6] << 16) | ((uint32_t) packet->body[5] << 8) |
                              ((uint32_t) packet->body[4]);
    uint32_t sw_build_number = ((uint32_t) packet->body[11] << 24) | ((uint32_t) packet->body[10] << 16) | ((uint32_t) packet->body[9] << 8) |
                               ((uint32_t) packet->body[8]);
    uint16_t sw_version_patch = ((uint16_t) packet->body[13] << 8) | ((uint16_t) packet->body[12]);

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
            (uint32_t) packet->body[0], (uint32_t) packet->body[2], (uint32_t) packet->body[3], sw_part_number, sw_build_number,
            (uint32_t) sw_version_patch);

    xQueueSend(queue_reset_reason, &reset_reason, 0);

    return 1;
}

/**
 * @brief Sends packet to be parsed to meta data function call (frs_read_word()) through queue.
 *
 * @param packet The packet containing the frs read report.
 * @return 1, always valid, parsing for this happens in frs_read_word()
 */
uint16_t BNO08x::parse_frs_read_response_report(bno08x_rx_packet_t* packet)
{
    xQueueSend(queue_frs_read_data, &packet->body, 0);
    return 1;
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
 * @return The report ID of the respective sensor, for ex. SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR, 0 if invalid.
 */
uint16_t BNO08x::parse_input_report(bno08x_rx_packet_t* packet)
{
    uint16_t i = 0;
    uint8_t status = 0;
    uint8_t command = 0;

    // Calculate the number of data bytes in this packet
    uint16_t data_length = ((uint16_t) packet->header[1] << 8 | packet->header[0]);
    data_length &= ~(1 << 15); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // Ignore it for now. TODO catch this as an error and exit

    data_length -= 4; // Remove the header bytes from the data count
    time_stamp = ((uint32_t) packet->body[4] << (8 * 3)) | ((uint32_t) packet->body[3] << (8 * 2)) | ((uint32_t) packet->body[2] << (8 * 1)) |
                 ((uint32_t) packet->body[1] << (8 * 0));

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence,
    // and status fields
    if (packet->header[2] == CHANNEL_GYRO)
    {
        raw_quat_I = (uint16_t) packet->body[1] << 8 | packet->body[0];
        raw_quat_J = (uint16_t) packet->body[3] << 8 | packet->body[2];
        raw_quat_K = (uint16_t) packet->body[5] << 8 | packet->body[4];
        raw_quat_real = (uint16_t) packet->body[7] << 8 | packet->body[6];
        raw_velocity_gyro_X = (uint16_t) packet->body[9] << 8 | packet->body[8];
        raw_velocity_gyro_Y = (uint16_t) packet->body[11] << 8 | packet->body[10];
        raw_velocity_gyro_Z = (uint16_t) packet->body[13] << 8 | packet->body[12];

        return SENSOR_REPORT_ID_GYRO_INTEGRATED_ROTATION_VECTOR;
    }

    status = packet->body[5 + 2] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t) packet->body[5 + 5] << 8 | packet->body[5 + 4];
    uint16_t data2 = (uint16_t) packet->body[5 + 7] << 8 | packet->body[5 + 6];
    uint16_t data3 = (uint16_t) packet->body[5 + 9] << 8 | packet->body[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0;
    uint16_t data6 = 0;

    if (data_length - 5 > 9)
    {
        data4 = (uint16_t) packet->body[5 + 11] << 8 | packet->body[5 + 10];
    }
    if (data_length - 5 > 11)
    {
        data5 = (uint16_t) packet->body[5 + 13] << 8 | packet->body[5 + 12];
    }
    if (data_length - 5 > 13)
    {
        data6 = (uint16_t) packet->body[5 + 15] << 8 | packet->body[5 + 14];
    }

    // Store these generic values to their proper global variable
    switch (packet->body[5])
    {
    case SENSOR_REPORT_ID_ACCELEROMETER:
        accel_accuracy = status;
        raw_accel_X = data1;
        raw_accel_Y = data2;
        raw_accel_Z = data3;
        break;

    case SENSOR_REPORT_ID_LINEAR_ACCELERATION:
        accel_lin_accuracy = status;
        raw_lin_accel_X = data1;
        raw_lin_accel_Y = data2;
        raw_lin_accel_Z = data3;
        break;

    case SENSOR_REPORT_ID_GYROSCOPE:
        gyro_accuracy = status;
        raw_gyro_X = data1;
        raw_gyro_Y = data2;
        raw_gyro_Z = data3;
        break;

    case SENSOR_REPORT_ID_UNCALIBRATED_GYRO:
        uncalib_gyro_accuracy = status;
        raw_uncalib_gyro_X = data1;
        raw_uncalib_gyro_Y = data2;
        raw_uncalib_gyro_Z = data3;
        raw_bias_X = data4;
        raw_bias_Y = data5;
        raw_bias_Z = data6;
        break;

    case SENSOR_REPORT_ID_MAGNETIC_FIELD:
        magf_accuracy = status;
        raw_magf_X = data1;
        raw_magf_Y = data2;
        raw_magf_Z = data3;
        break;

    case SENSOR_REPORT_ID_TAP_DETECTOR:
        tap_detector = packet->body[5 + 4]; // Byte 4 only
        break;

    case SENSOR_REPORT_ID_STEP_COUNTER:
        step_count = data3; // Bytes 8/9
        break;

    case SENSOR_REPORT_ID_STABILITY_CLASSIFIER:
        stability_classifier = packet->body[5 + 4]; // Byte 4 only
        break;

    case SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER:
        activity_classifier = packet->body[5 + 5]; // Most likely state

        // Load activity classification confidences into the array
        for (i = 0; i < 9; i++)                                // Hardcoded to max of 9. TODO - bring in array size
            activity_confidences[i] = packet->body[5 + 6 + i]; // 5 bytes of timestamp, byte 6 is first confidence
                                                               // byte
        break;

    case SENSOR_REPORT_ID_RAW_ACCELEROMETER:
        mems_raw_accel_X = data1;
        mems_raw_accel_Y = data2;
        mems_raw_accel_Z = data3;
        break;

    case SENSOR_REPORT_ID_RAW_GYROSCOPE:
        mems_raw_gyro_X = data1;
        mems_raw_gyro_Y = data2;
        mems_raw_gyro_Z = data3;
        break;

    case SENSOR_REPORT_ID_RAW_MAGNETOMETER:
        mems_raw_magf_X = data1;
        mems_raw_magf_Y = data2;
        mems_raw_magf_Z = data3;
        break;

    case SHTP_REPORT_COMMAND_RESPONSE:
        // The BNO080 responds with this report to command requests. It's up to use to remember which command we
        // issued.
        command = packet->body[5 + 2]; // This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
            calibration_status = packet->body[5 + 5]; // R0 - Status (0 = success, non-zero = fail)
        break;

    case SENSOR_REPORT_ID_GRAVITY:
        gravity_accuracy = status;
        gravity_X = data1;
        gravity_Y = data2;
        gravity_Z = data3;
        break;

    default:
        if (packet->body[5] == SENSOR_REPORT_ID_ROTATION_VECTOR || packet->body[5] == SENSOR_REPORT_ID_GAME_ROTATION_VECTOR ||
                packet->body[5] == SENSOR_REPORT_ID_ARVR_STABILIZED_ROTATION_VECTOR ||
                packet->body[5] == SENSOR_REPORT_ID_ARVR_STABILIZED_GAME_ROTATION_VECTOR)
        {
            quat_accuracy = status;
            raw_quat_I = data1;
            raw_quat_J = data2;
            raw_quat_K = data3;
            raw_quat_real = data4;

            // Only available on rotation vector and ar/vr stabilized rotation vector,
            //  not game rot vector and not ar/vr stabilized rotation vector
            raw_quat_radian_accuracy = data5;
        }
        else
        {
            // This sensor report ID is unhandled.
            // See reference manual to add additional feature reports as needed
            return 0;
        }

        break;
    }

    // TODO additional feature reports may be strung together. Parse them all.
    return packet->body[5];
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
 * @brief Sends command to enable gyro reports (See Ref. Manual 6.5.13)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_gyro(uint32_t time_between_reports)
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
void BNO08x::enable_activity_classifier(uint32_t time_between_reports, uint32_t activities_to_enable, uint8_t (&activity_confidence_vals)[9])
{
    activity_confidences = activity_confidence_vals; // Store pointer to array
    enable_report(SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER, time_between_reports, EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT);
}

/**
 * @brief Sends command to enable raw accelerometer reports (See Ref. Manual 6.5.8)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_accelerometer(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_RAW_ACCELEROMETER, time_between_reports, EVT_GRP_RPT_RAW_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to enable raw gyro reports (See Ref. Manual 6.5.12)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_gyro(uint32_t time_between_reports)
{
    enable_report(SENSOR_REPORT_ID_RAW_GYROSCOPE, time_between_reports, EVT_GRP_RPT_RAW_GYRO_BIT);
}

/**
 * @brief Sends command to enable raw magnetometer reports (See Ref. Manual 6.5.15)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x::enable_raw_magnetometer(uint32_t time_between_reports)
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
 * @brief Sends command to disable gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_gyro()
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
    disable_report(SENSOR_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER, EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT);
}

/**
 * @brief Sends command to disable raw accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_accelerometer()
{
    disable_report(SENSOR_REPORT_ID_RAW_ACCELEROMETER, EVT_GRP_RPT_RAW_ACCELEROMETER_BIT);
}

/**
 * @brief Sends command to disable raw gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_gyro()
{
    disable_report(SENSOR_REPORT_ID_RAW_GYROSCOPE, EVT_GRP_RPT_RAW_GYRO_BIT);
}

/**
 * @brief Sends command to disable raw magnetometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x::disable_raw_magnetometer()
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
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to save tare into non-volatile memory of BNO08x (See Ref. Manual 6.4.4.2)
 *
 * @return void, nothing to return
 */
void BNO08x::save_tare()
{
    queue_tare_command(TARE_PERSIST);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to clear persistent tare settings in non-volatile memory of BNO08x (See Ref. Manual 6.4.4.3)
 *
 * @return void, nothing to return
 */
void BNO08x::clear_tare()
{
    queue_tare_command(TARE_SET_REORIENTATION);
    wait_for_tx_done();                  // wait for transmit operation to complete
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
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
 * @return void, nothing to return
 */
uint32_t BNO08x::get_time_stamp()
{
    return time_stamp;
}

/**
 * @brief Get the full magnetic field vector.
 *
 * @param x Reference variable to save reported x magnitude.
 * @param y Reference variable to save reported y magnitude.
 * @param x Reference variable to save reported z magnitude.
 * @param accuracy Reference variable save reported accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_magf(float& x, float& y, float& z, uint8_t& accuracy)
{
    x = q_to_float(raw_magf_X, MAGNETOMETER_Q1);
    y = q_to_float(raw_magf_Y, MAGNETOMETER_Q1);
    z = q_to_float(raw_magf_Z, MAGNETOMETER_Q1);
    accuracy = magf_accuracy;
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
uint8_t BNO08x::get_magf_accuracy()
{
    return magf_accuracy;
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
void BNO08x::get_gravity(float& x, float& y, float& z, uint8_t& accuracy)
{
    x = q_to_float(gravity_X, GRAVITY_Q1);
    y = q_to_float(gravity_Y, GRAVITY_Q1);
    z = q_to_float(gravity_Z, GRAVITY_Q1);
    accuracy = gravity_accuracy;
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
uint8_t BNO08x::get_gravity_accuracy()
{
    return gravity_accuracy;
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
void BNO08x::get_quat(float& i, float& j, float& k, float& real, float& rad_accuracy, uint8_t& accuracy)
{
    i = q_to_float(raw_quat_I, ROTATION_VECTOR_Q1);
    j = q_to_float(raw_quat_J, ROTATION_VECTOR_Q1);
    k = q_to_float(raw_quat_K, ROTATION_VECTOR_Q1);
    real = q_to_float(raw_quat_real, ROTATION_VECTOR_Q1);
    rad_accuracy = q_to_float(raw_quat_radian_accuracy, ROTATION_VECTOR_Q1);
    accuracy = quat_accuracy;
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
uint8_t BNO08x::get_quat_accuracy()
{
    return quat_accuracy;
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
void BNO08x::get_accel(float& x, float& y, float& z, uint8_t& accuracy)
{
    x = q_to_float(raw_accel_X, ACCELEROMETER_Q1);
    y = q_to_float(raw_accel_Y, ACCELEROMETER_Q1);
    z = q_to_float(raw_accel_Z, ACCELEROMETER_Q1);
    accuracy = accel_accuracy;
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
uint8_t BNO08x::get_accel_accuracy()
{
    return accel_accuracy;
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
void BNO08x::get_linear_accel(float& x, float& y, float& z, uint8_t& accuracy)
{
    x = q_to_float(raw_lin_accel_X, LINEAR_ACCELEROMETER_Q1);
    y = q_to_float(raw_lin_accel_Y, LINEAR_ACCELEROMETER_Q1);
    z = q_to_float(raw_lin_accel_Z, LINEAR_ACCELEROMETER_Q1);
    accuracy = accel_lin_accuracy;
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
uint8_t BNO08x::get_linear_accel_accuracy()
{
    return accel_lin_accuracy;
}

/**
 * @brief Get raw accelerometer x axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer x axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_accel_X()
{
    return mems_raw_accel_X;
}

/**
 * @brief Get raw accelerometer y axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer y axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_accel_Y()
{
    return mems_raw_accel_Y;
}

/**
 * @brief Get raw accelerometer z axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer z axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_accel_Z()
{
    return mems_raw_accel_Z;
}

/**
 * @brief Get raw gyroscope x axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope x axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_gyro_X()
{
    return mems_raw_gyro_X;
}

/**
 * @brief Get raw gyroscope y axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope y axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_gyro_Y()
{
    return mems_raw_gyro_Y;
}

/**
 * @brief Get raw gyroscope z axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope z axis reading from physical MEMs sensor.
 */
int16_t BNO08x::get_raw_gyro_Z()
{
    return mems_raw_gyro_Z;
}

/**
 * @brief Get raw magnetometer x axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer x axis reading from physical magnetometer sensor.
 */
int16_t BNO08x::get_raw_magf_X()
{
    return mems_raw_magf_X;
}

/**
 * @brief Get raw magnetometer y axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer y axis reading from physical magnetometer sensor.
 */
int16_t BNO08x::get_raw_magf_Y()
{
    return mems_raw_magf_Y;
}

/**
 * @brief Get raw magnetometer z axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer z axis reading from physical magnetometer sensor.
 */
int16_t BNO08x::get_raw_magf_Z()
{
    return mems_raw_magf_Z;
}

/**
 * @brief Get full rotational velocity with drift compensation (units in Rad/s).
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 * @param accuracy Reference variable to save reported gyro accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_gyro_calibrated_velocity(float& x, float& y, float& z, uint8_t& accuracy)
{
    x = q_to_float(raw_gyro_X, GYRO_Q1);
    y = q_to_float(raw_gyro_Y, GYRO_Q1);
    z = q_to_float(raw_gyro_Z, GYRO_Q1);
    accuracy = gyro_accuracy;
}

/**
 * @brief Get calibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_gyro_calibrated_velocity_X()
{
    return q_to_float(raw_gyro_X, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro y axis angular velocity measurement.
 *
 * @return The angular reported y axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_gyro_calibrated_velocity_Y()
{
    return q_to_float(raw_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro z axis angular velocity measurement.
 *
 * @return The angular reported z axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x::get_gyro_calibrated_velocity_Z()
{
    return q_to_float(raw_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro accuracy.
 *
 * @return Accuracy of calibrated gyro.
 */
uint8_t BNO08x::get_gyro_accuracy()
{
    return gyro_accuracy;
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
 * @param accuracy Reference variable to save reported gyro accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x::get_uncalibrated_gyro(float& x, float& y, float& z, float& b_x, float& b_y, float& b_z, uint8_t& accuracy)
{
    x = q_to_float(raw_uncalib_gyro_X, GYRO_Q1);
    y = q_to_float(raw_uncalib_gyro_Y, GYRO_Q1);
    z = q_to_float(raw_uncalib_gyro_Z, GYRO_Q1);
    b_x = q_to_float(raw_bias_X, GYRO_Q1);
    b_y = q_to_float(raw_bias_Y, GYRO_Q1);
    b_z = q_to_float(raw_bias_Z, GYRO_Q1);
    accuracy = uncalib_gyro_accuracy;
}

/**
 * @brief Get uncalibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_X()
{
    return q_to_float(raw_uncalib_gyro_X, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Y axis angular velocity measurement.
 *
 * @return The angular reported Y axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_Y()
{
    return q_to_float(raw_uncalib_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Z axis angular velocity measurement.
 *
 * @return The angular reported Z axis angular velocity from uncalibrated gyro.
 */
float BNO08x::get_uncalibrated_gyro_Z()
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
 * @brief Get uncalibrated gyro accuracy.
 *
 * @return Accuracy of uncalibrated gyro.
 */
uint8_t BNO08x::get_uncalibrated_gyro_accuracy()
{
    return uncalib_gyro_accuracy;
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
void BNO08x::get_gyro_velocity(float& x, float& y, float& z)
{
    x = q_to_float(raw_velocity_gyro_X, ANGULAR_VELOCITY_Q1);
    y = q_to_float(raw_velocity_gyro_Y, ANGULAR_VELOCITY_Q1);
    z = q_to_float(raw_velocity_gyro_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get x axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported x axis angular velocity.
 */
float BNO08x::get_gyro_velocity_X()
{
    return q_to_float(raw_velocity_gyro_X, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get y axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported y axis angular velocity.
 */
float BNO08x::get_gyro_velocity_Y()
{
    return q_to_float(raw_velocity_gyro_Y, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get z axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported Z axis angular velocity.
 */
float BNO08x::get_gyro_velocity_Z()
{
    return q_to_float(raw_velocity_gyro_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get if tap has occured.
 *
 * @return 7 bit tap code indicated which axis taps have occurred. (See Ref. Manual 6.5.27)
 */
uint8_t BNO08x::get_tap_detector()
{
    uint8_t previous_tap_detector = tap_detector;
    tap_detector = 0; // Reset so user code sees exactly one tap
    return (previous_tap_detector);
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
int8_t BNO08x::get_stability_classifier()
{
    return stability_classifier;
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
uint8_t BNO08x::get_activity_classifier()
{
    return activity_classifier;
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
    return (uint16_t) (FRS_read_word(record_ID, 7) & 0xFFFF);
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
    return (uint16_t) (FRS_read_word(record_ID, 7) >> 16U);
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
    return (uint16_t) (FRS_read_word(record_ID, 8) >> 16U);
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
    uint32_t data_0;
    uint32_t data_1;
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

                if (xQueueReceive(queue_frs_read_data, &packet_body, HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS))
                {
                    if ((((uint16_t) packet_body[13] << 8) | (uint16_t) packet_body[12]) == record_ID)
                        break;
                }
            }

            data_length = packet_body[1] >> 4;
            frs_status = packet_body[1] & 0x0F;

            data_0 = (uint32_t) packet_body[7] << 24 | (uint32_t) packet_body[6] << 16 | (uint32_t) packet_body[5] << 8 | (uint32_t) packet_body[4];
            data_1 = (uint32_t) packet_body[11] << 24 | (uint32_t) packet_body[10] << 16 | (uint32_t) packet_body[9] << 8 | (uint32_t) packet_body[8];

            if (data_length > 0)
                meta_data[spot++] = data_0;

            if (data_length > 1)
                meta_data[spot++] = data_1;

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
 * @brief Queues a packet containing a command with a request for sensor reports, reported periodically. (See Ref.
 * Manual 6.5.4)
 *
 * @param report_ID ID of sensor report being requested.
 * @param time_between_reports Desired time between reports.
 *
 * @return void, nothing to return
 */
void BNO08x::queue_feature_command(uint8_t report_ID, uint32_t time_between_reports)
{
    queue_feature_command(report_ID, time_between_reports, 0); // No specific config
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
    static uint64_t prev_time = 0;
    bno08x_tx_packet_t tx_packet;

    while (1)
    {
        /*only re-enable interrupts if there are reports enabled, if no reports are enabled
         interrupts will be re-enabled upon the next user call to wait_for_tx_done(), wait_for_rx_done(), or wait_for_data()*/
        if (xEventGroupGetBits(evt_grp_report_en) != 0)
            gpio_intr_enable(imu_config.io_int);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until notified by ISR (hint_handler)
        if (imu_config.debug_en)
        {
            ESP_LOGI(TAG, "HINT asserted, time since last assertion: %llu", (esp_timer_get_time() - prev_time));
            prev_time = esp_timer_get_time();
        }

        if (xQueueReceive(queue_tx_data, &tx_packet, 0)) // check for queued packet to be sent, non blocking
            send_packet(&tx_packet);                     // send packet
        else
            receive_packet(); // receive packet
    }
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

    while (1)
    {
        if (xQueueReceive(queue_rx_data, &packet, portMAX_DELAY)) // receive packet from spi_task()
        {
            if (parse_packet(&packet) != 0)                                   // check if packet is valid
                xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_VALID_PACKET); // indicate valid packet to wait_for_data()
            else
                xEventGroupSetBits(evt_grp_spi, EVT_GRP_SPI_RX_INVALID_PACKET); // indicated invalid packet to wait_for_data()
        }
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
