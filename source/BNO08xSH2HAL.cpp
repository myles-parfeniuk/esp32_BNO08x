/**
 * @file BNO08xSH2HAL.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xSH2HAL.hpp"
#include "BNO08x.hpp"

BNO08x* BNO08xSH2HAL::imu;

/**
 * @brief Sets the BNO08x driver object to be used with sh2 HAL lib callbacks.
 *
 * @param hal_imu Pointer to BNO08x driver object to be used with sh2 HAL lib callbacks.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::set_hal_imu(BNO08x* hal_imu)
{
    imu = hal_imu;
}

/**
 * @brief Opens SPI instance by waiting for interrupt.
 *
 * @return Always returns 0.
 */
int BNO08xSH2HAL::spi_open(sh2_Hal_t* self)
{
    spi_wait_for_int();

    return 0;
}

/**
 * @brief Closes SPI instance (nothing to do here, but required by sh2 HAL lib for cases where other communication protocols are used.)
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::spi_close(sh2_Hal_t* self)
{
    // do nothing
}

/**
 * @brief SPI rx callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 * @param pBuffer Buffer to store received packet.
 * @param len Length of bytes to read.
 * @param t_us Time in microseconds (unused, forced sh2 HAL lib required function type)
 *
 * @return Size of received packet in bytes, 0 on failure.
 */
int BNO08xSH2HAL::spi_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us)
{
    uint16_t packet_sz = 0;

    // hint never asserted, fail transaction
    if (!spi_wait_for_int())
        return 0;

    // assert chip select
    gpio_set_level(imu->imu_config.io_cs, 0);

    packet_sz = spi_read_sh2_packet_header(pBuffer);

    if ((packet_sz > len) || (packet_sz == 0))
    {
        gpio_set_level(imu->imu_config.io_cs, 1);
        return 0;
    }

    packet_sz = spi_read_sh2_packet_body(pBuffer, packet_sz);

    // de-assert chip select
    gpio_set_level(imu->imu_config.io_cs, 1);

    return packet_sz;
}

/**
 * @brief SPI tx callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 * @param pBuffer Buffer containing data to write.
 * @param len Length in bytes to write.
 *
 * @return Size of sent data (len param), 0 on failure.
 */
int BNO08xSH2HAL::spi_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len)
{
    // hint never asserted, fail transaction
    if (!spi_wait_for_int())
        return 0;

    // setup transaction to send packet
    imu->spi_transaction.length = len * 8;
    imu->spi_transaction.rxlength = 0;
    imu->spi_transaction.tx_buffer = pBuffer;
    imu->spi_transaction.rx_buffer = NULL;
    imu->spi_transaction.flags = 0;

    gpio_set_level(imu->imu_config.io_cs, 0);                         // assert chip select
    spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction); // send data packet
    gpio_set_level(imu->imu_config.io_cs, 1);                         // de-assert chip select

    return len;
}

/**
 * @brief Get time in microseconds callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 *
 * @return Time in microseconds.
 */
uint32_t BNO08xSH2HAL::get_time_us(sh2_Hal_t* self)
{
    uint64_t time_us = esp_timer_get_time();

    if (time_us > UINT32_MAX)
        time_us -= UINT32_MAX;

    return static_cast<uint32_t>(time_us & 0xFFFFFFFFU);
}

/**
 * @brief General event callback for sh2 HAL lib, used to notify tasks of reset.
 *
 * @param cookie User set input parameter as void pointer (unused), see sh2_Open().
 * @param pEvent Pointer to asynchronous event.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::hal_cb(void* cookie, sh2_AsyncEvent_t* pEvent)
{
    if (pEvent->eventId == SH2_RESET)
        xEventGroupSetBits(imu->evt_grp_bno08x_task, BNO08xPrivateTypes::EVT_GRP_BNO08x_TASK_RESET_OCCURRED);
}

/**
 * @brief Sensor event callback for sh2 HAL lib, sends received reports to data_proc_task().
 *
 * @param cookie User set input parameter as void pointer (unused), see sh2_Open().
 * @param event Pointer to sensor event.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::sensor_event_cb(void* cookie, sh2_SensorEvent_t* event)
{
    xQueueSend(imu->queue_rx_sensor_event, event, 0);
}

/**
 * @brief Hardware reset callback for sh2 HAL lib, toggle RST gpio.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::hardware_reset()
{
    imu->toggle_reset();
}

/**
 * @brief SPI wait for HINT sh2 HAL lib callback.
 *
 * @return True if interrupt was detected before timeout.
 */
bool BNO08xSH2HAL::spi_wait_for_int()
{
    if (imu->wait_for_hint() != ESP_OK)
    {
        hardware_reset();
        return false;
    }

    return true;
}

/**
 * @brief SPI rx packet header (invoked from SPI rx callback.)
 *
 * @param pBuffer Buffer to store received header.
 *
 * @return Packet size (should always be 4), 0 on failure.
 */
uint16_t BNO08xSH2HAL::spi_read_sh2_packet_header(uint8_t* pBuffer)
{
    uint8_t dummy_header_tx[4] = {0};
    uint16_t packet_sz = 0;

    // setup transaction to receive first 4 bytes (packet header)
    imu->spi_transaction.rx_buffer = pBuffer;
    imu->spi_transaction.tx_buffer = dummy_header_tx;
    imu->spi_transaction.length = 4 * 8;
    imu->spi_transaction.rxlength = 4 * 8;
    imu->spi_transaction.flags = 0;

    if (spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction) != ESP_OK)
        return 0;

    packet_sz = PARSE_PACKET_LENGTH(pBuffer);

    // clear continuation/batch bit
    packet_sz &= ~0x8000U;

    return packet_sz;
}

/**
 * @brief SPI rx packet body (invoked from SPI rx callback.)
 *
 * @param pBuffer Buffer to store received packet body.
 *
 * @return Packet size, 0 on failure.
 */
int BNO08xSH2HAL::spi_read_sh2_packet_body(uint8_t* pBuffer, uint16_t packet_sz)
{
    imu->spi_transaction.rx_buffer = pBuffer + 4;
    imu->spi_transaction.tx_buffer = NULL;
    imu->spi_transaction.length = packet_sz * 8;
    imu->spi_transaction.rxlength = packet_sz * 8;
    imu->spi_transaction.flags = 0;

    if (spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction) != ESP_OK)
        return 0;
    else
        return packet_sz;
}
