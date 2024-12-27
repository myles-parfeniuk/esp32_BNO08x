/**
 * @file BNO08xRpt.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRpt.hpp"

/**
 * @brief Enables a sensor report such that the BNO08x begins sending it.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRpt::rpt_enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    int sh2_res = SH2_OK;
    int16_t idx = -1;

    sensor_cfg.reportInterval_us = time_between_reports;

    lock_sh2_HAL();
    sh2_res = sh2_setSensorConfig(ID, &sensor_cfg);
    unlock_sh2_HAL();

    if (sh2_res != SH2_OK)
    {
        return false;
    }
    else
    {
        xEventGroupSetBits(sync_ctx->evt_grp_rpt_en, rpt_bit); // set the event group bit

        vTaskDelay(30UL / portTICK_PERIOD_MS); // delay a bit to allow command to execute
        period_us = time_between_reports;      // update the period

        lock_user_data();
        for (int i = 0; i < sync_ctx->en_report_ids.size(); i++)
        {
            if (sync_ctx->en_report_ids[i] == ID)
            {
                idx = i;
                break;
            }
        }

        // if not already enabled (ie user called this, not re_enable_reports())
        if (idx == -1)
            sync_ctx->en_report_ids.push_back(ID); // add report ID to enabled report IDs

        unlock_user_data();

        return true;
    }
}

/**
 * @brief Disables a sensor report by setting its period to 0us such that the BNO08x stops sending
 * it.
 *
 * @param sensor_ID The ID of the sensor for the respective report to be disabled.
 * @param sensor_cfg Sensor special configuration.
 *
 * @return ESP_OK if report was successfully disabled.
 */
bool BNO08xRpt::disable(sh2_SensorConfig_t sensor_cfg)
{
    int sh2_res = SH2_OK;
    int16_t idx = -1;

    sensor_cfg.reportInterval_us = 0UL;

    lock_sh2_HAL();
    sh2_res = sh2_setSensorConfig(ID, &sensor_cfg);
    unlock_sh2_HAL();

    if (sh2_res != SH2_OK)
    {
        return false;
    }
    else
    {
        // clear the event group bit (this is redundant if called from BNO08x::disable_all_reports())
        xEventGroupClearBits(sync_ctx->evt_grp_rpt_en, rpt_bit);

        // remove report ID from enabled report IDs
        lock_user_data();
        for (int i = 0; i < sync_ctx->en_report_ids.size(); i++)
        {
            if (sync_ctx->en_report_ids[i] == ID)
            {
                idx = i;
                break;
            }
        }

        vTaskDelay(30UL / portTICK_PERIOD_MS); // delay a bit to allow command to execute
        period_us = 0UL;                       // update the period

        if (idx != -1)
            sync_ctx->en_report_ids.erase(sync_ctx->en_report_ids.begin() + idx);

        unlock_user_data();
    }

    return true;
}

/**
 * @brief Registers a callback to execute when new data from a specific report is received.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type void input param.
 *
 * @return void, nothing to return
 */
bool BNO08xRpt::register_cb(std::function<void(void)> cb_fxn)
{
    if (sync_ctx->cb_list.size() < CONFIG_ESP32_BNO08X_CB_MAX)
    {
        sync_ctx->cb_list.push_back(BNO08xCbParamVoid(cb_fxn, ID));
        return true;
    }
    return false;
}

/**
 * @brief Checks if a new report has been received since the last time this function was called.
 *
 *
 * @return True if a new report was received since the last time this function was called.
 */
bool BNO08xRpt::has_new_data()
{
    bool new_data = false;

    if (xEventGroupGetBits(sync_ctx->evt_grp_rpt_data_available) & rpt_bit)
    {
        new_data = true;
        xEventGroupClearBits(sync_ctx->evt_grp_rpt_data_available, rpt_bit);
    }

    return new_data;
}

/**
 * @brief Flush all buffered reports for this sensor/report module.
 *
 * @return True if flush operation succeeded.
 */
bool BNO08xRpt::flush()
{
    int success = SH2_OK;

    lock_sh2_HAL();
    success = sh2_flush(ID);
    unlock_sh2_HAL();

    return (success != SH2_OK) ? false : true;
}

/**
 * @brief Gets sample counts for this sensor (see SH-2 ref manual 6.4.3.1)
 *
 * @param Struct to store requested data.
 *
 * @return True get counts operation succeeded.
 */
bool BNO08xRpt::get_sample_counts(bno08x_sample_counts_t& sample_counts)
{
    int success = SH2_OK;
    sh2_Counts_t pCounts;

    lock_sh2_HAL();
    success = sh2_getCounts(ID, &pCounts);
    unlock_sh2_HAL();

    if (success != SH2_OK)
    {
        return false;
    }
    else
    {
        sample_counts = pCounts;
        return true;
    }
}

/**
 * @brief Clears BNO08x internal sample counts for this sensor. (see SH-2 ref manual 6.4.3.1)
 *
 * @return True clear counts operation succeeded.
 */
bool BNO08xRpt::clear_sample_counts()
{
    int success = SH2_OK;

    lock_sh2_HAL();
    success = sh2_clearCounts(ID);
    unlock_sh2_HAL();

    return (success == SH2_OK);
}

/**
 * @brief Retrieves meta data for this sensor/report by reading respective record in FRS (flash
 * record system).
 *
 * Can be used to retrieve the minimum period, maximum period, actual Q points, resolution, and
 * other info for a given sensor.
 *
 * @return True clear get meta data operation succeeded.
 */
bool BNO08xRpt::get_meta_data(bno08x_meta_data_t& meta_data)
{
    int success = SH2_OK;

    sh2_SensorMetadata_t sensor_meta_data;

    lock_sh2_HAL();
    success = sh2_getMetadata(ID, &sensor_meta_data);
    unlock_sh2_HAL();

    if (success == SH2_OK)
        meta_data = sensor_meta_data;

    return (success == SH2_OK);
}

/**
 * @brief Locks sh2 HAL lib to only allow the calling task to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::lock_sh2_HAL()
{
    xSemaphoreTake(sync_ctx->sh2_HAL_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks sh2 HAL lib to allow other tasks to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::unlock_sh2_HAL()
{
    xSemaphoreGive(sync_ctx->sh2_HAL_lock);
}

/**
 * @brief Locks locks user data to only allow the calling task to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::lock_user_data()
{
    xSemaphoreTake(sync_ctx->data_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks user data to allow other tasks to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::unlock_user_data()
{
    xSemaphoreGive(sync_ctx->data_lock);
}

/**
 * @brief Signals to BNO08x::data_available() that a new report has arrived.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::signal_data_available()
{
    xEventGroupSetBits(sync_ctx->evt_grp_rpt_data_available, rpt_bit);
    xEventGroupSetBits(sync_ctx->evt_grp_task, BNO08xPrivateTypes::EVT_GRP_BNO08x_TASK_DATA_AVAILABLE);
}
