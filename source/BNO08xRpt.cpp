#include "BNO08xRpt.hpp"
#include "BNO08x.hpp"

/**
 * @brief Enables a sensor report such that the BNO08x begins sending it.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see BNO08xRpt::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRpt::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    int sh2_res = SH2_OK;

    xEventGroupClearBits(imu->evt_grp_report_en, rpt_bit); // Set the event group bit

    imu->lock_sh2_HAL();
    sensor_cfg.reportInterval_us = time_between_reports;
    sh2_res = sh2_setSensorConfig(ID, &sensor_cfg);
    imu->unlock_sh2_HAL();

    if (sh2_res != SH2_OK)
    {
        return false;
    }
    else
    {
        flush();
        period_us = time_between_reports;                    // Update the period
        xEventGroupSetBits(imu->evt_grp_report_en, rpt_bit); // Set the event group bit
        return true;
    }
}

/**
 * @brief Disables a sensor report by setting its period to 0us such that the BNO08x stops sending it.
 *
 * @param sensor_ID The ID of the sensor for the respective report to be disabled.
 * @param sensor_cfg Sensor special configuration.
 *
 * @return ESP_OK if report was successfully disabled.
 */
bool BNO08xRpt::disable(sh2_SensorConfig_t sensor_cfg)
{
    EventBits_t evt_grp_report_en_bits = xEventGroupGetBits(imu->evt_grp_report_en);

    if (evt_grp_report_en_bits & rpt_bit)
    {
        if (!enable(0UL, sensor_cfg))
            return false;
        else
            xEventGroupClearBits(imu->evt_grp_report_en, rpt_bit); // Set the event group bit
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
void BNO08xRpt::register_cb(std::function<void(void)> cb_fxn)
{
    imu->cb_list.push_back({ID, cb_fxn});
}

/**
 * @brief Flush all buffered reports for this sensor/report module.
 *
 * @return True if flush operation succeeded.
 */
bool BNO08xRpt::flush()
{
    int success = SH2_OK;

    imu->lock_sh2_HAL();
    success = sh2_flush(ID);
    imu->unlock_sh2_HAL();

    return (success != SH2_OK) ? false : true;
}

/**
 * @brief Signals to BNO08x::data_available() that a new report has arrived.
 *
 * @return void, nothing to return
 */
void BNO08xRpt::signal_data_available()
{
    xEventGroupClearBits(imu->evt_grp_bno08x_task, BNO08x::EVT_GRP_BNO08x_TASK_DATA_AVAILABLE);
    imu->most_recent_rpt = ID;
    xEventGroupSetBits(imu->evt_grp_bno08x_task, BNO08x::EVT_GRP_BNO08x_TASK_DATA_AVAILABLE);
}
