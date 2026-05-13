/**
 * @file BNO08xRptSignificantMotion.cpp
 * @author Extended from Myles Parfeniuk's work
 */

#include "BNO08xRptSignificantMotion.hpp"

/**
 * @brief Updates significant motion data from decoded sensor event.
 *
 * Significant motion is a one-shot sensor. When motion is detected, it sends
 * one report with motion value and then automatically disables itself.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptSignificantMotion::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.sigMotion;
    unlock_user_data();

    // Signal that data is available (motion was detected)
    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables significant motion detector reports.
 *
 * This arms the one-shot significant motion detector. The sensor will monitor
 * for significant motion and send a single report when detected, then disable itself.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 *        For one-shot sensors, this parameter typically doesn't affect behavior.
 * @param sensor_cfg Sensor special configuration (optional).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptSignificantMotion::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.changeSensitivityEnabled = true; // this must be set regardless of user cfg or no reports will be received
    sensor_cfg.changeSensitivity = 0U;          // this must be set regardless of user cfg or no reports will be received

    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent significant motion detector data.
 *
 * @return Struct containing the requested data.
 */
bno08x_significant_motion_t BNO08xRptSignificantMotion::get()
{
    lock_user_data();
    bno08x_significant_motion_t rqdata = data;
    unlock_user_data();
    return rqdata;
}
