/**
 * @file BNO08xRptShakeDetector.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptShakeDetector.hpp"

/**
 * @brief Updates shake detector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptShakeDetector::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.shakeDetector;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    update_timestamp(sensor_val);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables shake detector reports such that the BNO08x begins sending them (only sends reports
 * when a shake is detected).
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptShakeDetector::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.changeSensitivityEnabled = true; // this must be set regardless of user cfg or no reports will be received
    sensor_cfg.changeSensitivity = 0U;          // this must be set regardless of user cfg or no reports will be received

    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent shake detector detector data.
 *
 * @return Struct containing the requested data.
 */
bno08x_shake_detector_t BNO08xRptShakeDetector::get()
{
    lock_user_data();
    bno08x_shake_detector_t rqdata = data;
    unlock_user_data();
    return rqdata;
}