/**
 * @file BNO08xRptTapDetector.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptTapDetector.hpp"

/**
 * @brief Enables tap detector reports such that the BNO08x begins sending them (only sends reports
 * when a tap is detected).
 *
 * @param time_between_reports The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptTapDetector::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.changeSensitivityEnabled = true; // this must be set regardless of user cfg or no reports will be received
    sensor_cfg.changeSensitivity = 0U;          // this must be set regardless of user cfg or no reports will be received

    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Updates tap detector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptTapDetector::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.tapDetector;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    BNO08xGuard::unlock_user_data(sync_ctx);

    signal_data_available();
}

/**
 * @brief Grabs most recent tap detector detector data.
 *
 * @return Struct containing requested data;
 */
bno08x_tap_detector_t BNO08xRptTapDetector::get()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_tap_detector_t rqdata = data;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}