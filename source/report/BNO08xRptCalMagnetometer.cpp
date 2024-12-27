/**
 * @file BNO08xRptCalMagnetometer.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptCalMagnetometer.hpp"

/**
 * @brief Updates calibrated magnetometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptCalMagnetometer::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.magneticField;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables calibrated magnetometer reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptCalMagnetometer::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent calibrated magnetometer data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_t BNO08xRptCalMagnetometer::get()
{
    lock_user_data();
    bno08x_magf_t rqdata = data;
    unlock_user_data();
    return rqdata;
}