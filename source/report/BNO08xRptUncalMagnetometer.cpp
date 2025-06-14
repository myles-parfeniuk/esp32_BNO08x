/**
 * @file BNO08xRptUncalMagnetometer.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptUncalMagnetometer.hpp"

/**
 * @brief Updates uncalibrated magf data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptUncalMagnetometer::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.magneticFieldUncal;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    bias_data = sensor_val->un.magneticFieldUncal;
    BNO08xGuard::unlock_user_data(sync_ctx);

    if (rpt_bit & xEventGroupGetBits(sync_ctx.evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables uncalibrated magnetometer reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptUncalMagnetometer::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent uncalibrated magnetometer data, units are in uTesla.
 *
 * @param magf Struct to store requested magf data.
 * @param bias Struct to store requested bias data.
 *
 * @return void, nothing to return
 */
void BNO08xRptUncalMagnetometer::get(bno08x_magf_t& magf, bno08x_magf_bias_t& bias)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    magf = data;
    bias = bias_data;
    BNO08xGuard::unlock_user_data(sync_ctx);
}

/**
 * @brief Grabs most recent uncalibrated magnetometer magnetic field data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_t BNO08xRptUncalMagnetometer::get_magf()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_magf_t rqdata = data;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}

/**
 * @brief Grabs most recent uncalibrated magnetometer bias data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_bias_t BNO08xRptUncalMagnetometer::get_bias()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_magf_bias_t rqdata = bias_data;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}