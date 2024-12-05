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
    lock_user_data();
    data = sensor_val->un.magneticFieldUncal;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    bias_data = sensor_val->un.magneticFieldUncal;
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
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
    lock_user_data();
    magf = data;
    bias = bias_data;
    unlock_user_data();
}

/**
 * @brief Grabs most recent uncalibrated magnetometer magnetic field data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_t BNO08xRptUncalMagnetometer::get_magf()
{
    lock_user_data();
    bno08x_magf_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent uncalibrated magnetometer bias data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_bias_t BNO08xRptUncalMagnetometer::get_bias()
{
    lock_user_data();
    bno08x_magf_bias_t rqdata = bias_data;
    unlock_user_data();
    return rqdata;
}