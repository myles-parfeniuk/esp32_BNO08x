#include "BNO08xRptUncalMagnetometer.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates uncalibrated magf data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptUncalMagnetometer::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.magneticFieldUncal;
    bias_data = sensor_val->un.magneticFieldUncal;
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
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
    imu->lock_user_data();
    magf = data;
    bias = bias_data;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent uncalibrated magnetometer magnetic field data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_t BNO08xRptUncalMagnetometer::get_magf()
{
    imu->lock_user_data();
    bno08x_magf_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent uncalibrated magnetometer bias data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_bias_t BNO08xRptUncalMagnetometer::get_bias()
{
    imu->lock_user_data();
    bno08x_magf_bias_t rqdata = bias_data;
    imu->unlock_user_data();
    return rqdata;
}