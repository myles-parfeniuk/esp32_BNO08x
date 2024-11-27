/**
 * @file BNO08xRptCalMagnetometer.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptCalMagnetometer.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates calibrated magnetometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptCalMagnetometer::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.magneticField;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent calibrated magnetometer data, units are in uTesla.
 *
 * @return Struct containing requested data.
 */
bno08x_magf_t BNO08xRptCalMagnetometer::get()
{
    imu->lock_user_data();
    bno08x_magf_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}