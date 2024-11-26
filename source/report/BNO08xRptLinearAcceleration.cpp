#include "BNO08xRptLinearAcceleration.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates accelerometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptLinearAcceleration::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.linearAcceleration;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent acceleration data (including gravity), units are in m/s^2.
 *
 * @return Struct containing requested data.
 */
bno08x_accel_t BNO08xRptLinearAcceleration::get()
{
    imu->lock_user_data();
    bno08x_accel_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}