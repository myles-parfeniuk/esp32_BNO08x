#include "BNO08xRptGravity.hpp" // Include the header file for the class
#include "BNO08x.hpp"

/**
 * @brief Updates gravity data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptGravity::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.gravity;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent gravity data, units are in m/s^2.
 *
 * @return Struct containing requested data.
 */
bno08x_accel_data_t BNO08xRptGravity::get()
{
    imu->lock_user_data();
    bno08x_accel_data_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}