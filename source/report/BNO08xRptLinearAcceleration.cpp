#include "BNO08xRptLinearAcceleration.hpp" // Include the header file for the class
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
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent acceleration data (including gravity), units are in m/s^2.
 *
 * @return Struct containing requested data.
 */
bno08x_accel_data_t BNO08xRptLinearAcceleration::get()
{
    imu->lock_user_data();
    bno08x_accel_data_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}