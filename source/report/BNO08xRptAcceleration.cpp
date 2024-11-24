#include "BNO08xRptAcceleration.hpp" 
#include "BNO08x.hpp"

/**
 * @brief Updates accelerometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptAcceleration::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.accelerometer;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent acceleration data (including gravity), units are in m/s^2.
 *
 * @return Struct containing requested data.
 */
bno08x_accel_t BNO08xRptAcceleration::get()
{
    imu->lock_user_data();
    bno08x_accel_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}