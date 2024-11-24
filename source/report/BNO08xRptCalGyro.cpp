#include "BNO08xRptCalGyro.hpp" 
#include "BNO08x.hpp"

/**
 * @brief Updates calibrated gyro data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptCalGyro::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.gyroscope;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent gyroscope data (velocity), units are in rad/s.
 *
 * @return Struct containing requested data.
 */
bno08x_gyro_t BNO08xRptCalGyro::get()
{
    imu->lock_user_data();
    bno08x_gyro_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}