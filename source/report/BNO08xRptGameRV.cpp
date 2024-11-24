#include "BNO08xRptRV.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates game rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptGameRV::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.gameRotationVector;
    imu->unlock_user_data();
}