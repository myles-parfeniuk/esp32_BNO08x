#include "BNO08xRptARVRStabilizedRV.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates ARVR stabilized rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptARVRStabilizedRV::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.arvrStabilizedRV;
    imu->unlock_user_data();
}