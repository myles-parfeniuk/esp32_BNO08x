#include "BNO08xShakeDetector.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates shake detector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xShakeDetector::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.shakeDetector;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent shake detector detector data.
 *
 * @return Struct containing the requested data.
 */
bno08x_shake_detector_t BNO08xShakeDetector::get()
{
    imu->lock_user_data();
    bno08x_shake_detector_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}