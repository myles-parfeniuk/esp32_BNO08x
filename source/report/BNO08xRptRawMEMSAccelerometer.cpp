#include "BNO08xRptRawMEMSAccelerometer.hpp" 
#include "BNO08x.hpp"

/**
 * @brief Updates raw accelerometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptRawMEMSAccelerometer::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.rawAccelerometer;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent raw accelerometer data, units are ADC counts, time_stamp in microseconds.
 *
 * @return Struct containing requested data.
 */
bno08x_raw_accel_t BNO08xRptRawMEMSAccelerometer::get()
{
    imu->lock_user_data();
    bno08x_raw_accel_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}