#include "BNO08xRptRawMEMSGyro.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates raw mems gyro data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptRawMEMSGyro::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.rawGyroscope;
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent raw mems gyro report (units in ADC counts, time_stamp in microseconds)
 *
 * @return Struct containing requested data.
 */
bno08x_raw_gyro_t BNO08xRptRawMEMSGyro::get()
{
    imu->lock_user_data();
    bno08x_raw_gyro_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}