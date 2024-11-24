#include "BNO08xRptRawMEMSMagnetometer.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates raw magnetometer data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptRawMEMSMagnetometer::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.rawMagnetometer;
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent raw magnetometer data, units are ADC counts, time_stamp in microseconds.
 *
 * @return Struct containing requested data.
 */
bno08x_raw_magf_t BNO08xRptRawMEMSMagnetometer::get()
{
    imu->lock_user_data();
    bno08x_raw_magf_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}