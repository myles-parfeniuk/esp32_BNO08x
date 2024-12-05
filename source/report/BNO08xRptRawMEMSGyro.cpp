/**
 * @file BNO08xRptRawMEMSGyro.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptRawMEMSGyro.hpp"

/**
 * @brief Updates raw mems gyro data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptRawMEMSGyro::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.rawGyroscope;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent raw mems gyro report (units in ADC counts, time_stamp in microseconds)
 *
 * @return Struct containing requested data.
 */
bno08x_raw_gyro_t BNO08xRptRawMEMSGyro::get()
{
    lock_user_data();
    bno08x_raw_gyro_t rqdata = data;
    unlock_user_data();
    return rqdata;
}