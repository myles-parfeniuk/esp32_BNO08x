/**
 * @file BNO08xRptARVRStabilizedRV.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptARVRStabilizedRV.hpp"

/**
 * @brief Updates ARVR stabilized rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptARVRStabilizedRV::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.arvrStabilizedRV;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    update_timestamp(sensor_val);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}