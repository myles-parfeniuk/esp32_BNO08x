/**
 * @file BNO08xRptARVRStabilizedGameRV.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptARVRStabilizedGameRV.hpp"

/**
 * @brief Updates ARVR stabilized game rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptARVRStabilizedGameRV::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.arvrStabilizedGRV;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    BNO08xGuard::unlock_user_data(sync_ctx);

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}