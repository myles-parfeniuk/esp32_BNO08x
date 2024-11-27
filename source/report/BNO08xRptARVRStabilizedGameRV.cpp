/**
 * @file BNO08xRptARVRStabilizedGameRV.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptARVRStabilizedGameRV.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates ARVR stabilized game rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptARVRStabilizedGameRV::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.arvrStabilizedGRV;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}