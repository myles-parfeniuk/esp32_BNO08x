#include "BNO08xStabilityClassifier.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates stability classifier data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xStabilityClassifier::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.stabilityClassifier;
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent stability classifier data.
 *
 * @return BNO08xStability enum object with detected state.
 */
BNO08xStability BNO08xStabilityClassifier::get()
{
    imu->lock_user_data();
    BNO08xStability rqdata = static_cast<BNO08xStability>(data.classification);
    imu->unlock_user_data();
    return rqdata;
}