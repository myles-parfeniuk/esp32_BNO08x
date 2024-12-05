/**
 * @file BNO08xStabilityClassifier.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xStabilityClassifier.hpp"

/**
 * @brief Updates stability classifier data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xStabilityClassifier::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.stabilityClassifier;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(*_evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent stability classifier data.
 *
 * @return BNO08xStability enum object with detected state.
 */
bno08x_stability_classifier_t BNO08xStabilityClassifier::get()
{
    lock_user_data();
    bno08x_stability_classifier_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent stability classifier reading (excludes accuracy)
 *
 * @return BNO08xStability enum object with detected state.
 */
BNO08xStability BNO08xStabilityClassifier::get_stability()
{
    lock_user_data();
    BNO08xStability rqdata = data.stability;
    unlock_user_data();
    return rqdata;
}
