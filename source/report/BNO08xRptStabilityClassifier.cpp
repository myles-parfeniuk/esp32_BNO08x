/**
 * @file BNO08xRptStabilityClassifier.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptStabilityClassifier.hpp"

/**
 * @brief Updates stability classifier data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptStabilityClassifier::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.stabilityClassifier;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    BNO08xGuard::unlock_user_data(sync_ctx);

    signal_data_available();
}

/**
 * @brief Enables stability classifier reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptStabilityClassifier::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent stability classifier data.
 *
 * @return BNO08xStability enum object with detected state.
 */
bno08x_stability_classifier_t BNO08xRptStabilityClassifier::get()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_stability_classifier_t rqdata = data;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}

/**
 * @brief Grabs most recent stability classifier reading (excludes accuracy)
 *
 * @return BNO08xStability enum object with detected state.
 */
BNO08xStability BNO08xRptStabilityClassifier::get_stability()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    BNO08xStability rqdata = data.stability;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}
