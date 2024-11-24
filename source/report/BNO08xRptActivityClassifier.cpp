#include "BNO08xRptActivityClassifier.hpp"
#include "BNO08x.hpp"

/**
 * @brief Enables activity classifier reports such that the BNO08x begins sending them.
 *
 * @param time_between_reports The period/interval of the report in microseconds.
 * @param activities_to_enable Which activities to enable.
 * @param sensor_cfg Sensor special configuration (optional, see BNO08xRpt::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptActivityClassifier::enable(uint32_t time_between_reports, BNO08xActivityEnable activities_to_enable, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.sensorSpecific = static_cast<uint8_t>(activities_to_enable); // this must be set regardless of user cfg or no reports will be received

    return BNO08xRpt::enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Updates activity classifier data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptActivityClassifier::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.personalActivityClassifier;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent activity classifier data.
 *
 * @return Struct containing requested data.
 */
bno08x_activity_classifier_t BNO08xRptActivityClassifier::get()
{
    imu->lock_user_data();
    bno08x_activity_classifier_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most the most likely activity from most recent activity classifier data.
 *
 * @return BNO08xActivity object with detected state.
 */
BNO08xActivity BNO08xRptActivityClassifier::get_most_likely_activity()
{
    imu->lock_user_data();
    BNO08xActivity rqdata = static_cast<BNO08xActivity>(data.mostLikelyState);
    imu->unlock_user_data();
    return rqdata;
}