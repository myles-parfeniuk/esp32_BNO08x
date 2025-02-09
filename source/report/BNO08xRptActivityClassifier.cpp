/**
 * @file BNO08xRptActivityClassifier.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptActivityClassifier.hpp"

/**
 * @brief Updates activity classifier data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptActivityClassifier::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.personalActivityClassifier;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    update_timestamp(sensor_val);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables activity classifier reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptActivityClassifier::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.sensorSpecific = static_cast<uint8_t>(activities_to_enable); // this must be set regardless of user cfg
                                                                            // or no reports will be received
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent activity classifier data.
 *
 * @return Struct containing requested data.
 */
bno08x_activity_classifier_t BNO08xRptActivityClassifier::get()
{
    lock_user_data();
    bno08x_activity_classifier_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most the most likely activity from most recent activity classifier data.
 *
 * @return BNO08xActivity object with detected state.
 */
BNO08xActivity BNO08xRptActivityClassifier::get_most_likely_activity()
{
    lock_user_data();
    BNO08xActivity rqdata = static_cast<BNO08xActivity>(data.mostLikelyState);
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Sets the activities to be monitored for with ActivityClassifier reports, all enable after setting.
 *
 * @param activities_to_enable The activities to be monitored with sent reports.
 *
 * @return void, nothing to return
 */
void BNO08xRptActivityClassifier::set_activities_to_enable(BNO08xActivityEnable activities_to_enable)
{
    this->activities_to_enable = activities_to_enable;
}