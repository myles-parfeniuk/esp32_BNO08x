#include "BNO08xTapDetector.hpp"
#include "BNO08x.hpp"

/**
 * @brief Enables tap detector reports such that the BNO08x begins sending them (only sends reports when a tap is detected).
 *
 * @param time_between_reports The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see BNO08xRpt::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xTapDetector::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    sensor_cfg.changeSensitivityEnabled = true; // this must be set regardless of user cfg or no reports will be received
    sensor_cfg.changeSensitivity = 0U;          // this must be set regardless of user cfg or no reports will be received

    return BNO08xRpt::enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Updates tap detector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xTapDetector::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.tapDetector;
    imu->unlock_user_data();
}

/**
 * @brief Grabs most recent tap detector detector data.
 *
 * @return Struct containing requested data;
 */
bno08x_tap_detector_t BNO08xTapDetector::get()
{
    imu->lock_user_data();
    bno08x_tap_detector_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}