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
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.rawGyroscope;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    BNO08xGuard::unlock_user_data(sync_ctx);

    if (rpt_bit & xEventGroupGetBits(sync_ctx.evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables raw gyro reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptRawMEMSGyro::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent raw mems gyro report (units in ADC counts, time_stamp in microseconds)
 *
 * @return Struct containing requested data.
 */
bno08x_raw_gyro_t BNO08xRptRawMEMSGyro::get()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_raw_gyro_t rqdata = data;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}