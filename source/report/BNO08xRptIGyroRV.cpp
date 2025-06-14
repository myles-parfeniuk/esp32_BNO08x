/**
 * @file BNO08xRptIGyroRV.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptIGyroRV.hpp"

/**
 * @brief Updates gyro integrated rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptIGyroRV::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.gyroIntegratedRV;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    data_vel = sensor_val->un.gyroIntegratedRV;
    BNO08xGuard::unlock_user_data(sync_ctx);

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent gyro integrated rotation vector data.
 *
 * @param quat Struct to store requested unit quaternion data.
 * @param vel Struct to store requested velocity data (units in rad/s).
 *
 * @return void, nothing to return
 */
void BNO08xRptIGyroRV::get(bno08x_quat_t& quat, bno08x_ang_vel_t& vel)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    quat = data;
    vel = data_vel;
    BNO08xGuard::unlock_user_data(sync_ctx);
}

/**
 * @brief Grabs most recent gyro integrated rotation vector angular velocity data, units are in
 * rad/s.
 *
 * @return Struct containing requested data.
 */
bno08x_ang_vel_t BNO08xRptIGyroRV::get_vel()
{
    BNO08xGuard::lock_user_data(sync_ctx);
    bno08x_ang_vel_t rqdata = data_vel;
    BNO08xGuard::unlock_user_data(sync_ctx);
    return rqdata;
}