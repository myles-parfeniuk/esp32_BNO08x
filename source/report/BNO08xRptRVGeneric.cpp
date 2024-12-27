/**
 * @file BNO08xRptRVGeneric.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptRVGeneric.hpp"

/**
 * @brief Enables a rotation vector report such that the BNO08x begins it.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptRVGeneric::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent rotation vector data in form of unit quaternion, rad accuracy units in
 * radians (if available, else constant 0.0f).
 *
 * The following RV reports have rad accuracy data:
 *
 * - rotation vector
 * - geomagnetic rotation vector
 *
 *
 * @return Struct containing requested data.
 */
bno08x_quat_t BNO08xRptRVGeneric::get_quat()
{
    lock_user_data();
    bno08x_quat_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent rotation vector data in form of an euler angle, units are in degrees or
 * rads.
 *
 * @param in_degrees If true returned euler angle is in degrees, if false in radians
 *
 * @return Struct containing requested data.
 */
bno08x_euler_angle_t BNO08xRptRVGeneric::get_euler(bool in_degrees)
{
    bno08x_euler_angle_t rqdata;
    bno08x_quat_t quat = get_quat();

    rqdata = quat; // conversion handled by overloaded operator

    // convert to degrees if requested
    if (in_degrees)
        rqdata *= RAD_2_DEG;

    return rqdata;
}

/**
 * @brief Tares vector basis according to axis flags.
 *
 * @param x If true tare x axis.
 * @param y If true tare y axis.
 * @param z If true tare z axis.
 * @param basis Rotation vector basis to undergo tare operation.
 *
 * @return True if tare operation succeeded.
 */
bool BNO08xRptRVGeneric::tare(bool x, bool y, bool z, sh2_TareBasis_t basis)
{
    int success = SH2_ERR;

    uint8_t axis_flag = 0U;

    if (x)
        axis_flag |= SH2_TARE_X;

    if (y)
        axis_flag |= SH2_TARE_Y;

    if (z)
        axis_flag |= SH2_TARE_Z;

    lock_sh2_HAL();
    success = sh2_setTareNow(axis_flag, basis);
    unlock_sh2_HAL();

    if (success != SH2_OK)
        return false;
    else
        return true;
}
