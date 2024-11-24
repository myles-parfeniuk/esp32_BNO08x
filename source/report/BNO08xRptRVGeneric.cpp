#include "BNO08xRptRVGeneric.hpp"
#include "BNO08x.hpp"

/**
 * @brief Grabs most recent rotation vector data in form of unit quaternion, accuracy units in radians (if available, else constant 0.0f).
 *
 * The following RV reports have accuracy data:
 *
 * - rotation vector
 * - geomagnetic rotation vector
 *
 *
 * @return Struct containing requested data.
 */
bno08x_quat_t BNO08xRptRVGeneric::get_quat()
{
    imu->lock_user_data();
    bno08x_quat_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent rotation vector data in form of an euler angle, units are in degrees or rads.
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

    imu->lock_sh2_HAL();
    success = sh2_setTareNow(axis_flag, basis);
    imu->unlock_sh2_HAL();

    if (success != SH2_OK)
        return false;
    else
        return true;
}
