#include "BNO08xRptRVGeneric.hpp"
#include "BNO08x.hpp"

/**
 * @brief Grabs most recent rotation vector data in form of unit quaternion, accuracy units in radians (if available).
 *
 * The following RV reports have accuracy data:
 *
 * - rotation vector
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
