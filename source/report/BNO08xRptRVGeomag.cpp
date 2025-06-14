/**
 * @file BNO08xRptRVGeomag.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptRVGeomag.hpp"

/**
 * @brief Updates geomagnetic rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptRVGeomag::update_data(sh2_SensorValue_t* sensor_val)
{
    BNO08xGuard::lock_user_data(sync_ctx);
    data = sensor_val->un.geoMagRotationVector;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    BNO08xGuard::unlock_user_data(sync_ctx);

    signal_data_available();
}

/**
 * @brief Tares geomagnetic rotation vector.
 *
 * @param x If true tare x axis (optional, default true).
 * @param y If true tare y axis (optional, default true).
 * @param z If true tare z axis (optional, default true).
 *
 * @return True if tare operation succeeded.
 */
bool BNO08xRptRVGeomag::tare(bool x, bool y, bool z)
{
    return BNO08xRptRVGeneric::tare(x, y, z, SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR);
}

/**
 * @brief Saves most recent tare operation to BNO08x internal flash, such that it persists on reset.
 *
 * @return True if tare operation succeeded.
 */
bool BNO08xRptRVGeomag::tare_persist()
{
    int success = SH2_ERR;

    BNO08xGuard::lock_sh2_HAL(sync_ctx);
    success = sh2_persistTare();
    BNO08xGuard::unlock_sh2_HAL(sync_ctx);

    if (success != SH2_OK)
        return false;
    else
        return true;
}

/**
 * @brief Clears most recent tare operation.
 *
 * @return void, nothing to return
 */
void BNO08xRptRVGeomag::tare_clear()
{
    BNO08xGuard::lock_sh2_HAL(sync_ctx);
    sh2_clearTare();
    BNO08xGuard::unlock_sh2_HAL(sync_ctx);
}