#include "BNO08xRptGameRV.hpp"
#include "BNO08x.hpp"

/**
 * @brief Updates game rotation vector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptGameRV::update_data(sh2_SensorValue_t* sensor_val)
{
    imu->lock_user_data();
    data = sensor_val->un.gameRotationVector;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Tares game rotation vector axis.
 *
 * @param x If true tare x axis (optional, default true).
 * @param y If true tare y axis (optional, default true).
 * @param z If true tare z axis (optional, default true).
 *
 * @return True if tare operation succeeded.
 */
bool BNO08xRptGameRV::tare(bool x, bool y, bool z)
{

    return BNO08xRptRVGeneric::tare(x, y, z, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
}

/**
 * @brief Saves most recent tare operation to BNO08x internal flash, such that it persists on reset.
 *
 * @return True if tare operation succeeded.
 */
bool BNO08xRptGameRV::tare_persist()
{
    int success = SH2_ERR;

    imu->lock_sh2_HAL();
    success = sh2_persistTare();
    imu->unlock_sh2_HAL();

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
void BNO08xRptGameRV::tare_clear()
{
    imu->lock_sh2_HAL();
    sh2_clearTare();
    imu->unlock_sh2_HAL();
}