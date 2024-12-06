/**
 * @file BNO08xRptRawMEMSGyro.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptRawMEMSGyro
 *
 * @brief Class to represent raw gyro reports. (See Ref. Manual 6.5.12)
 */
class BNO08xRptRawMEMSGyro : public BNO08xRpt
{
    public:
        BNO08xRptRawMEMSGyro(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bno08x_raw_gyro_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_raw_gyro_t data;
        static const constexpr char* TAG = "BNO08xRptRawMEMSGyro";
};
