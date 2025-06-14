/**
 * @file BNO08xRptGameRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptGameRV
 *
 * @brief Class to represent game rotation vector reports. (See Ref. Manual 6.5.19)
 */
class BNO08xRptGameRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptGameRV(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRptRVGeneric(ID, rpt_bit, sync_ctx)
        {
        }

        bool tare(bool x = true, bool y = true, bool z = true);
        bool tare_persist();
        void tare_clear();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptGameRV";
};
