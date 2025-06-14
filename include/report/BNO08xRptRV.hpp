/**
 * @file BNO08xRptRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptRV
 *
 * @brief Class to represent rotation vector reports. (See Ref. Manual 6.5.18)
 */
class BNO08xRptRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptRV(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRptRVGeneric(ID, rpt_bit, sync_ctx)
        {
        }

        bool tare(bool x = true, bool y = true, bool z = true);
        bool tare_persist();
        void tare_clear();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptRV";
};
