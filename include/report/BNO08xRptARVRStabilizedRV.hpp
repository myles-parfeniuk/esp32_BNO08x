/**
 * @file BNO08xRptARVRStabilizedRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptARVRStabilizedRV
 *
 * @brief Class to represent ARVR stabilized rotation vector reports. (See Ref. Manual 6.5.42)
 */
class BNO08xRptARVRStabilizedRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptARVRStabilizedRV(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRptRVGeneric(ID, rpt_bit, sync_ctx)
        {
        }

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptARVRStabilizedRV";
};
