/**
 * @file BNO08xRptShakeDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptShakeDetector
 *
 * @brief Class to represent shake detector reports. (See Ref. Manual 6.5.32)
 */
class BNO08xRptShakeDetector : public BNO08xRpt
{
    public:
        BNO08xRptShakeDetector(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bno08x_shake_detector_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_shake_detector_t data;
        static const constexpr char* TAG = "BNO08xRptShakeDetector";
};