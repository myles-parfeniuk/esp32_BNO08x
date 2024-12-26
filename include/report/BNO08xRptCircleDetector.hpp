/**
 * @file BNO08xRptCircleDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptCircleDetector
 *
 * @brief Class to represent circle detector reports. (See Ref. Manual 6.40.2)
 */
class BNO08xRptCircleDetector : public BNO08xRpt
{
    public:
        BNO08xRptCircleDetector(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bool enable(
                uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg) override;
        uint16_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        uint16_t data; ///< Total amount of circles detected.
        static const constexpr char* TAG = "BNO08xRptCircleDetector";
};
