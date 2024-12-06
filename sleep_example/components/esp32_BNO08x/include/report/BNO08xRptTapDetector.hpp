/**
 * @file BNO08xRptTapDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptTapDetector
 *
 * @brief Class to represent tap detector reports. (See Ref. Manual 6.5.27)
 */
class BNO08xRptTapDetector : public BNO08xRpt
{
    public:
        BNO08xRptTapDetector(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bool enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        bno08x_tap_detector_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_tap_detector_t data;
        static const constexpr char* TAG = "BNO08xRptTapDetector";
};