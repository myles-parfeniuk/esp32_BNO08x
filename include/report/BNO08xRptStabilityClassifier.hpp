/**
 * @file BNO08xRptStabilityClassifier.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptStabilityClassifier
 *
 * @brief Class to represent stability classifier reports. (See Ref. Manual 6.5.31)
 */
class BNO08xRptStabilityClassifier : public BNO08xRpt
{
    public:
        BNO08xRptStabilityClassifier(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bool enable(
                uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg) override;
        bno08x_stability_classifier_t get();
        BNO08xStability get_stability();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_stability_classifier_t data;
        static const constexpr char* TAG = "BNO08xRptStabilityClassifier";
};
