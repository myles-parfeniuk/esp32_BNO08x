/**
 * @file BNO08xRptActivityClassifier.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptActivityClassifier
 *
 * @brief Class to represent activity classifier reports. (See Ref. Manual 6.5.36)
 */
class BNO08xRptActivityClassifier : public BNO08xRpt
{
    public:
        BNO08xRptActivityClassifier(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bool enable(
                uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg) override;
        bno08x_activity_classifier_t get();
        BNO08xActivity get_most_likely_activity();
        void set_activities_to_enable(BNO08xActivityEnable activities_to_enable);

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_activity_classifier_t data; ///< Most recent report data, doesn't account for step rollover.
        BNO08xActivityEnable activities_to_enable =
                BNO08xActivityEnable::ALL; ///< Activities to be monitored, call enable after setting.
        static const constexpr char* TAG = "BNO08xRptActivityClassifier";
};
