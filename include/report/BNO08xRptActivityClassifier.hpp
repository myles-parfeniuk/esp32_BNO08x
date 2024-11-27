/**
 * @file BNO08xRptActivityClassifier.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent activity classifier reports. (See Ref. Manual 6.5.36)
 */
class BNO08xRptActivityClassifier : public BNO08xRpt
{
    public:
        BNO08xRptActivityClassifier(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bool enable(uint32_t time_between_reports, BNO08xActivityEnable activities_to_enable, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bno08x_activity_classifier_t get();
        BNO08xActivity get_most_likely_activity();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_activity_classifier_t data; ///< Most recent report data, doesn't account for step rollover.
        static const constexpr char* TAG = "BNO08xRptActivityClassifier";
};
