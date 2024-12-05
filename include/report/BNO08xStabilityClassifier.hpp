/**
 * @file BNO08xStabilityClassifier.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xStabilityClassifier
 *
 * @brief Class to represent stability classifier reports. (See Ref. Manual 6.5.31)
 */
class BNO08xStabilityClassifier : public BNO08xRpt
{
    public:
        BNO08xStabilityClassifier(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_stability_classifier_t get();
        BNO08xStability get_stability();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_stability_classifier_t data;
        static const constexpr char* TAG = "BNO08xStabilityClassifier";
};
