#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent stability classifier reports. (See Ref. Manual 6.5.31)
 */
class BNO08xStabilityClassifier : public BNO08xRpt
{
    public:
        BNO08xStabilityClassifier(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        BNO08xStability get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        sh2_StabilityClassifier_t data = {0U};
        static const constexpr char* TAG = "BNO08xStabilityClassifier";
};
