#pragma once

#include "BNO08xRpt.hpp" // Include the base class header

/**
 * @brief Class to represent accelerometer reports. (See Ref. Manual 6.5.9)
 */
class BNO08xRptAcceleration : public BNO08xRpt
{
    public:
        // Constructor declaration
        BNO08xRptAcceleration(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bno08x_accel_data_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_data_t data;
        static const constexpr char* TAG = "BNO08xRptAcceleration";
};
