/**
 * @file BNO08xRptAcceleration.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptAcceleration
 *
 * @brief Class to represent accelerometer reports. (See Ref. Manual 6.5.9)
 */
class BNO08xRptAcceleration : public BNO08xRpt
{
    public:
        BNO08xRptAcceleration(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bno08x_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_t data;
        static const constexpr char* TAG = "BNO08xRptAcceleration";
};
