/**
 * @file BNO08xRptLinearAcceleration.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent linear accelerometer reports. (See Ref. Manual 6.5.10)
 */
class BNO08xRptLinearAcceleration : public BNO08xRpt
{
    public:
        BNO08xRptLinearAcceleration(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bno08x_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_t data;
        static const constexpr char* TAG = "BNO08xRptLinearAcceleration";
};
