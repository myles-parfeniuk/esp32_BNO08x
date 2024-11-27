
/**
 * @file BNO08xRptUncalMagnetometer.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent uncalibrated magnetometer reports. (See Ref. Manual 6.5.17)
 */
class BNO08xRptUncalMagnetometer : public BNO08xRpt
{
    public:
        BNO08xRptUncalMagnetometer(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        void get(bno08x_magf_t& magf, bno08x_magf_bias_t& bias);
        bno08x_magf_t get_magf();
        bno08x_magf_bias_t get_bias();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_magf_t data;
        bno08x_magf_bias_t bias_data;
        static const constexpr char* TAG = "BNO08xRptUncalMagnetometer";
};
