
/**
 * @file BNO08xRptUncalMagnetometer.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptUncalMagnetometer
 *
 * @brief Class to represent uncalibrated magnetometer reports. (See Ref. Manual 6.5.17)
 */
class BNO08xRptUncalMagnetometer : public BNO08xRpt
{
    public:
        BNO08xRptUncalMagnetometer(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
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
