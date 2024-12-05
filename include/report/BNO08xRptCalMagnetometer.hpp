/**
 * @file BNO08xRptCalMagnetometer.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptCalMagnetometer
 *
 * @brief Class to represent calibrated magnetometer reports. (See Ref. Manual 6.5.16)
 */
class BNO08xRptCalMagnetometer : public BNO08xRpt
{
    public:
        BNO08xRptCalMagnetometer(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_magf_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_magf_t data;
        static const constexpr char* TAG = "BNO08xRptCalMagnetometer";
};
