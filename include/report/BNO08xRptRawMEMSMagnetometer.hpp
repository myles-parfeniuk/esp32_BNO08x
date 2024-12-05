/**
 * @file BNO08xRptRawMEMSMagnetometer.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptRawMEMSMagnetometer
 *
 * @brief Class to represent raw magnetometer reports. (See Ref. Manual 6.5.15)
 */
class BNO08xRptRawMEMSMagnetometer : public BNO08xRpt
{
    public:
        BNO08xRptRawMEMSMagnetometer(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_raw_magf_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_raw_magf_t data;
        static const constexpr char* TAG = "BNO08xRptRawMEMSMagnetometer";
};
