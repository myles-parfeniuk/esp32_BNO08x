/**
 * @file BNO08xRptRawMEMSAccelerometer.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptRawMEMSAccelerometer
 *
 * @brief Class to represent raw accelerometer reports. (See Ref. Manual 6.5.8)
 */
class BNO08xRptRawMEMSAccelerometer : public BNO08xRpt
{
    public:
        BNO08xRptRawMEMSAccelerometer(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_raw_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_raw_accel_t data;
        static const constexpr char* TAG = "BNO08xRptRawMEMSAccelerometer";
};
