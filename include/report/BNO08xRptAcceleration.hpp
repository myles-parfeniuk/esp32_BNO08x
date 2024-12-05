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
        BNO08xRptAcceleration(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_t data;
        static const constexpr char* TAG = "BNO08xRptAcceleration";
};
