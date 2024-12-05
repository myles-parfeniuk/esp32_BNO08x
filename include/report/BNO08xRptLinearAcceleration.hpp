/**
 * @file BNO08xRptLinearAcceleration.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptLinearAcceleration
 *
 * @brief Class to represent linear accelerometer reports. (See Ref. Manual 6.5.10)
 */
class BNO08xRptLinearAcceleration : public BNO08xRpt
{
    public:
        BNO08xRptLinearAcceleration(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_t data;
        static const constexpr char* TAG = "BNO08xRptLinearAcceleration";
};
