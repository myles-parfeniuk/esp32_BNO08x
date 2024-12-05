/**
 * @file BNO08xRptCalGyro.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptCalGyro
 *
 * @brief Class to represent calibrated gyro reports. (See Ref. Manual 6.5.13)
 */
class BNO08xRptCalGyro : public BNO08xRpt
{
    public:
        BNO08xRptCalGyro(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_gyro_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_gyro_t data;
        static const constexpr char* TAG = "BNO08xRptCalGyro";
};
