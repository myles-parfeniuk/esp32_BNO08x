/**
 * @file BNO08xRptRawMEMSGyro.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptRawMEMSGyro
 *
 * @brief Class to represent raw gyro reports. (See Ref. Manual 6.5.12)
 */
class BNO08xRptRawMEMSGyro : public BNO08xRpt
{
    public:
        BNO08xRptRawMEMSGyro(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_raw_gyro_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_raw_gyro_t data;
        static const constexpr char* TAG = "BNO08xRptRawMEMSGyro";
};
