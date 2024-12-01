/**
 * @file BNO08xRptARVRStabilizedRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptARVRStabilizedRV
 *
 * @brief Class to represent ARVR stabilized rotation vector reports. (See Ref. Manual 6.5.42)
 */
class BNO08xRptARVRStabilizedRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptARVRStabilizedRV(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRptRVGeneric(imu, report_ID, period_us, rpt_bit)
        {
        }

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptARVRStabilizedRV";
};
