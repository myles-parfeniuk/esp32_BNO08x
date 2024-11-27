/**
 * @file BNO08xRptARVRStabilizedGameRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @brief Class to represent ARVR stabilized rotation vector reports. (See Ref. Manual 6.5.43)
 */
class BNO08xRptARVRStabilizedGameRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptARVRStabilizedGameRV(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRptRVGeneric(imu, report_ID, period_us, rpt_bit)
        {
        }

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptARVRStabilizedGameRV";
};
