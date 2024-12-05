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
        BNO08xRptARVRStabilizedRV(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRptRVGeneric(info)
        {
        }

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptARVRStabilizedRV";
};
