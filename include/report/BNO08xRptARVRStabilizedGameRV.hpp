/**
 * @file BNO08xRptARVRStabilizedGameRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptARVRStabilizedGameRV
 *
 * @brief Class to represent ARVR stabilized rotation vector reports. (See Ref. Manual 6.5.43)
 */
class BNO08xRptARVRStabilizedGameRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptARVRStabilizedGameRV(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRptRVGeneric(info)
        {
        }

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptARVRStabilizedGameRV";
};
