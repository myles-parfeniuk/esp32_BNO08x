/**
 * @file BNO08xRptGameRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptGameRV
 *
 * @brief Class to represent game rotation vector reports. (See Ref. Manual 6.5.19)
 */
class BNO08xRptGameRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptGameRV(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRptRVGeneric(info)
        {
        }

        bool tare(bool x = true, bool y = true, bool z = true);
        bool tare_persist();
        void tare_clear();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptGameRV";
};
