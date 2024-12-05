/**
 * @file BNO08xRptRVGeomag.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptRVGeomag
 *
 * @brief Class to represent geomagnetic rotation vector reports. (See Ref. Manual 6.5.20)
 */
class BNO08xRptRVGeomag : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptRVGeomag(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRptRVGeneric(info)
        {
        }

        bool tare(bool x = true, bool y = true, bool z = true);
        bool tare_persist();
        void tare_clear();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptRVGeomag";
};
