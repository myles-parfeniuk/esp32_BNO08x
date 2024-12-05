/**
 * @file BNO08xRptRVGeneric.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptRVGeneric
 *
 * @brief Class to represent rotation vector reports.
 */
class BNO08xRptRVGeneric : public BNO08xRpt
{
    public:
        bno08x_quat_t get_quat();
        bno08x_euler_angle_t get_euler(bool in_degrees = true);

    protected:
        BNO08xRptRVGeneric(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }
        bool tare(bool x, bool y, bool z, sh2_TareBasis_t basis);
        bno08x_quat_t data;
        static const constexpr char* TAG = "BNO08xRptRVGeneric";
};
