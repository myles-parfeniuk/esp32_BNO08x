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
        BNO08xRptRVGeneric(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }
        bool tare(bool x, bool y, bool z, sh2_TareBasis_t basis);
        bno08x_quat_t data;
        static const constexpr char* TAG = "BNO08xRptRVGeneric";
};
