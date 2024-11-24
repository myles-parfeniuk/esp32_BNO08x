#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent rotation vector reports.
 */
class BNO08xRptRVGeneric : public BNO08xRpt
{
    public:
        bno08x_quat_t get_quat();
        bno08x_euler_angle_t get_euler(bool in_degrees = true);

    protected:
        BNO08xRptRVGeneric(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }
        bno08x_quat_t data;
        static const constexpr char* TAG = "BNO08xRptRVGeneric";
};
