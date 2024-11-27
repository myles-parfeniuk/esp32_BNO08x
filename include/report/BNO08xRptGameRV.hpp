/**
 * @file BNO08xRptGameRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @brief Class to represent game rotation vector reports. (See Ref. Manual 6.5.19)
 */
class BNO08xRptGameRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptGameRV(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRptRVGeneric(imu, report_ID, period_us, rpt_bit)
        {
        }

        bool tare(bool x = true, bool y = true, bool z = true);
        bool tare_persist();
        void tare_clear();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        static const constexpr char* TAG = "BNO08xRptGameRV";
};
