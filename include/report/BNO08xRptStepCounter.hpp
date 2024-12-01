/**
 * @file BNO08xRptStepCounter.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptStepCounter
 *
 * @brief Class to represent step counter reports. (See Ref. Manual 6.5.29)
 */
class BNO08xRptStepCounter : public BNO08xRpt
{
    public:
        BNO08xRptStepCounter(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bno08x_step_counter_t get();
        uint32_t get_total_steps();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_step_counter_t data; ///< Most recent report data, doesn't account for step rollover.
        uint32_t step_accumulator =
                0UL; ///< Every time step count rolls over, the previous steps are accumulated here such that the total steps can always be calculated.
        static const constexpr char* TAG = "BNO08xRptStepCounter";
};
