/**
 * @file BNO08xTapDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent tap detector reports. (See Ref. Manual 6.5.27)
 */
class BNO08xTapDetector : public BNO08xRpt
{
    public:
        BNO08xTapDetector(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bool enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bno08x_tap_detector_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_tap_detector_t data;
        static const constexpr char* TAG = "BNO08xTapDetector";
};
