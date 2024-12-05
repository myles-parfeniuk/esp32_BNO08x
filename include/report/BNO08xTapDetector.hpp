/**
 * @file BNO08xTapDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xTapDetector
 *
 * @brief Class to represent tap detector reports. (See Ref. Manual 6.5.27)
 */
class BNO08xTapDetector : public BNO08xRpt
{
    public:
        BNO08xTapDetector(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bool enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        bno08x_tap_detector_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_tap_detector_t data;
        static const constexpr char* TAG = "BNO08xTapDetector";
};
